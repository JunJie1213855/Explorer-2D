"""ROS adapter for the independent grid world and simulated Nav2 action."""
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import threading
import time
import uuid

import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path as PathMessage
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .world import load_world


class GridSimulator(Node):
    def __init__(self):
        super().__init__('grid_simulator')
        defaults = {
            'map_file': '', 'algorithm': 'none', 'resolution': 0.0,
            'start_x': -1, 'start_y': -1, 'sensor_range': 1.5,
            'speed': 0.8, 'angular_speed': 1.5, 'inflation_radius': 0.0,
            'max_duration': 180.0, 'stall_timeout': 45.0,
            'coverage_target': 0.98, 'results_dir': 'results/grid_benchmark',
        }
        self.settings = {key: self.declare_parameter(key, value).value
                         for key, value in defaults.items()}
        s = self.settings
        for key in ('sensor_range', 'speed', 'angular_speed', 'max_duration', 'stall_timeout'):
            if not math.isfinite(s[key]) or s[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        if not 0 < s['coverage_target'] <= 1:
            raise ValueError('coverage_target must be in (0, 1]')
        if not math.isfinite(s['inflation_radius']) or s['inflation_radius'] < 0:
            raise ValueError('inflation_radius must be finite and nonnegative')
        if (s['start_x'] < 0) != (s['start_y'] < 0):
            raise ValueError('start_x and start_y must be supplied together')
        start = None if s['start_x'] < 0 else [s['start_x'], s['start_y']]
        self.world = load_world(s['map_file'], s['resolution'] or None, start)
        self.world.observe(s['sensor_range'])
        self.lock = threading.RLock()
        self.finished = False
        self.reason = 'interrupted'
        self.started = time.monotonic()
        self.last_tick = self.started
        self.last_progress = self.started
        self.best_coverage = self.world.coverage
        self.task = None
        self.tasks = []
        self.goals = []
        self.samples = []
        self.positions = []
        self.cmd = Twist()
        self.cmd_time = 0.0
        self.rrt_enabled = False
        self.last_map = 0.0
        self.last_metrics = 0.0
        self.last_trajectory = 0.0
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.output = (Path(s['results_dir']).expanduser().resolve()
                       / f"{s['algorithm']}_{stamp}_{uuid.uuid4().hex[:6]}")
        self.output.mkdir(parents=True, exist_ok=False)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', latched)
        self.global_pub = self.create_publisher(OccupancyGrid, '/global_costmap/costmap', latched)
        self.local_pub = self.create_publisher(OccupancyGrid, '/local_costmap/costmap', latched)
        self.truth_pub = self.create_publisher(OccupancyGrid, '/benchmark/truth', latched)
        self.path_pub = self.create_publisher(PathMessage, '/benchmark/path', latched)
        self.metrics_pub = self.create_publisher(String, '/benchmark/metrics', latched)
        self.enable_pub = self.create_publisher(Bool, '/robot1/rrt/enable', 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        transforms = []
        for child in ('base_link', 'robot1/base_footprint'):
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'base_footprint'
            transform.child_frame_id = child
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        self.static_tf.sendTransform(transforms)
        group = ReentrantCallbackGroup()
        self.create_subscription(Twist, '/cmd_vel', self.on_velocity, 10, callback_group=group)
        self.server = ActionServer(
            self, NavigateToPose, '/navigate_to_pose', execute_callback=self.execute,
            goal_callback=self.accept_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
            handle_accepted_callback=self.start_goal, callback_group=group,
        )
        self.create_timer(0.05, self.tick, callback_group=group)
        self.get_logger().info(f"Matrix {self.world.width}x{self.world.height}; results: {self.output}")

    def on_velocity(self, msg):
        with self.lock:
            self.cmd, self.cmd_time = msg, time.monotonic()

    def accept_goal(self, request):
        p = request.pose.pose.position
        if request.pose.header.frame_id not in ('map', '') or not all(
                math.isfinite(v) for v in (p.x, p.y)):
            return GoalResponse.REJECT
        with self.lock:
            if self.finished or not self.world.inside(self.world.world_cell(p.x, p.y)):
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def start_goal(self, handle):
        with self.lock:
            if self.task is not None and self.task['status'] == 'active':
                self.task['status'] = 'preempted'
                self.task['record']['status'] = 'preempted'
                self.task['done'].set_result(None)
            p = handle.request.pose.pose.position
            goal = self.world.world_cell(p.x, p.y)
            record = {'elapsed_s': time.monotonic() - self.started,
                      'x': p.x, 'y': p.y, 'cell_x': goal[0], 'cell_y': goal[1],
                      'status': 'active'}
            task = {'handle': handle, 'goal': goal, 'status': 'active',
                    'path': None, 'record': record, 'done': Future(executor=self.executor)}
            self.goals.append(record)
            self.task = task
            self.tasks.append(task)
            # Bind before scheduling; older execute callbacks must not move a new goal.
            handle._grid_task = task
        handle.execute()

    async def execute(self, handle):
        task = handle._grid_task
        await task['done']
        with self.lock:
            task['record']['status'] = task['status']
            if task['status'] == 'succeeded':
                handle.succeed()
            elif task['status'] == 'canceled':
                handle.canceled()
            else:
                handle.abort()
        return NavigateToPose.Result()

    def finish_actions(self):
        for task in self.tasks:
            if task['status'] == 'active' and self.finished:
                task['status'] = 'interrupted'
            if task['status'] != 'active' and not task['done'].done():
                task['record']['status'] = task['status']
                task['done'].set_result(None)

    def pose(self):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = self.world.x, self.world.y
        pose.pose.orientation.z = math.sin(self.world.yaw / 2)
        pose.pose.orientation.w = math.cos(self.world.yaw / 2)
        return pose

    def navigate(self, dt):
        task, world = self.task, self.world
        if task is None or task['status'] != 'active':
            if time.monotonic() - self.cmd_time < 0.5:
                angular = self.cmd.angular.z
                linear = self.cmd.linear.x
                if not all(math.isfinite(v) for v in (angular, linear)):
                    return
                world.yaw += max(-self.settings['angular_speed'], min(
                    self.settings['angular_speed'], angular)) * dt
                linear = max(-self.settings['speed'], min(self.settings['speed'], linear))
                world.move_to(world.x + linear * math.cos(world.yaw) * dt,
                              world.y + linear * math.sin(world.yaw) * dt)
            return
        # Check the current perceived grid on every step; hidden truth is used only
        # by sensing and swept-point collision checking, never by A*.
        costs = world.costmap(self.settings['inflation_radius'])
        if task['path'] is None or any(costs[y, x] == 100 for x, y in task['path']):
            task['path'] = world.plan(task['goal'], self.settings['inflation_radius'])
        if task['path'] is None:
            task['status'] = 'aborted'
            return
        if not task['path']:
            task['status'] = 'succeeded'
            task['record']['status'] = 'succeeded'
            return
        x, y = world.cell_center(task['path'][0])
        dx, dy = x - world.x, y - world.y
        length = math.hypot(dx, dy)
        if length < 1e-8:
            task['path'].pop(0)
            return
        desired = math.atan2(dy, dx)
        delta = math.atan2(math.sin(desired - world.yaw), math.cos(desired - world.yaw))
        turn = self.settings['angular_speed'] * dt
        world.yaw += max(-turn, min(turn, delta))
        if abs(delta) > 0.05:
            return
        step = min(length, self.settings['speed'] * dt)
        if not world.move_to(world.x + dx / length * step, world.y + dy / length * step):
            task['path'] = None
        elif step >= length:
            task['path'].pop(0)

    def grid_message(self, data):
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.world.resolution
        msg.info.width, msg.info.height = self.world.width, self.world.height
        msg.info.origin.position.x, msg.info.origin.position.y = self.world.origin
        msg.info.origin.orientation.w = 1.0
        msg.data = data.ravel().tolist()
        return msg

    def metrics(self):
        counts = {status: sum(g['status'] == status for g in self.goals)
                  for status in ('succeeded', 'aborted', 'preempted', 'canceled', 'active', 'interrupted')}
        return {'elapsed_s': time.monotonic() - self.started,
                'coverage': float(self.world.coverage), 'path_length_m': self.world.distance,
                'collision_attempts': self.world.collisions, 'goals': len(self.goals), **counts}

    def tick(self):
        with self.lock:
            if self.finished:
                pose = self.pose()
                transform = TransformStamped()
                transform.header = pose.header
                transform.child_frame_id = 'base_footprint'
                transform.transform.translation.x = self.world.x
                transform.transform.translation.y = self.world.y
                transform.transform.rotation = pose.pose.orientation
                self.tf.sendTransform(transform)
                return
            now = time.monotonic()
            dt = min(now - self.last_tick, 0.2)
            self.last_tick = now
            if self.task is not None and self.task['status'] == 'active':
                if self.task['handle'].is_cancel_requested:
                    self.task['status'] = 'canceled'
            self.navigate(dt)
            self.finish_actions()
            self.world.observe(self.settings['sensor_range'])
            if self.task is not None and self.task['status'] == 'active':
                feedback = NavigateToPose.Feedback()
                feedback.current_pose = self.pose()
                feedback.distance_remaining = float(len(self.task['path'] or []) * self.world.resolution)
                elapsed = now - self.started - self.task['record']['elapsed_s']
                feedback.navigation_time.sec = int(elapsed)
                feedback.navigation_time.nanosec = int((elapsed % 1) * 1e9)
                self.task['handle'].publish_feedback(feedback)
            pose = self.pose()
            transform = TransformStamped()
            transform.header = pose.header
            transform.child_frame_id = 'base_footprint'
            transform.transform.translation.x = self.world.x
            transform.transform.translation.y = self.world.y
            transform.transform.rotation = pose.pose.orientation
            self.tf.sendTransform(transform)
            if now - self.last_trajectory >= 0.2:
                self.last_trajectory = now
                self.positions.append((now - self.started, self.world.x, self.world.y, self.world.yaw))
                if not hasattr(self, 'trajectory'):
                    self.trajectory = PathMessage()
                self.trajectory.header = pose.header
                self.trajectory.poses.append(pose)
                self.path_pub.publish(self.trajectory)
            if now - self.last_map >= 0.2:
                self.last_map = now
                self.map_pub.publish(self.grid_message(self.world.known))
                costmap = self.grid_message(self.world.costmap(self.settings['inflation_radius']))
                self.global_pub.publish(costmap)
                self.local_pub.publish(costmap)
                self.truth_pub.publish(self.grid_message(self.world.truth))
            if self.settings['algorithm'] == 'rrt' and not self.rrt_enabled:
                if self.enable_pub.get_subscription_count():
                    self.enable_pub.publish(Bool(data=True))
                    self.rrt_enabled = True
            coverage = self.world.coverage
            if coverage > self.best_coverage:
                self.best_coverage, self.last_progress = coverage, now
            if now - self.last_metrics >= 1.0:
                self.last_metrics = now
                self.samples.append(self.metrics())
                self.metrics_pub.publish(String(data=json.dumps(self.samples[-1])))
            if coverage >= self.settings['coverage_target']:
                self.reason, self.finished = 'coverage_target', True
            elif now - self.started >= self.settings['max_duration']:
                self.reason, self.finished = 'timeout', True
            elif now - self.last_progress >= self.settings['stall_timeout']:
                self.reason, self.finished = 'stalled', True

    def save(self):
        with self.lock:
            self.finished = True
            self.finish_actions()
            for goal in self.goals:
                if goal['status'] == 'active':
                    goal['status'] = 'interrupted'
            summary = {
                'algorithm': self.settings['algorithm'], 'stop_reason': self.reason,
                'settings': self.settings, 'map_file': str(Path(self.settings['map_file']).resolve()),
                'grid_shape': list(self.world.truth.shape),
                'resolution': self.world.resolution, 'start_cell': list(self.world.start),
                'reachable_free_cells': len(self.world.reachable),
                'model': 'point robot, 4-connected A*, 360-degree occluded range sensor; wall time',
                **self.metrics(),
            }
            (self.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
            self.samples.append(self.metrics())
            for filename, rows, fields in (
                ('metrics.csv', self.samples, list(self.samples[-1])),
                ('goals.csv', self.goals, ['elapsed_s', 'x', 'y', 'cell_x', 'cell_y', 'status']),
                ('trajectory.csv', [dict(zip(('elapsed_s', 'x', 'y', 'yaw'), p)) for p in self.positions],
                 ['elapsed_s', 'x', 'y', 'yaw']),
            ):
                with (self.output / filename).open('w', newline='') as stream:
                    writer = csv.DictWriter(stream, fieldnames=fields)
                    writer.writeheader()
                    writer.writerows(rows)
            np.save(self.output / 'observed.npy', self.world.known)
            self.get_logger().info(
                f"Stopped: {self.reason}; coverage={summary['coverage']:.1%}; "
                f"distance={self.world.distance:.2f}m; results={self.output}")

    def destroy_node(self):
        # Node.destroy_node() does not destroy action waitables on Humble.
        # Release their goal/result handles before shutting down the ROS context.
        self.server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = GridSimulator()
        executor.add_node(node)
        while rclpy.ok() and not node.finished:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.save()
            executor.remove_node(node)
        # Let action execution/result futures finish before destroying publishers.
        if rclpy.ok():
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
        executor.shutdown(timeout_sec=3.0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
