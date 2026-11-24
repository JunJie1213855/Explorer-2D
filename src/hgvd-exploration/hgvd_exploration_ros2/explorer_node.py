"""ROS 2 map, TF and NavigateToPose adapter for HGVD."""
from contextlib import nullcontext
from dataclasses import dataclass
from functools import partial
from math import atan2, cos, floor, isfinite, sin, tau

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from hgvd.explore import Planner

from .timing import TimingCsv

RING_ANGLES = tuple(index * tau / 32 for index in range(32))


def yaw_of(rotation):
    return atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y),
                 1 - 2 * (rotation.y ** 2 + rotation.z ** 2))


@dataclass(frozen=True)
class GridGeometry:
    frame: str
    width: int
    height: int
    resolution: float
    x: float
    y: float
    yaw: float

    def grid_position(self, x, y):
        """Return continuous grid coordinates with integer cell centers."""
        dx, dy = x - self.x, y - self.y
        c, s = cos(self.yaw), sin(self.yaw)
        return ((c * dx + s * dy) / self.resolution - 0.5,
                (-s * dx + c * dy) / self.resolution - 0.5)

    def cell_center(self, cell):
        dx, dy = ((value + 0.5) * self.resolution for value in cell)
        c, s = cos(self.yaw), sin(self.yaw)
        return self.x + c * dx - s * dy, self.y + s * dx + c * dy


class HgvdExplorer(Node):
    def __init__(self):
        super().__init__('hgvd_explorer')
        defaults = {
            'map_topic': '/map', 'robot_base_frame': 'base_link',
            'navigate_action': 'navigate_to_pose', 'planning_period_s': 0.5,
            'local_radius_m': 5.0, 'local_sample_stride': 2, 'global_sample_stride': 8,
            'cluster_bandwidth_m': 2.0, 'frontier_spacing_m': 0.6,
            'obstacle_clearance_m': 0.0, 'occupied_threshold': 65,
            'min_goal_distance_m': 0.35, 'goal_switch_margin_m': 1.0, 'goal_timeout_s': 120.0,
            'failed_goal_retry_s': 30.0, 'visualize': True, 'navigation_enabled': True,
            'return_to_start': False, 'completion_hold_s': 5.0, 'timing_csv': '',
        }
        params = {name: self.declare_parameter(name, value).value
                  for name, value in defaults.items()}
        for name, value in params.items():
            if isinstance(value, (float, int)) and not isinstance(value, bool):
                allow_zero = name in ('obstacle_clearance_m', 'goal_switch_margin_m', 'min_goal_distance_m')
                if not isfinite(value) or value < 0 or (value == 0 and not allow_zero):
                    raise ValueError(f'Invalid {name}: {value}')
        if params['global_sample_stride'] <= params['local_sample_stride']:
            raise ValueError('global_sample_stride must be greater than local_sample_stride')
        if params['occupied_threshold'] > 100:
            raise ValueError('occupied_threshold must be at most 100')
        self.params = params
        self.timing = TimingCsv(params['timing_csv']) if params['timing_csv'] else None
        self.measure = self.timing.measure if self.timing is not None else nullcontext
        if self.timing is not None:
            self.get_logger().info(f'Recording HGVD timings to {self.timing.path}')
        self.planner = Planner(
            params['local_radius_m'], params['local_sample_stride'], params['global_sample_stride'],
            params['cluster_bandwidth_m'], params['obstacle_clearance_m'], params['frontier_spacing_m'],
            measure=self.measure,
        )
        self.geometry = None
        self.map_revision = 0
        self.excluded = {}
        self.goal = None
        self.goal_handle = None
        self.goal_pose = None
        self.canceling = False
        self.start_pose = None
        self.frontier_free_since = None
        self.exploration_complete = False
        self.returning = False
        self.return_done = False
        self.return_attempts = 0
        self.return_retry_at = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = (ActionClient(self, NavigateToPose, params['navigate_action'])
                    if params['navigation_enabled'] else None)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        live = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for qos in (latched, live):
            self.create_subscription(OccupancyGrid, params['map_topic'], self._on_map, qos)
        self.markers = (self.create_publisher(MarkerArray, '~/topology', latched)
                        if params['visualize'] else None)
        self.goal_marker = (self.create_publisher(Marker, '~/goal_marker', latched)
                            if params['visualize'] else None)
        self.marker_lifetime = Duration(seconds=max(5.0, 3 * params['planning_period_s'])).to_msg()
        callback = self._timed_plan if self.timing is not None else self._plan
        self.create_timer(params['planning_period_s'], callback)

    def _timed_plan(self):
        with self.measure('planning_tick_total'):
            self._plan()

    def _on_map(self, msg):
        info = msg.info
        orientation = info.origin.orientation
        geometry = GridGeometry(msg.header.frame_id, info.width, info.height, info.resolution,
                                info.origin.position.x, info.origin.position.y, yaw_of(orientation))
        if (not geometry.frame or not geometry.width or not geometry.height
                or not all(isfinite(v) for v in (geometry.resolution, geometry.x, geometry.y,
                                                  geometry.yaw))
                or geometry.resolution <= 0 or len(msg.data) != geometry.width * geometry.height):
            self.get_logger().warn('Ignoring invalid occupancy grid', throttle_duration_sec=5.0)
            return
        with self.measure('map_decode'):
            raw = np.asarray(msg.data, dtype=np.int8).reshape(geometry.height, geometry.width)
            grid = np.where(raw >= self.params['occupied_threshold'], 100,
                            np.where(raw == 0, 0, -1)).astype(np.int8)
        if geometry == self.geometry:
            if np.array_equal(self.planner.grid, grid):
                return
        else:
            self.geometry = geometry
            self.map_revision += 1
            self.excluded.clear()
            self.planner.grid = None
        self.planner.update_map(grid, geometry.resolution)

    def _plan(self):
        if self.return_done:
            return
        if self.geometry is None:
            self.get_logger().info('Waiting for occupancy grid', throttle_duration_sec=10.0)
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self.goal is not None:
            if self.goal_handle is not None:
                self._publish_goal_marker(self.goal_pose)
            if (self.goal_pose.header.frame_id != self.geometry.frame
                    or (not self.returning and self.goal_revision != self.map_revision)):
                self._cancel()
                return
            if now - self.goal_sent_at > self.params['goal_timeout_s']:
                if not self.canceling and not self.returning:
                    self.excluded[self.goal] = now + self.params['failed_goal_retry_s']
                self._cancel()
                return
            if self.returning:
                return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.geometry.frame, self.params['robot_base_frame'], Time(),
            ).transform
        except TransformException as error:
            self.frontier_free_since = None
            self.get_logger().info(f'Waiting for robot TF: {error}', throttle_duration_sec=10.0)
            return

        x, y = tf.translation.x, tf.translation.y
        yaw = yaw_of(tf.rotation)
        if not all(isfinite(v) for v in (x, y, yaw)):
            self.frontier_free_since = None
            return
        robot_position = self.geometry.grid_position(x, y)
        robot = tuple(floor(value + 0.5) for value in robot_position)
        if not self.planner.update_topology(robot):
            self.frontier_free_since = None
            self.get_logger().debug(f'Robot cell {robot} is outside known traversable space',
                                    throttle_duration_sec=5.0)
            return
        if self.nav is not None and self.params['return_to_start']:
            if self.start_pose is None:
                self.start_pose = (self.geometry.frame, x, y, yaw)
            if self.exploration_complete:
                self._return_to_start(robot, now)
                return
            if self.goal is not None or self.planner.frontiers.any():
                self.frontier_free_since = None
            elif self.frontier_free_since is None or now < self.frontier_free_since:
                self.frontier_free_since = now
            elif now - self.frontier_free_since >= self.params['completion_hold_s']:
                self.exploration_complete = True
                self.get_logger().info('Exploration complete; returning to the start pose')
                self._return_to_start(robot, now)
                return

        self.excluded = {cell: until for cell, until in self.excluded.items() if until > now}
        active = self.goal
        if active is not None:
            if not self.planner.links.contains(active):
                self._cancel()
                return
            if self.goal_handle is None or self.canceling:
                return

        with self.measure('viewpoint_selection'):
            targets = set(self.planner.viewpoints(
                robot, self.excluded, self.params['min_goal_distance_m'],
                robot_position=robot_position,
            ))
        if self.markers is not None:
            self._publish_markers(targets)
        if not targets:
            if active is None:
                self.get_logger().info('No reachable HGVD viewpoints meet the goal filters',
                                       throttle_duration_sec=10.0)
            return
        if self.nav is None:
            return
        with self.measure('goal_scoring'):
            costs = dict(self.planner.scores(robot, yaw - self.geometry.yaw,
                                            targets | ({active} if active is not None else set())))
            target = min(targets, key=lambda cell: (costs[cell], cell))
        if active is None:
            if self.nav.server_is_ready():
                self._send(target, robot, now)
            else:
                self.get_logger().info('Waiting for NavigateToPose server',
                                       throttle_duration_sec=10.0)
        elif target != active and costs[active] - costs[target] > self.params['goal_switch_margin_m']:
            self._cancel()

    def _return_to_start(self, robot, now):
        if self.markers is not None:
            self._publish_markers(set())
        if now < self.return_retry_at:
            return
        frame, x, y, _ = self.start_pose
        if self.geometry.frame != frame:
            self.get_logger().warn('Waiting for the recorded start map frame',
                                   throttle_duration_sec=10.0)
            return
        cell = tuple(floor(value + 0.5) for value in self.geometry.grid_position(x, y))
        if not self.planner.links.contains(cell):
            self.get_logger().warn('Start pose is outside reachable free space',
                                   throttle_duration_sec=10.0)
            return
        if not self.nav.server_is_ready():
            self.get_logger().info('Waiting for NavigateToPose to return to the start',
                                   throttle_duration_sec=10.0)
            return
        self.return_attempts += 1
        self._send(cell, robot, now, returning=True)

    def _send(self, cell, robot, now, *, returning=False):
        goal = NavigateToPose.Goal()
        if returning:
            frame, x, y, yaw = self.start_pose
        else:
            frame = self.geometry.frame
            x, y = self.geometry.cell_center(cell)
            yaw = atan2(cell[1] - robot[1], cell[0] - robot[0]) + self.geometry.yaw
        goal.pose.header.frame_id = frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = sin(yaw / 2), cos(yaw / 2)
        self.returning = returning
        self.goal = cell
        self.goal_pose = goal.pose
        self.goal_revision = self.map_revision
        self.goal_sent_at = now
        try:
            with self.measure('nav2_goal_dispatch'):
                self.nav.send_goal_async(goal).add_done_callback(self._on_goal_response)
        except (RuntimeError, ValueError) as error:
            self._finish(None, str(error))

    def _on_goal_response(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self._finish(None, 'goal rejected')
                return
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(self._on_result)
        except Exception as error:
            self._finish(None, str(error))
            return
        self._publish_goal_marker(self.goal_pose)
        if self.returning:
            self.get_logger().info('Navigating back to the recorded start pose')
        else:
            self.get_logger().info(f'Navigating to HGVD viewpoint {self.goal}')

    def _cancel(self):
        if self.goal_handle is None or self.canceling:
            return
        try:
            self.canceling = True
            handle = self.goal_handle
            handle.cancel_goal_async().add_done_callback(partial(self._on_cancel_response, handle))
        except RuntimeError as error:
            self.canceling = False
            self.get_logger().warn(f'Unable to cancel HGVD goal: {error}')

    def _on_cancel_response(self, handle, future):
        if handle is not self.goal_handle:
            return
        try:
            self.canceling = bool(future.result().goals_canceling)
        except Exception as error:
            self.canceling = False
            self.get_logger().warn(f'Unable to cancel HGVD goal: {error}')

    def _on_result(self, future):
        try:
            status = future.result().status
        except Exception as error:
            self._finish(None, str(error))
        else:
            self._finish(status)

    def _finish(self, status, error=None):
        cell = self.goal
        now = self.get_clock().now().nanoseconds / 1e9
        if self.returning:
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.return_done = True
                self.get_logger().info('Returned to the start pose')
            elif self.return_attempts >= 3:
                self.return_done = True
                self.get_logger().warn(f'Could not return to the start after 3 attempts: {error or status}')
            else:
                self.return_retry_at = now + self.params['failed_goal_retry_s']
                self.get_logger().warn(f'Return to the start failed; will retry: {error or status}')
        elif self.goal_revision == self.map_revision:
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.excluded[cell] = now + 2.0
            elif status != GoalStatus.STATUS_CANCELED or not self.canceling:
                self.excluded[cell] = now + self.params['failed_goal_retry_s']
                self.get_logger().warn(f'HGVD goal {cell} failed: {error or status}')
        self._publish_goal_marker(None)
        self.goal = self.goal_handle = self.goal_pose = None
        self.returning = False
        self.canceling = False

    def _marker(self, namespace, kind, rgb, alpha=1.0, *, frame=None):
        marker = Marker()
        marker.header.frame_id = frame if frame is not None else self.geometry.frame
        marker.ns, marker.type = namespace, kind
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b = rgb
        marker.color.a = alpha
        marker.lifetime = self.marker_lifetime
        return marker

    def _publish_markers(self, targets):
        with self.measure('rviz_markers'):
            self._publish_topology_markers(targets)

    def _publish_topology_markers(self, targets):
        graph = self.planner.graph
        direct_edges, astar_links = [], []
        for first, neighbors in graph.items():
            for second in neighbors:
                if first < second:
                    edges = direct_edges if self.planner.links.direct(first, second) else astar_links
                    edges.extend((first, second))

        markers = []
        for name, kind, cells, rgb, scale, height in (
            ('samples', Marker.POINTS, [sample.cell for sample in self.planner.samples],
             (0.16, 0.85, 1.0), 0.08, 0.09),
            ('centers', Marker.POINTS, self.planner.centers, (0.68, 0.36, 1.0), 0.14, 0.12),
            ('edges', Marker.LINE_LIST, direct_edges, (0.15, 0.66, 0.84), 0.022, 0.06),
            ('astar_links', Marker.LINE_LIST, astar_links, (0.87, 0.40, 0.81), 0.022, 0.06),
            ('viewpoints', Marker.POINTS, sorted(targets), (1.0, 0.65, 0.12), 0.12, 0.11),
        ):
            marker = self._marker(name, kind, rgb)
            marker.action = Marker.ADD if cells else Marker.DELETE
            marker.scale.x = marker.scale.y = scale
            marker.points = [Point(x=x, y=y, z=height)
                             for x, y in map(self.geometry.cell_center, cells)]
            markers.append(marker)

        rings = self._marker('gvd_radii', Marker.LINE_LIST, (0.20, 0.78, 0.66), alpha=0.55)
        rings.scale.x = 0.018
        for sample in self.planner.samples:
            x, y = self.geometry.cell_center(sample.cell)
            radius = sample.radius * self.geometry.resolution
            circle = [Point(x=x + radius * cos(angle), y=y + radius * sin(angle), z=0.04)
                      for angle in RING_ANGLES]
            for index, point in enumerate(circle):
                rings.points.extend((point, circle[(index + 1) % len(circle)]))
        rings.action = Marker.ADD if rings.points else Marker.DELETE
        self.markers.publish(MarkerArray(markers=[rings, *markers]))

    def _publish_goal_marker(self, pose):
        if self.goal_marker is None:
            return
        frame = pose.header.frame_id if pose is not None else self.geometry.frame
        marker = self._marker('active_goal', Marker.ARROW, (0.36, 1.0, 0.22), frame=frame)
        if pose is None:
            marker.action = Marker.DELETE
        else:
            marker.pose.position.x = pose.pose.position.x
            marker.pose.position.y = pose.pose.position.y
            marker.pose.position.z = 0.17
            marker.pose.orientation = pose.pose.orientation
            marker.scale.x, marker.scale.y, marker.scale.z = 0.34, 0.12, 0.12
        self.goal_marker.publish(marker)

    def destroy_node(self):
        if self.timing is not None:
            self.timing.close()
        if self.nav is not None:
            self.nav.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HgvdExplorer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
