"""Expose a Nav2 ``NavigateToPose`` action server over topic-based navigation.

The Explore-2D planners (hgvd, rrt, mrtsp) reach the robot through the
``nav2_msgs/action/NavigateToPose`` action. Stacks such as
``astar_global_node`` + ``neupan_cpp_node`` instead take a goal pose on
``/goal_pose`` and report arrival on ``/neupan_arrive``. This node joins
the two interfaces so the explorers do not need to change.

Arrival detection relies on NeuPAN's periodic ``Bool``: it publishes
``False`` while tracking a path and ``True`` once the path end is
reached (including while idle). A goal is only reported as succeeded
after a ``False`` has been seen since the goal was forwarded, so a stale
``True`` from a previous goal cannot end the new one early.
"""
from __future__ import annotations

import math
import threading
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import TransformException

POLL_PERIOD_S = 0.05


class NavigateBridge(Node):
    def __init__(self):
        super().__init__('navigate_bridge')
        self.action_name = str(self.declare_parameter(
            'navigate_action', 'navigate_to_pose').value)
        self.goal_topic = str(self.declare_parameter(
            'goal_pose_topic', '/goal_pose').value)
        self.arrive_topic = str(self.declare_parameter(
            'arrive_topic', '/neupan_arrive').value)
        self.path_topic = str(self.declare_parameter(
            'path_topic', '/initial_path').value)
        self.cmd_topic = str(self.declare_parameter(
            'cmd_vel_topic', '/cmd_vel').value)
        self.map_frame = str(self.declare_parameter('map_frame', 'map').value)
        self.base_frame = str(self.declare_parameter('base_frame', 'base_link').value)
        self.goal_timeout = float(self.declare_parameter('goal_timeout_s', 120.0).value)
        self.feedback_period = float(self.declare_parameter('feedback_period_s', 0.3).value)
        self.cancel_stop_duration = float(self.declare_parameter(
            'cancel_stop_duration_s', 0.5).value)
        self.clear_path_on_cancel = bool(self.declare_parameter(
            'clear_path_on_cancel', True).value)
        self.arm_warning = float(self.declare_parameter('unaccepted_goal_warn_s', 5.0).value)

        self._lock = threading.Lock()
        self._arrived = False
        self._arrive_armed = False
        self._goal_seq = 0

        self._goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self._path_pub = self.create_publisher(Path, self.path_topic, 10)
        self._cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_subscription(Bool, self.arrive_topic, self._on_arrive, 10)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._server = ActionServer(
            self,
            NavigateToPose,
            self.action_name,
            execute_callback=self._execute,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info(
            f'navigate bridge ready: {self.action_name} -> '
            f'{self.goal_topic}, arrival on {self.arrive_topic}')

    # -- inputs ---------------------------------------------------------

    def _on_arrive(self, msg: Bool):
        with self._lock:
            if not msg.data:
                self._arrive_armed = True
            self._arrived = bool(msg.data)

    def _new_goal(self) -> int:
        with self._lock:
            self._goal_seq += 1
            self._arrived = False
            self._arrive_armed = False
            return self._goal_seq

    # -- action server --------------------------------------------------

    def _execute(self, goal_handle):
        request = goal_handle.request
        pose = request.pose
        if not pose.header.frame_id:
            pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()

        seq = self._new_goal()
        self._goal_pub.publish(pose)
        self.get_logger().info(
            f'goal ({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}) '
            f'in {pose.header.frame_id} forwarded to {self.goal_topic}')

        start = self.get_clock().now()
        last_feedback = start
        warned = False
        result = NavigateToPose.Result()

        while rclpy.ok():
            with self._lock:
                preempted = seq != self._goal_seq
                arrived = self._arrived and self._arrive_armed
                armed = self._arrive_armed

            if preempted:
                self.get_logger().info('goal superseded by a newer one')
                goal_handle.abort()
                return result

            if goal_handle.is_cancel_requested:
                self._stop_motion(seq)
                goal_handle.canceled()
                self.get_logger().info('goal canceled')
                return result

            if arrived:
                goal_handle.succeed()
                elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
                self.get_logger().info(f'goal reached in {elapsed:.1f} s')
                return result

            now = self.get_clock().now()
            if self.goal_timeout > 0.0 and \
                    (now - start).nanoseconds / 1e9 > self.goal_timeout:
                self._stop_motion(seq)
                goal_handle.abort()
                self.get_logger().warning(
                    f'goal timed out after {self.goal_timeout:.1f} s '
                    f'(arrival signal {"seen" if armed else "never seen"})')
                return result

            if not armed and not warned and \
                    (now - start).nanoseconds / 1e9 > self.arm_warning:
                warned = True
                self.get_logger().warning(
                    f'no navigation progress yet: {self.arrive_topic} has not '
                    f'reported False since the goal was forwarded '
                    f'(did {self.goal_topic} produce a path?)')

            if (now - last_feedback).nanoseconds / 1e9 >= self.feedback_period:
                last_feedback = now
                self._publish_feedback(goal_handle, pose, start, now)

            time.sleep(POLL_PERIOD_S)

        goal_handle.abort()
        return result

    # -- helpers --------------------------------------------------------

    def _publish_feedback(self, goal_handle, goal_pose, start, now):
        feedback = NavigateToPose.Feedback()
        feedback.navigation_time = (now - start).to_msg()
        try:
            transform = self._tf_buffer.lookup_transform(
                goal_pose.header.frame_id, self.base_frame, Time())
        except TransformException:
            pass
        else:
            translation = transform.transform.translation
            feedback.current_pose.header.frame_id = goal_pose.header.frame_id
            feedback.current_pose.header.stamp = now.to_msg()
            feedback.current_pose.pose.position.x = translation.x
            feedback.current_pose.pose.position.y = translation.y
            feedback.current_pose.pose.orientation = transform.transform.rotation
            feedback.distance_remaining = float(math.hypot(
                goal_pose.pose.position.x - translation.x,
                goal_pose.pose.position.y - translation.y))
        goal_handle.publish_feedback(feedback)

    def _stop_motion(self, seq: int):
        """Stop NeuPAN: clear its reference path, then push zero velocity."""
        if self.clear_path_on_cancel:
            path = Path()
            path.header.frame_id = self.map_frame
            path.header.stamp = self.get_clock().now().to_msg()
            self._path_pub.publish(path)
        deadline = time.monotonic() + max(self.cancel_stop_duration, 0.0)
        twist = Twist()
        while time.monotonic() < deadline:
            with self._lock:
                if seq != self._goal_seq:
                    return
            self._cmd_pub.publish(twist)
            time.sleep(POLL_PERIOD_S)


def main(args=None):
    rclpy.init(args=args)
    node = NavigateBridge()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
