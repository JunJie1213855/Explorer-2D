"""Exercise return-to-start against a NavigateToPose server."""
from math import cos, sin
from time import monotonic

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')

from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor

from hgvd_exploration_ros2.explorer_node import HgvdExplorer


def test_returns_once_to_exact_start_after_frontiers_disappear(monkeypatch):
    action_name = '/hgvd_return_to_start_test/navigate'
    rclpy.init(args=['--ros-args', '-p', f'navigate_action:={action_name}',
                    '-p', 'map_topic:=/hgvd_return_to_start_test/map',
                    '-p', 'return_to_start:=true',
                    '-p', 'completion_hold_s:=0.2', '-p', 'planning_period_s:=0.05',
                    '-p', 'min_goal_distance_m:=100.0', '-p', 'visualize:=false'])
    node = HgvdExplorer()
    server_node = rclpy.create_node('return_to_start_test')
    executor = SingleThreadedExecutor()
    executor.add_node(server_node)
    executor.add_node(node)
    received = []

    def execute(goal):
        received.append(goal.request.pose)
        goal.succeed()
        return NavigateToPose.Result()

    server = ActionServer(server_node, NavigateToPose, action_name, execute)
    tf = TransformStamped()
    tf.transform.translation.x, tf.transform.translation.y = 1.25, 1.4
    tf.transform.rotation.z, tf.transform.rotation.w = sin(0.3), cos(0.3)
    monkeypatch.setattr(node.tf_buffer, 'lookup_transform', lambda *_: tf)

    def publish_map(grid, origin_x):
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info.height, msg.info.width = grid.shape
        msg.info.resolution = 1.0
        msg.info.origin.position.x = origin_x
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.ravel().tolist()
        node._on_map(msg)

    def spin_for(seconds, until=lambda: False):
        deadline = monotonic() + seconds
        while monotonic() < deadline and not until():
            executor.spin_once(timeout_sec=0.05)

    try:
        grid = np.zeros((5, 8), dtype=np.int8)
        grid[:, -2:] = -1
        publish_map(grid, 0.0)
        spin_for(0.7)
        assert node.start_pose is not None
        assert node.nav.server_is_ready()
        assert not received and not node.exploration_complete

        # Move the robot and shift the grid origin as SLAM expands the map.
        tf.transform.translation.x = 3.5
        tf.transform.rotation.z, tf.transform.rotation.w = 0.0, 1.0
        publish_map(np.zeros((5, 9), dtype=np.int8), -1.0)
        spin_for(5, until=lambda: node.return_done)
        assert node.return_done
        spin_for(0.3)
        assert len(received) == 1
        assert received[0].header.frame_id == 'map'
        pose = received[0].pose
        assert (pose.position.x, pose.position.y) == (1.25, 1.4)
        assert pose.orientation.z == pytest.approx(sin(0.3))
        assert pose.orientation.w == pytest.approx(cos(0.3))
    finally:
        executor.shutdown()
        server.destroy()
        server_node.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()
