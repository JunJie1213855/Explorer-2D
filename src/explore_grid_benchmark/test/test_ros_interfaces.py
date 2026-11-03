"""Opt-in ROS action contract test; run in a dedicated ROS domain."""
import os
import threading
import time
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get('EXPLORER_ROS_TESTS') != '1', reason='Set EXPLORER_ROS_TESTS=1 in a dedicated ROS domain')


def test_navigation_success_cancel_preempt_and_reject(tmp_path):
    import rclpy
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from explore_grid_benchmark.node import GridSimulator

    map_file = Path(__file__).resolve().parents[1] / 'maps/rooms.yaml'
    rclpy.init(args=['--ros-args', '-p', f'map_file:={map_file}',
                    '-p', f'results_dir:={tmp_path}', '-p', 'speed:=5.0',
                    '-p', 'angular_speed:=10.0', '-p', 'coverage_target:=1.0'])
    simulator = GridSimulator()
    client_node = rclpy.create_node('benchmark_contract_test')
    client = ActionClient(client_node, NavigateToPose, '/navigate_to_pose')
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(simulator)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def wait(future, timeout=10):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert future.done(), 'ROS action future timed out'
        return future.result()

    def send(cell):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = simulator.world.cell_center(cell)
        goal.pose.pose.orientation.w = 1.0
        return wait(client.send_goal_async(goal))

    try:
        assert client.wait_for_server(timeout_sec=5)
        first = send((7, 5))
        assert first.accepted
        assert wait(first.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED
        assert simulator.world.distance > 0.3
        assert simulator.world.coverage > 0.05
        canceled = send((35, 25))
        assert canceled.accepted
        wait(canceled.cancel_goal_async())
        assert wait(canceled.get_result_async()).status == GoalStatus.STATUS_CANCELED
        replaced = send((35, 25))
        replacement = send(simulator.world.world_cell(simulator.world.x, simulator.world.y))
        assert wait(replaced.get_result_async()).status == GoalStatus.STATUS_ABORTED
        assert wait(replacement.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED
        rejected = send((-10, -10))
        assert not rejected.accepted
        blocked = send((0, 0))
        assert wait(blocked.get_result_async()).status == GoalStatus.STATUS_ABORTED
    finally:
        simulator.save()
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        client.destroy()
        client_node.destroy_node()
        simulator.destroy_node()
        rclpy.shutdown()


def test_process_stops_and_saves_with_active_navigation(tmp_path, monkeypatch):
    import json
    import subprocess
    import sys
    import rclpy
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient

    # Avoid stale DDS endpoints from the preceding context in the same process.
    domain = (int(os.environ.get('ROS_DOMAIN_ID', '75')) + 10) % 101
    monkeypatch.setenv('ROS_DOMAIN_ID', str(domain))
    map_file = Path(__file__).resolve().parents[1] / 'maps/rooms.yaml'
    command = [sys.executable, '-c', 'from explore_grid_benchmark.node import main; main()',
               '--ros-args', '-p', f'map_file:={map_file}', '-p', f'results_dir:={tmp_path}',
               '-p', 'max_duration:=5.0', '-p', 'stall_timeout:=10.0']
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    node = rclpy.create_node('benchmark_shutdown_test')
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    try:
        assert client.wait_for_server(timeout_sec=5)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = 7.1, 5.1
        goal.pose.pose.orientation.w = 1.0
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=4)
        assert future.done() and future.result().accepted
        output, _ = process.communicate(timeout=8)
        assert process.returncode == 0, output
        assert 'Destroyable' not in output and 'Traceback' not in output, output
        summary = json.loads(next(tmp_path.glob('*/summary.json')).read_text())
        assert summary['stop_reason'] == 'timeout'
        assert summary['goals'] == 1
        assert summary['interrupted'] == 1
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()
