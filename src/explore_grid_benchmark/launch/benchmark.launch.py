"""Launch one unmodified explorer against a matrix-world navigation simulator."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown, matches_action
from launch.events.process import ShutdownProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def configure(context, sequence=None, include_rviz=True):
    value = lambda name: LaunchConfiguration(name).perform(context)
    share = Path(get_package_share_directory('explore_grid_benchmark'))
    algorithm = sequence[0] if sequence else value('algorithm')
    params = {'map_file': value('map_file'), 'algorithm': algorithm, 'use_sim_time': False,
              'results_dir': value('results_dir')}
    for key in ('resolution', 'sensor_range', 'speed', 'angular_speed', 'inflation_radius',
                'max_duration', 'stall_timeout', 'coverage_target'):
        params[key] = float(value(key))
    for key in ('start_x', 'start_y'):
        params[key] = int(value(key))
    simulator = Node(package='explore_grid_benchmark', executable='grid_simulator',
                     parameters=[params], output='screen')
    actions = [simulator]
    common = {'use_sim_time': False, 'robot_base_frame': 'base_footprint'}
    remaps = [('/tf', '/tf'), ('/tf_static', '/tf_static')]
    if algorithm == 'frontier':
        explorer = Node(package='frontier_exploration', executable='frontier_planner',
                        parameters=[common, {'map_revolution': float(value('resolution'))
                                             or _map_resolution(value('map_file')),
                                             'obstacle_inflation': float(value('inflation_radius'))}],
                        output='screen')
    elif algorithm == 'rrt':
        explorer = Node(package='rrt_explore', executable='rrt', namespace='robot1', name='rrt',
                        remappings=remaps,
                        parameters=[common, {'map_topic': '/map',
                            'costmap_topic': '/global_costmap/costmap',
                            'robot_frame_prefix': 'robot', 'robot_count': 1,
                            'rate': 30.0, 'eta': 0.5, 'info_radius': 1.0,
                            'costmap_pixel_threshold': 100.0}], output='screen')
    elif algorithm == 'explore_lite':
        explorer = Node(package='explore_lite', executable='explore', name='explore_node',
                        parameters=[common, {'costmap_topic': '/map',
                            'costmap_updates_topic': '/map_updates', 'visualize': True,
                            'return_to_init': False, 'planner_frequency': 1.0,
                            'progress_timeout': 15.0, 'potential_scale': 3.0,
                            'gain_scale': 1.0, 'min_frontier_size': 0.2}], output='screen')
    elif algorithm == 'mrtsp':
        config = Path(get_package_share_directory('mrtsp_exploration_ros2')) / 'config/params.yaml'
        explorer = Node(package='mrtsp_exploration_ros2', executable='mrtsp_explorer',
                        name='mrtsp_explorer', parameters=[str(config), common], output='screen')
    else:
        explorer = None
    if explorer:
        actions.append(TimerAction(period=1.0, actions=[explorer]))
    if sequence and explorer is not None:
        state = {'sim_done': False, 'explorer_done': False}

        def following(context):
            if len(sequence) > 1:
                return configure(context, sequence=sequence[1:], include_rviz=False)
            return [EmitEvent(event=Shutdown(reason='All grid benchmarks finished'))]

        def sim_exited(event, context):
            state['sim_done'] = True
            if event.returncode:
                return [EmitEvent(event=Shutdown(reason='Grid simulator failed'))]
            if state['explorer_done']:
                return following(context)
            return [EmitEvent(event=ShutdownProcess(
                process_matcher=matches_action(explorer)))]

        def explorer_exited(event, context):
            state['explorer_done'] = True
            if state['sim_done']:
                return following(context)
            return [EmitEvent(event=ShutdownProcess(
                process_matcher=matches_action(simulator)))]

        actions.extend([
            RegisterEventHandler(OnProcessExit(target_action=simulator, on_exit=sim_exited)),
            RegisterEventHandler(OnProcessExit(target_action=explorer, on_exit=explorer_exited)),
        ])
    else:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=simulator,
            on_exit=[EmitEvent(event=Shutdown(reason='Grid benchmark finished'))])))
    if include_rviz and value('rviz').lower() == 'true':
        actions.append(Node(package='rviz2', executable='rviz2',
                            arguments=['-d', str(share / 'rviz/benchmark.rviz')],
                            parameters=[{'use_sim_time': False}], output='screen'))
    return actions


def _map_resolution(path):
    # Resolve YAML metadata without importing the source package into launch's Python path.
    import yaml
    if Path(path).suffix.lower() in ('.yaml', '.yml'):
        with open(path) as stream:
            return float(yaml.safe_load(stream).get('resolution', 0.2))
    return 0.2


def generate_launch_description(sequence=None):
    share = Path(get_package_share_directory('explore_grid_benchmark'))
    arguments = [
        DeclareLaunchArgument('algorithm', default_value='mrtsp',
                              choices=['frontier', 'rrt', 'explore_lite', 'mrtsp', 'none']),
        DeclareLaunchArgument('map_file', default_value=str(share / 'maps/rooms.yaml')),
        DeclareLaunchArgument('resolution', default_value='0.0', description='0 uses map metadata or 0.2 m/cell'),
        DeclareLaunchArgument('start_x', default_value='-1', description='Start column; -1 uses map metadata'),
        DeclareLaunchArgument('start_y', default_value='-1', description='Start row; -1 uses map metadata'),
        DeclareLaunchArgument('sensor_range', default_value='1.5'),
        DeclareLaunchArgument('speed', default_value='0.8'),
        DeclareLaunchArgument('angular_speed', default_value='1.5'),
        DeclareLaunchArgument('inflation_radius', default_value='0.0'),
        DeclareLaunchArgument('max_duration', default_value='180.0'),
        DeclareLaunchArgument('stall_timeout', default_value='45.0'),
        DeclareLaunchArgument('coverage_target', default_value='0.98'),
        DeclareLaunchArgument('results_dir', default_value=str(Path.cwd() / 'results/grid_benchmark')),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('domain_id', default_value='71', description='Dedicated ROS domain for this benchmark'),
    ]
    if sequence:
        arguments = arguments[1:]
    return LaunchDescription(arguments + [
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        OpaqueFunction(function=lambda context: configure(context, sequence=sequence)),
    ])
