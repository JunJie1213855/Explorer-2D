"""Launch the HGVD explorer alongside an existing SLAM and Nav2 stack."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false'],
                              description='Open the HGVD RViz visualization'),
        DeclareLaunchArgument('robot_base_frame', default_value='base_link'),
        DeclareLaunchArgument('navigate_action', default_value='navigate_to_pose'),
        DeclareLaunchArgument('navigation_enabled', default_value='true',
                              description='Send Nav2 goals; set false to preview HGVD only'),
        DeclareLaunchArgument('return_to_start', default_value='false', choices=['true', 'false'],
                              description='Return to the initial pose when exploration finishes'),
        DeclareLaunchArgument('completion_hold_s', default_value='5.0',
                              description='Seconds without reachable frontiers before returning'),
        DeclareLaunchArgument('min_goal_distance_m', default_value='0.35',
                              description='Minimum viewpoint distance from the robot in meters'),
        Node(
            package='hgvd_exploration_ros2',
            executable='hgvd_explorer',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'map_topic': LaunchConfiguration('map_topic'),
                'robot_base_frame': LaunchConfiguration('robot_base_frame'),
                'navigate_action': LaunchConfiguration('navigate_action'),
                'navigation_enabled': ParameterValue(
                    LaunchConfiguration('navigation_enabled'), value_type=bool,
                ),
                'return_to_start': ParameterValue(
                    LaunchConfiguration('return_to_start'), value_type=bool,
                ),
                'completion_hold_s': ParameterValue(
                    LaunchConfiguration('completion_hold_s'), value_type=float,
                ),
                'min_goal_distance_m': ParameterValue(
                    LaunchConfiguration('min_goal_distance_m'), value_type=float,
                ),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('hgvd_exploration_ros2'), 'launch', 'rviz.launch.py',
            ])),
            condition=IfCondition(LaunchConfiguration('rviz')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_topic': LaunchConfiguration('map_topic'),
            }.items(),
        ),
    ])
