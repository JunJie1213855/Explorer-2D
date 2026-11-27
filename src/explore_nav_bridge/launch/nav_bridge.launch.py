"""Run the NavigateToPose-to-topic navigation bridge."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('navigate_action', default_value='navigate_to_pose'),
        DeclareLaunchArgument('goal_pose_topic', default_value='/goal_pose'),
        DeclareLaunchArgument('arrive_topic', default_value='/neupan_arrive'),
        DeclareLaunchArgument('path_topic', default_value='/initial_path'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('goal_timeout_s', default_value='120.0'),
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(
            package='explore_nav_bridge',
            executable='navigate_bridge',
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            parameters=[{
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'navigate_action': LaunchConfiguration('navigate_action'),
                'goal_pose_topic': LaunchConfiguration('goal_pose_topic'),
                'arrive_topic': LaunchConfiguration('arrive_topic'),
                'path_topic': LaunchConfiguration('path_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'map_frame': LaunchConfiguration('map_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'goal_timeout_s': ParameterValue(
                    LaunchConfiguration('goal_timeout_s'), value_type=float),
            }],
        ),
    ])
