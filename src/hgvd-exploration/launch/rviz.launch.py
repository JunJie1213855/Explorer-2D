"""Open the HGVD workspace in RViz without starting navigation."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('hgvd_exploration_ros2'))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('fixed_frame', default_value='map'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='hgvd_rviz',
            arguments=['-d', str(share / 'rviz/hgvd.rviz'), '-f', LaunchConfiguration('fixed_frame')],
            parameters=[{
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
            remappings=[('/map', LaunchConfiguration('map_topic'))],
            output='screen',
        ),
    ])
