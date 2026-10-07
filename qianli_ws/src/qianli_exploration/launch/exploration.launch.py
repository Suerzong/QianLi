"""Frontier exploration; requires an existing live SLAM and Nav2 session."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = Path(get_package_share_directory('qianli_exploration')) / 'config/exploration.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('report_file', default_value=''),
        DeclareLaunchArgument('max_duration_s', default_value='900.0'),
        Node(package='qianli_exploration', executable='frontier_explorer.py',
             name='qianli_frontier_explorer', output='screen',
             parameters=[str(config), {
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'max_duration_s': ParameterValue(LaunchConfiguration('max_duration_s'), value_type=float),
                 'report_file': ParameterValue(LaunchConfiguration('report_file'), value_type=str),
             }]),
    ])
