"""Saved map localization only; mutually exclusive with slam_toolbox."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    map_file = Path(LaunchConfiguration('map_file').perform(context)).expanduser()
    if not map_file.is_file():
        raise RuntimeError(f'Saved map YAML does not exist: {map_file}')
    params = str(Path(get_package_share_directory('qianli_navigation')) / 'config/nav2_params.yaml')
    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[params, {'yaml_filename': str(map_file.resolve()), 'use_sim_time': True}], output='screen'),
        Node(package='nav2_amcl', executable='amcl', name='amcl', parameters=[params], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
             parameters=[{'use_sim_time': True, 'autostart': True, 'bond_timeout': 10.0,
                          'node_names': ['map_server', 'amcl']}], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('map_file'), OpaqueFunction(function=setup)])
