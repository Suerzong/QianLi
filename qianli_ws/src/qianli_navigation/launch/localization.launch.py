"""Saved map localization only; mutually exclusive with slam_toolbox."""
import math
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
    initial = {name: float(LaunchConfiguration(name).perform(context))
               for name in ('initial_x', 'initial_y', 'initial_yaw')}
    if not all(math.isfinite(value) for value in initial.values()):
        raise RuntimeError('AMCL initial pose must contain finite values')
    params = str(Path(get_package_share_directory('qianli_navigation')) / 'config/nav2_params.yaml')
    amcl_initial_pose = {
        'use_sim_time': True,
        'set_initial_pose': True,
        'initial_pose.x': initial['initial_x'],
        'initial_pose.y': initial['initial_y'],
        'initial_pose.z': 0.0,
        'initial_pose.yaw': initial['initial_yaw'],
    }
    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[params, {'yaml_filename': str(map_file.resolve()), 'use_sim_time': True}], output='screen'),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             parameters=[params, amcl_initial_pose], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
             parameters=[{'use_sim_time': True, 'autostart': True, 'bond_timeout': 10.0,
                          'node_names': ['map_server', 'amcl']}], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_file'),
        DeclareLaunchArgument('initial_x', default_value='0.0', description='Initial base pose x in map, metres'),
        DeclareLaunchArgument('initial_y', default_value='0.0', description='Initial base pose y in map, metres'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0', description='Initial base yaw in map, radians'),
        OpaqueFunction(function=setup),
    ])
