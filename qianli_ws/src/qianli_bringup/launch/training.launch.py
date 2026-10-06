"""Gazebo teaching-building scenarios with saved-map AMCL or online SLAM.

Scenario map and world share their metric XY frame. AMCL starts at the Gazebo
spawn pose, while the controller starts odom at its own origin. Only AMCL or
slam_toolbox owns map -> odom. All downstream nodes use simulation time.
"""
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def include(package, launch, arguments=None, condition=None):
    kwargs = {'launch_arguments': (arguments or {}).items()}
    if condition is not None:
        kwargs['condition'] = condition
    return GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory(package)) / 'launch' / launch)),
        **kwargs,
    )])


def setup(context):
    variant = LaunchConfiguration('variant').perform(context)
    if not variant or any(character not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for character in variant):
        raise RuntimeError('variant must be a scenario name containing letters, digits, underscores or hyphens')
    scenario = Path(get_package_share_directory('qianli_training_scenarios')) / 'generated' / variant
    paths = {}
    for argument, filename in (('world', 'teaching.sdf'), ('map', 'teaching.yaml')):
        override = LaunchConfiguration(argument).perform(context).strip()
        paths[argument] = Path(override).expanduser() if override else scenario / filename
    if not paths['world'].is_file():
        raise RuntimeError(f'Training world does not exist: {paths["world"]}')
    slam = LaunchConfiguration('slam')
    nav2 = LaunchConfiguration('nav2')
    saved_map_mode = (LaunchConfiguration('nav2').perform(context).lower() == 'true'
                      and LaunchConfiguration('slam').perform(context).lower() != 'true')
    if saved_map_mode and not paths['map'].is_file():
        raise RuntimeError(f'Training map does not exist: {paths["map"]}')
    spawn = {name: LaunchConfiguration(name).perform(context)
             for name in ('spawn_x', 'spawn_y', 'spawn_z', 'spawn_yaw')}
    if not all(math.isfinite(float(value)) for value in spawn.values()):
        raise RuntimeError('Gazebo spawn pose must contain finite values')
    scene_share = Path(get_package_share_directory('qianli_training_scenarios'))
    localization = IfCondition(PythonExpression([
        "'", nav2, "'.lower() == 'true' and '", slam, "'.lower() != 'true'"
    ]))
    fixed_frame = PythonExpression([
        "'map' if '", slam, "'.lower() == 'true' or '", nav2, "'.lower() == 'true' else 'odom'"
    ])
    return [
        include('qianli_sim', 'sim.launch.py', {
            'gui': LaunchConfiguration('gui'),
            'rviz': 'false',
            'world': str(paths['world'].resolve()),
            'headless_rendering': LaunchConfiguration('headless_rendering'),
            'sim_mode': 'ideal_kinematic_sim',
            **spawn,
        }),
        include('qianli_slam', 'slam.launch.py', condition=IfCondition(slam)),
        include('qianli_navigation', 'localization.launch.py', {
            'map_file': str(paths['map'].resolve()),
            'initial_x': spawn['spawn_x'],
            'initial_y': spawn['spawn_y'],
            'initial_yaw': spawn['spawn_yaw'],
        }, condition=localization),
        include('qianli_navigation', 'navigation.launch.py', condition=IfCondition(nav2)),
        Node(package='rviz2', executable='rviz2', name='qianli_training_rviz',
             arguments=['-d', str(scene_share / 'rviz/training.rviz'), '-f', fixed_frame],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('variant', default_value='baseline'),
        DeclareLaunchArgument('world', default_value='', description='Override generated variant SDF'),
        DeclareLaunchArgument('map', default_value='', description='Override generated variant map YAML'),
        DeclareLaunchArgument('spawn_x', default_value='13.5'),
        DeclareLaunchArgument('spawn_y', default_value='-16.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='Gazebo and AMCL yaw in radians'),
        DeclareLaunchArgument('slam', default_value='false', description='Online SLAM replaces AMCL and saved map'),
        DeclareLaunchArgument('nav2', default_value='true'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('headless_rendering', default_value='false',
                              description='Keep false with this VM software GL; use an existing Xvfb display'),
        OpaqueFunction(function=setup),
    ])
