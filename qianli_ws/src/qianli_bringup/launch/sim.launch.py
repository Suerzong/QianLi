"""Modular QianLi simulation, sensors, optional SLAM/Nav2/RViz."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def include(package, launch, arguments=None, condition=None):
    kwargs = {'launch_arguments': (arguments or {}).items()}
    if condition is not None:
        kwargs['condition'] = condition
    # Child launch arguments must not overwrite sibling/top-level selectors.
    return GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(Path(get_package_share_directory(package)) / 'launch' / launch)), **kwargs)])


def generate_launch_description():
    slam, nav2, rviz, gui = [LaunchConfiguration(p) for p in ('slam', 'nav2', 'rviz', 'gui')]
    sim = Path(get_package_share_directory('qianli_sim'))
    navigation = Path(get_package_share_directory('qianli_navigation'))
    localization = IfCondition(PythonExpression(["'", nav2, "' == 'true' and '", slam, "' != 'true'"]))
    fixed_frame = PythonExpression(["'map' if '", slam, "' == 'true' or '", nav2, "' == 'true' else 'odom'"])
    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('nav2', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('sim_mode', default_value='ideal_kinematic_sim'),
        DeclareLaunchArgument('map', default_value=str(Path(get_package_share_directory('qianli_slam')) / 'maps/qianli_test_map.yaml')),
        include('qianli_sim', 'sim.launch.py', {'gui': gui, 'rviz': 'false', 'sim_mode': LaunchConfiguration('sim_mode')}),
        include('qianli_slam', 'slam.launch.py', condition=IfCondition(slam)),
        include('nav2_bringup', 'localization_launch.py', {'use_sim_time': 'true', 'autostart': 'true',
                'use_composition': 'false', 'map': LaunchConfiguration('map'),
                'params_file': str(navigation / 'config/nav2_params.yaml')}, condition=localization),
        include('qianli_navigation', 'navigation.launch.py', condition=IfCondition(nav2)),
        Node(package='rviz2', executable='rviz2', name='qianli_sim_rviz',
             arguments=['-d', str(sim / 'rviz/simulation.rviz'), '-f', fixed_frame],
             parameters=[{'use_sim_time': True}], condition=IfCondition(rviz)),
    ])
