"""QianLi Base Geometry v0.1: description, joint states and RViz only."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    share = get_package_share_directory('qianli_description')
    description = xacro.process_file(
        os.path.join(share, 'urdf', 'qianli.urdf.xacro')).toxml()
    gui = LaunchConfiguration('gui')
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='false',
                              description='Use joint_state_publisher_gui sliders'),
        DeclareLaunchArgument('rviz_config',
                              default_value=os.path.join(share, 'rviz', 'qianli.rviz')),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='qianli_robot_state_publisher', output='screen',
             parameters=[{'robot_description': ParameterValue(description, value_type=str)}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='qianli_joint_state_publisher', condition=UnlessCondition(gui),
             parameters=[{'rate': 30}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             name='qianli_joint_state_publisher_gui', condition=IfCondition(gui)),
        Node(package='rviz2', executable='rviz2', name='qianli_rviz',
             arguments=['-d', LaunchConfiguration('rviz_config')],
             condition=IfCondition(LaunchConfiguration('use_rviz')), output='screen'),
    ])
