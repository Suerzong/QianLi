"""QianLi 机械臂（SO-ARM101）display launch。

启动 robot_state_publisher + joint_state_publisher_gui + RViz2，
用于在 RViz 中显示机械臂模型与 TF（Milestone 1）。

用法：
    ros2 launch qianli_description display.launch.py
    ros2 launch qianli_description display.launch.py use_rviz:=false   # 无头验证
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('qianli_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'so101.urdf')

    # so101.urdf 为纯 URDF（onshape-to-robot 生成），直接读取文本
    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description_content = f.read()

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    rviz_config = os.path.join(pkg_share, 'rviz', 'arm.rviz')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='是否启动 RViz2（无头验证时设为 false）'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
