"""Launch the SO-101 RViz control bench and safe driver.

Usage (in the ROS 2 VM):
    ros2 launch so101_bringup ik_demo.launch.py
    ros2 launch so101_bringup ik_demo.launch.py demo_circle:=true
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory('so101_bringup'))
    urdf_path = pkg_share / 'urdf' / 'so101.urdf'
    rviz_config = pkg_share / 'config' / 'ik_demo.rviz'
    driver_config = Path(os.environ.get('QI_DRIVER_CONFIG',
                                       str(pkg_share / 'config' / 'driver_params.yaml')))

    urdf_text = urdf_path.read_text(encoding='utf-8')
    urdf_text = urdf_text.replace(
        'filename="assets/',
        f'filename="file://{urdf_path.parent}/assets/')
    urdf_text = urdf_text.replace(
        'filename="package://qianli_description/meshes/',
        f'filename="file://{urdf_path.parent}/assets/')

    demo_circle = LaunchConfiguration('demo_circle')
    driver_mode = LaunchConfiguration('driver_mode')
    port = LaunchConfiguration('port')
    allow_motion = LaunchConfiguration('allow_motion')
    calibrated = LaunchConfiguration('calibrated')
    preserve_torque = LaunchConfiguration('preserve_torque')
    use_rviz = LaunchConfiguration('use_rviz')
    use_ik = LaunchConfiguration('use_ik')
    straight_pose = LaunchConfiguration('straight_pose')
    max_joint_speed = LaunchConfiguration('max_joint_speed')
    orientation_mode = LaunchConfiguration('orientation_mode')

    from launch_ros.parameter_descriptions import ParameterValue

    return LaunchDescription([
        DeclareLaunchArgument(
            'demo_circle', default_value='false',
            description='Run the built-in circular trajectory demo'),
        DeclareLaunchArgument(
            'driver_mode', default_value='sim', choices=['sim', 'direct'],
            description='sim for RViz only; direct for the USB servo controller'),
        DeclareLaunchArgument(
            'port', default_value=os.environ.get('QI_ARM_PORT', '/dev/ttyACM0'),
            description='Serial port used in direct mode'),
        DeclareLaunchArgument(
            'allow_motion', default_value='false', choices=['true', 'false'],
            description='Permit the direct driver to energize motors'),
        DeclareLaunchArgument(
            'calibrated', default_value='false', choices=['true', 'false'],
            description='Confirm zero_raw and direction were measured'),
        DeclareLaunchArgument(
            'preserve_torque', default_value='false',
            choices=['true', 'false'],
            description='Adopt an already-locked calibrated arm without releasing it'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true', choices=['true', 'false'],
            description='Start RViz; set false for headless testing'),
        DeclareLaunchArgument(
            'use_ik', default_value='true', choices=['true', 'false'],
            description='Start IK command publisher; set false for read-only hardware checks'),
        DeclareLaunchArgument(
            'straight_pose', default_value='false', choices=['true', 'false'],
            description='Use the all-zero straight pose as the IK start'),
        DeclareLaunchArgument(
            'max_joint_speed', default_value='0.8',
            description='Hardware joint command speed limit in rad/s'),
        DeclareLaunchArgument(
            'orientation_mode', default_value='Z',
            choices=['none', 'Z', 'all'],
            description='QianLi: IK attitude constraint; all = full '
                        'pose (jaw opening direction controlled)'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': urdf_text,
                'use_sim_time': False,
            }],
        ),

        Node(
            package='so101_bringup',
            executable='driver_node',
            name='so101_driver',
            output='screen',
            parameters=[
                str(driver_config),
                {
                    'mode': driver_mode,
                    'port': port,
                    'allow_motion': ParameterValue(
                        allow_motion, value_type=bool),
                    'calibrated': ParameterValue(
                        calibrated, value_type=bool),
                    'preserve_torque': ParameterValue(
                        preserve_torque, value_type=bool),
                    'max_joint_speed': ParameterValue(
                        max_joint_speed, value_type=float),
                },
            ],
        ),

        Node(
            package='so101_bringup',
            executable='ik_node',
            name='so101_ik_node',
            output='screen',
            condition=IfCondition(use_ik),
            parameters=[{
                'demo_circle': ParameterValue(demo_circle, value_type=bool),
                'command_topic': '/joint_commands_raw',
                'max_joint_speed': ParameterValue(max_joint_speed, value_type=float),
                'straight_pose': ParameterValue(
                    straight_pose, value_type=bool),
                # QianLi: 让 IK 对齐工具 Z 轴 → 夹爪"完全朝下"
                # 默认 'none' 会忽略姿态，导致每次爪子朝向随机
                'orientation_mode': orientation_mode,
            }],
        ),

        Node(
            package='qianli_vision', executable='safety_gate',
            name='safety_gate', output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', str(rviz_config)],
        ),
    ])
