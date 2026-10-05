"""Mock mode for controller verification; Gazebo owns its manager in simulation."""
import math
import os
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro
import yaml


def controller_config(use_sim_time=False, open_loop=True):
    description = get_package_share_directory('qianli_description')
    control = get_package_share_directory('qianli_control')
    props = ET.parse(os.path.join(description, 'urdf', 'common.xacro')).getroot()
    p = {e.get('name'): float(e.get('value')) for e in props
         if e.tag.endswith('property') and not e.get('value').startswith('${')}
    assert math.isclose(p['wheel_x'], p['wheel_y']), 'X-drive requires equal radial angles'
    params = yaml.safe_load(open(os.path.join(control, 'config', 'controllers.yaml')))
    params['controller_manager']['ros__parameters']['use_sim_time'] = use_sim_time
    cfg = params['omni_base_controller']['ros__parameters']
    cfg.update(robot_radius=math.hypot(p['wheel_x'], p['wheel_y']),
               wheel_radius=p['wheel_radius'], wheel_offset=math.atan2(p['wheel_y'], p['wheel_x']),
               use_sim_time=use_sim_time, open_loop=open_loop)
    params['joint_state_broadcaster'] = {'ros__parameters': {'use_sim_time': use_sim_time}}
    with tempfile.NamedTemporaryFile(mode='w', prefix='qianli_controllers_', suffix='.yaml', delete=False) as f:
        yaml.safe_dump(params, f)
        return f.name


def spawners(config):
    broadcaster = Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', '-p', config, '--controller-manager-timeout', '90',
                        '--service-call-timeout', '30', '--switch-timeout', '30'])
    omni = Node(package='controller_manager', executable='spawner',
             arguments=['omni_base_controller', '-p', config, '--controller-manager-timeout', '90',
                        '--service-call-timeout', '30', '--switch-timeout', '30',
                        '--controller-ros-args=-r', '--controller-ros-args=~/cmd_vel:=/cmd_vel',
                        '--controller-ros-args=-r', '--controller-ros-args=~/odom:=/odom'])
    return [RegisterEventHandler(OnProcessExit(target_action=broadcaster, on_exit=[omni])), broadcaster]


def setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    open_loop = LaunchConfiguration('open_loop').perform(context).lower() == 'true'
    config = controller_config(use_sim_time, open_loop)
    path = os.path.join(get_package_share_directory('qianli_description'), 'urdf', 'qianli.urdf.xacro')
    description = xacro.process_file(path, mappings={'control_mode': 'mock'}).toxml()
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': use_sim_time}]),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[config], output='screen'),
    ] + spawners(config)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('open_loop', default_value='true'), OpaqueFunction(function=setup)])
