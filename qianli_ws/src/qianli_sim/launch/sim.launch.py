"""Harmonic, model, gz_ros2_control, bridge, ideal motion, optional RViz."""
import os
from pathlib import Path
import runpy
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def setup(context):
    sim = Path(get_package_share_directory('qianli_sim'))
    desc = Path(get_package_share_directory('qianli_description'))
    mode = LaunchConfiguration('sim_mode').perform(context)
    if mode != 'ideal_kinematic_sim':
        raise RuntimeError('Only ideal_kinematic_sim is validated; wheel_physics_sim remains a separate TODO')
    control = runpy.run_path(str(Path(get_package_share_directory('qianli_control')) / 'launch/control.launch.py'))
    config = control['controller_config'](use_sim_time=True, open_loop=True)
    document = xacro.process_file(str(desc / 'urdf/qianli.urdf.xacro'),
                                 mappings={'control_mode': 'gazebo', 'simulation': 'true', 'controllers_file': config})
    description = document.toxml()
    converted = ET.fromstring(description)
    for mesh in converted.findall('.//mesh'):
        uri = mesh.get('filename')
        if uri.startswith('package://'):
            package, relative = uri[len('package://'):].split('/', 1)
            mesh.set('filename', str(Path(get_package_share_directory(package)) / relative))
    with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as f:
        f.write(ET.tostring(converted))
        urdf_path = f.name
    sdf = ET.fromstring(subprocess.check_output(['gz', 'sdf', '-p', urdf_path], text=True))
    model = sdf.find('model')
    if model is None or len(model.findall('joint')) < 4:
        raise RuntimeError('URDF→SDF conversion lost the wheel joints')
    # Honest ideal abstraction: no robot contacts, no gravitational drift.
    # World obstacles retain collisions and are seen by the actual Gazebo laser.
    for link in model.findall('link'):
        for collision in link.findall('collision'):
            link.remove(collision)
        ET.SubElement(link, 'gravity').text = 'false'
    plugin = ET.SubElement(model, 'plugin', filename='gz-sim-velocity-control-system',
                           name='gz::sim::systems::VelocityControl')
    ET.SubElement(plugin, 'topic').text = '/qianli/ideal_body_velocity'
    plugin = ET.SubElement(model, 'plugin', filename='gz-sim-odometry-publisher-system',
                           name='gz::sim::systems::OdometryPublisher')
    for key, value in {'odom_topic': '/qianli/ground_truth', 'odom_frame': 'world',
                       'robot_base_frame': 'base_footprint', 'odom_publish_frequency': '50',
                       'dimensions': '2', 'tf_topic': '/qianli/ground_truth_tf_unused'}.items():
        ET.SubElement(plugin, key).text = value
    with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', prefix='qianli_ideal_', delete=False) as f:
        f.write(ET.tostring(sdf, encoding='unicode'))
        model_path = f.name
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    world = LaunchConfiguration('world').perform(context)
    gz_args = ['-r ', world] if gui else ['-r -s --headless-rendering ', world]
    return [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(desc.parent)),
        AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', '/opt/ros/jazzy/lib'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py')),
                                 launch_arguments={'gz_args': gz_args}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': True}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='qianli_bridge',
             parameters=[{'config_file': str(sim / 'config/bridge.yaml'), 'use_sim_time': True}], output='screen'),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-name', 'qianli', '-file', model_path, '-x', '0', '-y', '0', '-z', '0'],
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='qianli_sim', executable='ideal_kinematic_sim.py', parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(desc / 'rviz/qianli.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ] + control['spawners'](config)


def generate_launch_description():
    sim = Path(get_package_share_directory('qianli_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('sim_mode', default_value='ideal_kinematic_sim'),
        DeclareLaunchArgument('world', default_value=str(sim / 'worlds/qianli_test_world.sdf')),
        OpaqueFunction(function=setup)])
