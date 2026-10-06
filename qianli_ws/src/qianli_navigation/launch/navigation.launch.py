"""Minimal holonomic Nav2 servers; slam_toolbox or AMCL owns map→odom."""
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    share = Path(get_package_share_directory('qianli_navigation'))
    params = yaml.safe_load((share / 'config/nav2_params.yaml').read_text())
    footprints = yaml.safe_load((Path(get_package_share_directory('qianli_description')) / 'config/nav2_footprint.yaml').read_text())
    for costmap in ('local_costmap', 'global_costmap'):
        params[costmap][costmap]['ros__parameters'].update(footprints[costmap][costmap]['ros__parameters'])
    with tempfile.NamedTemporaryFile(mode='w', prefix='qianli_nav2_', suffix='.yaml', delete=False) as f:
        yaml.safe_dump(params, f)
        config = f.name
    servers = [('nav2_controller', 'controller_server'), ('nav2_planner', 'planner_server'),
               ('nav2_smoother', 'smoother_server'), ('nav2_behaviors', 'behavior_server'),
               ('nav2_bt_navigator', 'bt_navigator')]
    nodes = [Node(package=pkg, executable=name, name=name, parameters=[config], output='screen')
             for pkg, name in servers]
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
                      parameters=[{'use_sim_time': True, 'autostart': True, 'bond_timeout': 10.0,
                                   'node_names': [name for _, name in servers]}], output='screen'))
    return LaunchDescription(nodes)
