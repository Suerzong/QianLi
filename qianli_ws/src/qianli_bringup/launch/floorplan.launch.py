"""Open the provisional floor plan through the common training bringup."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim = Path(get_package_share_directory('qianli_sim'))
    slam = Path(get_package_share_directory('qianli_slam'))
    bringup = Path(get_package_share_directory('qianli_bringup'))
    defaults = {
        'world': str(sim / 'worlds/qianli_bupt_shahe_public_teaching_floor1_clean_abstract_v0_2.sdf'),
        'map': str(slam / 'maps/qianli_bupt_shahe_public_teaching_floor1_clean_abstract_v0_2.yaml'),
        'spawn_x': '7.80', 'spawn_y': '8.30', 'spawn_z': '0.0', 'spawn_yaw': '0.0',
        'slam': 'false', 'nav2': 'true', 'rviz': 'false', 'gui': 'false',
        'headless_rendering': 'false', 'explore': 'false', 'exploration_report': '',
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch/training.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in defaults}.items(),
        ),
    ])
