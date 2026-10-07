"""Minimal holonomic Nav2 servers; slam_toolbox or AMCL owns map→odom."""
from pathlib import Path
import tempfile
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def exploration_parameters(params, radius):
    """Make controller geometry and map knowledge match frontier eligibility."""
    count = 32
    for name in ('local_costmap', 'global_costmap'):
        cfg = params[name][name]['ros__parameters']
        resolution = cfg['resolution']
        # The global planner reasons in grid centres. Reserve its cell
        # half-diagonal and a small interpolation margin. Local collision
        # checks already rasterize the physical footprint against cells.
        clearance = radius+(resolution/math.sqrt(2)+resolution*.1 if name=='global_costmap' else 0.)
        vertex_radius = (clearance+1e-6)/math.cos(math.pi/count)
        polygon = [[vertex_radius*math.cos(i*math.tau/count),
                    vertex_radius*math.sin(i*math.tau/count)] for i in range(count)]
        cfg.update(footprint=str(polygon), footprint_padding=0.0, track_unknown_space=True,
                   plugins=['static_layer', 'obstacle_layer', 'inflation_layer'])
        cfg['static_layer'] = {'plugin': 'nav2_costmap_2d::StaticLayer',
                               'map_subscribe_transient_local': True}
        # Live scans may add obstacles, but may not turn SLAM-unknown space free.
        cfg['obstacle_layer']['combination_method'] = 2
        cfg['inflation_layer']['inflate_around_unknown'] = True
    params['planner_server']['ros__parameters']['GridBased'] = {
        'plugin': 'nav2_smac_planner::SmacPlanner2D', 'tolerance': .05,
        'allow_unknown': False, 'downsample_costmap': False,
        'max_iterations': 1000000, 'max_on_approach_iterations': 1000,
        'max_planning_time': 3., 'cost_travel_multiplier': 2.,
        'use_final_approach_orientation': False,
        'smoother': {'max_iterations': 0, 'do_refinement': False},
    }
    params['controller_server']['ros__parameters']['FollowPath'] = {
        'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
        'desired_linear_vel': .25, 'lookahead_dist': .35,
        'min_lookahead_dist': .2, 'max_lookahead_dist': .5, 'lookahead_time': 1.,
        'use_velocity_scaled_lookahead_dist': True, 'transform_tolerance': .5,
        'use_collision_detection': True, 'max_allowed_time_to_collision_up_to_carrot': 1.,
        'use_regulated_linear_velocity_scaling': True, 'regulated_linear_scaling_min_radius': .8,
        'regulated_linear_scaling_min_speed': .08, 'use_cost_regulated_linear_velocity_scaling': True,
        'cost_scaling_dist': .55, 'inflation_cost_scaling_factor': 3.,
        'min_approach_linear_velocity': .04, 'approach_velocity_scaling_dist': .5,
        'use_rotate_to_heading': True, 'rotate_to_heading_min_angle': .4,
        'rotate_to_heading_angular_vel': .5, 'max_angular_accel': 1.2,
        'allow_reversing': False,
    }
    return params


def make_servers(context):
    share = Path(get_package_share_directory('qianli_navigation'))
    params = yaml.safe_load((share / 'config/nav2_params.yaml').read_text())
    footprints = yaml.safe_load((Path(get_package_share_directory('qianli_description')) / 'config/nav2_footprint.yaml').read_text())
    for costmap in ('local_costmap', 'global_costmap'):
        params[costmap][costmap]['ros__parameters'].update(footprints[costmap][costmap]['ros__parameters'])
    if LaunchConfiguration('exploration_mode').perform(context).lower() == 'true':
        config = Path(get_package_share_directory('qianli_exploration'))/'config/exploration.yaml'
        radius = yaml.safe_load(config.read_text())['qianli_frontier_explorer']['ros__parameters']['robot_radius']
        params = exploration_parameters(params, radius)
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
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('exploration_mode', default_value='false'),
        OpaqueFunction(function=make_servers),
    ])
