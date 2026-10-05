#!/usr/bin/env python3
"""Check live simulated controllers, sensors, clocks, topic graph and TF ownership."""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Imu, JointState, LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.srv import GetParameters
from lifecycle_msgs.srv import GetState


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', action='store_true')
    parser.add_argument('--navigation', action='store_true')
    parser.add_argument('--localization', choices=['slam', 'amcl'], default='slam')
    parser.add_argument('--output', type=Path)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = rclpy.create_node('qianli_system_acceptance')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    data, counts, owners = {}, Counter(), defaultdict(set)

    def receive(name):
        def callback(msg):
            data[name] = msg
            counts[name] += 1
        return callback

    for name, cls, topic, qos in [
        ('clock', Clock, '/clock', 10), ('odom', Odometry, '/odom', 10),
        ('truth', Odometry, '/simulation/ground_truth', 10),
        ('joints', JointState, '/joint_states', 10), ('scan', LaserScan, '/scan', qos_profile_sensor_data),
        ('imu', Imu, '/imu/data', qos_profile_sensor_data)]:
        node.create_subscription(cls, topic, receive(name), qos)
    if args.map:
        node.create_subscription(OccupancyGrid, '/map', receive('map'),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def tf_callback(msg):
        for t in msg.transforms:
            owners[(t.header.frame_id, t.child_frame_id)].add('observed')

    node.create_subscription(TFMessage, '/tf', tf_callback, 100)
    required = {'clock', 'odom', 'truth', 'joints', 'scan', 'imu'} | ({'map'} if args.map else set())
    deadline = time.monotonic()+60
    start = None
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
        if required <= data.keys():
            start = start or time.monotonic()
            if time.monotonic()-start > 3. and counts['clock'] > 10 and counts['scan'] > 5 and counts['imu'] > 5:
                break
    assert required <= data.keys(), f'Missing: {required-data.keys()}'
    assert counts['clock'] > 10 and counts['scan'] > 5 and counts['imu'] > 5, dict(counts)
    scan, imu, odom = data['scan'], data['imu'], data['odom']
    assert scan.header.frame_id == 'lidar_link', scan.header.frame_id
    assert imu.header.frame_id == 'imu_link', imu.header.frame_id
    assert len(scan.ranges) == 360 and sum(math.isfinite(v) for v in scan.ranges) > 300
    assert abs(scan.angle_max-scan.angle_min-2*math.pi) < .01
    assert abs(scan.range_min-.12) < .001 and abs(scan.range_max-12.) < .01
    values = [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w,
              imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z,
              imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z]
    assert all(math.isfinite(v) for v in values)
    assert abs(sum(v*v for v in values[:4])-1.) < .01
    assert odom.header.frame_id == 'odom' and odom.child_frame_id == 'base_footprint'
    assert len(data['joints'].name) == 4 and len(data['joints'].velocity) == 4
    assert node.count_publishers('/odom') == 1, 'duplicate odom publisher'
    assert node.count_subscribers('/cmd_vel') >= 1
    topic_types = dict(node.get_topic_names_and_types())
    assert topic_types['/cmd_vel'] == ['geometry_msgs/msg/TwistStamped'], topic_types['/cmd_vel']
    pairs = [('odom', 'base_footprint')] + ([('map', 'odom')] if args.map else [])
    tf_publisher_counts = Counter(p.node_name for p in node.get_publishers_info_by_topic('/tf'))
    tf_publishers = set(tf_publisher_counts)
    localization_node = 'slam_toolbox' if args.localization == 'slam' else 'amcl'
    expected_tf_publishers = {'robot_state_publisher', 'omni_base_controller'} | ({localization_node} if args.map else set())
    assert tf_publishers == expected_tf_publishers, ('unexpected TF publisher', tf_publishers)
    assert all(count == 1 for count in tf_publisher_counts.values()), ('duplicate TF publisher', tf_publisher_counts)
    for pair in pairs:
        assert owners[pair], ('missing TF edge', pair)
    frames = ['base_link', 'imu_link', 'lidar_link']+[n+'_wheel_link' for n in ('front_left', 'front_right', 'rear_left', 'rear_right')]
    for frame in frames:
        assert buffer.can_transform('base_footprint', frame, rclpy.time.Time()), frame
    assert buffer.can_transform('odom', 'base_footprint', rclpy.time.Time())
    if args.map:
        assert buffer.can_transform('map', 'base_footprint', rclpy.time.Time())
        cells = data['map'].data
        assert sum(v >= 0 for v in cells) > 100 and sum(v > 50 for v in cells) > 10

    def call(client, request):
        assert client.wait_for_service(timeout_sec=10), client.srv_name
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        assert future.done() and future.result() is not None, client.srv_name
        return future.result()

    response = call(node.create_client(ListControllers, '/controller_manager/list_controllers'), ListControllers.Request())
    controllers = {c.name: c.state for c in response.controller}
    assert controllers['omni_base_controller'] == controllers['joint_state_broadcaster'] == 'active'
    names = ['controller_manager', 'omni_base_controller', 'joint_state_broadcaster',
             'robot_state_publisher', 'qianli_bridge', 'ideal_kinematic_sim'] + ([localization_node] if args.map else [])
    navigation_nodes = ['controller_server', 'planner_server', 'smoother_server', 'behavior_server', 'bt_navigator']
    lifecycle = {}
    if args.map and args.localization == 'amcl':
        names += ['map_server', 'lifecycle_manager_localization']
        for name in ['map_server', 'amcl']:
            state = call(node.create_client(GetState, f'/{name}/get_state'), GetState.Request()).current_state
            assert state.id == 3, (name, state.label)
            lifecycle[name] = state.label
    if args.navigation:
        assert args.map, '--navigation requires --map'
        names += navigation_nodes + ['lifecycle_manager_navigation', 'local_costmap/local_costmap', 'global_costmap/global_costmap']
        for name in navigation_nodes:
            state = call(node.create_client(GetState, f'/{name}/get_state'), GetState.Request()).current_state
            assert state.id == 3, (name, state.label)
            lifecycle[name] = state.label
        expected_footprint = [[.35,.20],[.20,.35],[-.20,.35],[-.35,.20],[-.35,-.20],[-.20,-.35],[.20,-.35],[.35,-.20]]
        for name in ['local_costmap/local_costmap', 'global_costmap/global_costmap']:
            request = GetParameters.Request(names=['footprint', 'footprint_padding'])
            footprint_values = call(node.create_client(GetParameters, f'/{name}/get_parameters'), request).values
            assert json.loads(footprint_values[0].string_value) == expected_footprint
            assert abs(footprint_values[1].double_value-.06) < 1e-6
    sim_time = {}
    for name in names:
        request = GetParameters.Request(names=['use_sim_time'])
        result = call(node.create_client(GetParameters, f'/{name}/get_parameters'), request)
        sim_time[name] = result.values[0].bool_value
        assert sim_time[name], f'{name} does not use simulation clock'
    report = {'result': 'PASS', 'controllers': controllers, 'message_counts': dict(counts),
              'scan': {'frame': scan.header.frame_id, 'samples': len(scan.ranges), 'min': min(scan.ranges), 'max': max(scan.ranges)},
              'imu': {'frame': imu.header.frame_id, 'values': values},
              'tf_publishers': sorted(tf_publishers), 'tf_publisher_counts': dict(tf_publisher_counts),
              'tf_edges_seen': [list(p) for p in pairs], 'use_sim_time': sim_time}
    if args.navigation:
        report['nav2_lifecycle'] = lifecycle
        report['nav2_footprint'] = expected_footprint
        report['footprint_padding'] = .06
    if args.map:
        report['map'] = {'width': data['map'].info.width, 'height': data['map'].info.height,
                         'known_cells': sum(v >= 0 for v in data['map'].data),
                         'occupied_cells': sum(v > 50 for v in data['map'].data)}
    text = json.dumps(report, indent=2)
    print(text)
    if args.output:
        args.output.write_text(text)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
