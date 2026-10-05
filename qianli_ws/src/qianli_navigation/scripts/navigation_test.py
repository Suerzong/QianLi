#!/usr/bin/env python3
"""NavigateToPose acceptance: goals, holonomic commands and no Gazebo obstacle crossing."""
import argparse
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path as NavPath
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def intersects(a, b):
    for polygon in (a, b):
        for i, (x, y) in enumerate(polygon):
            xx, yy = polygon[(i+1) % len(polygon)]
            nx, ny = y-yy, xx-x
            pa, pb = [nx*x+ny*y for x, y in a], [nx*x+ny*y for x, y in b]
            if max(pa) <= min(pb) or max(pb) <= min(pa):
                return False
    return True


def world_obstacles():
    root = ET.parse(Path(get_package_share_directory('qianli_sim')) / 'worlds/qianli_test_world.sdf').getroot()
    result = {}
    for model in root.findall('world/model'):
        if model.get('name') == 'ground':
            continue
        pose = list(map(float, model.findtext('pose').split()))
        assert pose[-1] == 0, 'test world obstacle rotation unsupported'
        sx, sy, _ = map(float, model.findtext('link/collision/geometry/box/size').split())
        x, y = pose[:2]
        result[model.get('name')] = [(x-sx/2, y-sy/2), (x+sx/2, y-sy/2),
                                     (x+sx/2, y+sy/2), (x-sx/2, y+sy/2)]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--goals', default='[[0,1,0],[2.5,-1.8,1.5707963267948966],[0,0,0]]')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = rclpy.create_node('qianli_navigation_acceptance')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    publisher = node.create_publisher(TwistStamped, '/cmd_vel', 10)
    data = {'commands': [], 'plans': 0, 'truth': None, 'trajectory': [], 'collisions': []}
    obstacles = world_obstacles()
    # Requested octagonal outline + current navigation's 60 mm padding.
    vertices = [(0.41,.26),(.26,.41),(-.26,.41),(-.41,.26),
                (-.41,-.26),(-.26,-.41),(.26,-.41),(.41,-.26)]

    def truth(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        h = yaw(q)
        data['truth'] = (p.x, p.y, h)
        polygon = [(p.x+math.cos(h)*x-math.sin(h)*y, p.y+math.sin(h)*x+math.cos(h)*y) for x, y in vertices]
        for name, box in obstacles.items():
            if intersects(polygon, box):
                data['collisions'].append({'obstacle': name, 'pose': [p.x, p.y, h]})
        data['trajectory'].append([p.x, p.y, h])

    def command(msg):
        data['commands'].append([msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z])

    def plan(_):
        data['plans'] += 1

    node.create_subscription(Odometry, '/simulation/ground_truth', truth, 10)
    node.create_subscription(TwistStamped, '/cmd_vel', command, 10)
    node.create_subscription(NavPath, '/plan', plan, 10)

    def wait(future, timeout):
        deadline = time.monotonic()+timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        assert future.done(), 'action timeout'
        return future.result()

    def stop():
        until = time.monotonic()+.5
        while time.monotonic() < until:
            msg = TwistStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'base_footprint'
            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=.05)

    goals, handle = [], None
    try:
        assert client.wait_for_server(timeout_sec=60), 'NavigateToPose unavailable'
        deadline = time.monotonic()+30
        while data['truth'] is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        assert data['truth'] is not None
        for x, y, heading in json.loads(args.goals):
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
            goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
            print('Sending goal', x, y, heading, flush=True)
            handle = wait(client.send_goal_async(goal), 20)
            assert handle.accepted, 'goal rejected'
            result = wait(handle.get_result_async(), 180)
            assert result.status == 4, ('navigation did not succeed', result.status, result.result)
            transform = buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
            actual = [transform.translation.x, transform.translation.y, yaw(transform.rotation)]
            error = [actual[0]-x, actual[1]-y, math.atan2(math.sin(actual[2]-heading), math.cos(actual[2]-heading))]
            assert math.hypot(*error[:2]) < .22 and abs(error[2]) < .22, error
            assert math.hypot(data['truth'][0]-x, data['truth'][1]-y) < .3, 'map pose and Gazebo pose disagree'
            assert not data['collisions'], data['collisions'][:3]
            goals.append({'goal': [x, y, heading], 'status': result.status, 'map_pose': actual,
                          'gazebo_pose': data['truth'], 'map_error': error})
            print('Goal succeeded', goals[-1], flush=True)
            handle = None
        maximum = [max(abs(c[i]) for c in data['commands']) for i in range(3)]
        assert all(m > .03 for m in maximum), ('did not observe vx/vy/wz', maximum)
        assert data['plans'] > 0
        report = {'result': 'PASS', 'goals': goals, 'maximum_abs_cmd_vel_xyz': maximum,
                  'plans_received': data['plans'], 'gazebo_trajectory_samples': len(data['trajectory']),
                  'obstacle_crossings': len(data['collisions']), 'trajectory': data['trajectory'][::10]}
        if args.output:
            args.output.write_text(json.dumps(report, indent=2))
        print(json.dumps({k: v for k, v in report.items() if k != 'trajectory'}, indent=2))
    finally:
        if handle is not None:
            wait(handle.cancel_goal_async(), 5)
        stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
