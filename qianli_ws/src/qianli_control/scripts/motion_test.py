#!/usr/bin/env python3
"""Timed motion smoke test; checks commanded odometry and Gazebo pose deltas."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

CASES = [('forward', (.15, 0., 0.)), ('strafe_left', (0., .15, 0.)),
         ('rotate_ccw', (0., 0., .3)), ('diagonal', (.10, .10, 0.))]


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=2.)
    parser.add_argument('--gazebo', action='store_true')
    parser.add_argument('--output', type=Path)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = rclpy.create_node('qianli_motion_test')
    data = {}
    node.create_subscription(Odometry, '/odom', lambda m: data.update(odom=m), 10)
    node.create_subscription(Odometry, '/simulation/ground_truth', lambda m: data.update(truth=m), 10)
    publisher = node.create_publisher(TwistStamped, '/cmd_vel', 10)

    def send(cmd):
        m = TwistStamped()
        m.header.stamp = node.get_clock().now().to_msg()
        m.header.frame_id = 'base_footprint'
        m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = cmd
        publisher.publish(m)

    def run(cmd, seconds):
        # Duration follows the ROS clock, while wall time bounds failures.
        start = node.get_clock().now().nanoseconds
        deadline = time.monotonic()+max(30., seconds*10.)
        while (node.get_clock().now().nanoseconds-start)/1e9 < seconds:
            assert time.monotonic() < deadline, 'clock stopped'
            send(cmd)
            # Bound publish rate; a busy subscription must not flood reliable
            # command queues and leave the stop command behind stale commands.
            until = time.monotonic()+.05
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.005)

    def pose(topic):
        p = data[topic].pose.pose
        return p.position.x, p.position.y, yaw(p.orientation)

    report = {}
    try:
        deadline = time.monotonic()+60
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if 'odom' in data and (not args.gazebo or 'truth' in data) and publisher.get_subscription_count():
                break
        assert 'odom' in data and (not args.gazebo or 'truth' in data)
        run((0., 0., 0.), .3)
        for name, cmd in CASES:
            topics = ['odom', 'truth'] if args.gazebo else ['odom']
            before = {t: pose(t) for t in topics}
            run(cmd, args.duration)
            run((0., 0., 0.), .4)
            result = {}
            for t in topics:
                x, y, heading = pose(t)
                x0, y0, h0 = before[t]
                dx, dy = x-x0, y-y0
                body = (math.cos(h0)*dx+math.sin(h0)*dy, -math.sin(h0)*dx+math.cos(h0)*dy,
                        math.atan2(math.sin(heading-h0), math.cos(heading-h0)))
                target = tuple(v*args.duration for v in cmd)
                assert all(abs(a-e) < .10 for a, e in zip(body, target)), (name, t, body, target)
                result[t+'_body_delta'] = body
            report[name] = result
        report['result'] = 'PASS'
    finally:
        run((0., 0., 0.), .5)
        node.destroy_node()
        rclpy.shutdown()
    text = json.dumps(report, indent=2)
    print(text)
    if args.output:
        args.output.write_text(text)


if __name__ == '__main__':
    main()
