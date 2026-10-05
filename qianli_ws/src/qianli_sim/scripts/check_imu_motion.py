#!/usr/bin/env python3
"""Verify actual Gazebo IMU reacts to controlled CCW motion and stop."""
import argparse
import json
from pathlib import Path
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = rclpy.create_node('qianli_imu_motion_test')
    publisher = node.create_publisher(TwistStamped, '/cmd_vel', 10)
    values = []
    node.create_subscription(Imu, '/imu/data', lambda m: values.append(m.angular_velocity.z), qos_profile_sensor_data)

    def run(wz, duration):
        start = node.get_clock().now().nanoseconds
        deadline = time.monotonic()+40
        while (node.get_clock().now().nanoseconds-start)/1e9 < duration:
            assert time.monotonic() < deadline
            msg = TwistStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'base_footprint'
            msg.twist.angular.z = wz
            publisher.publish(msg)
            until = time.monotonic()+.05
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.005)
    try:
        deadline = time.monotonic()+20
        while (not values or publisher.get_subscription_count() == 0) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        assert values and publisher.get_subscription_count() > 0, 'IMU/controller discovery timed out'
        run(0., .5)
        values.clear()
        run(.3, 1.)
        assert values and max(values) > .2, values[-10:]
        maximum = max(values)
        run(0., .5)
        assert abs(values[-1]) < .03, values[-1]
        print(f'PASS: Gazebo IMU responds to CCW: maximum wz={maximum:.6f}; stopped wz={values[-1]:.6f}')
        if args.output:
            Path(args.output).write_text(json.dumps({'result': 'PASS', 'max_wz': maximum,
                                                    'stopped_wz': values[-1]}, indent=2))
    finally:
        run(0., .3)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
