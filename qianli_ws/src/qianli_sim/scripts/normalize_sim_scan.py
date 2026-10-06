#!/usr/bin/env python3
"""Expose Gazebo no-hit rays to SLAM, with the unmodified scan kept separately.

Only the known Gazebo +inf/no-hit convention is converted. Do not attach this
adapter to a physical scanner without checking its invalid/no-hit semantics.
The mapper's max_laser_range must be below the converted clear range; otherwise
Karto treats the synthetic endpoint as an obstacle (11.99 < 11.999 here).
"""
from copy import deepcopy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from scan_ranges import normalize_no_returns


class ScanAdapter(Node):
    def __init__(self):
        super().__init__('qianli_sim_scan_adapter')
        self.publisher = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/simulation/scan_raw', self.on_scan, qos_profile_sensor_data)
        self.logged = False

    def on_scan(self, message):
        output = deepcopy(message)
        output.ranges, converted = normalize_no_returns(message.ranges, message.range_min, message.range_max)
        self.publisher.publish(output)
        if converted and not self.logged:
            self.logged = True
            self.get_logger().info('Gazebo no-hit rays exposed as clear ranges; raw scan: /simulation/scan_raw')


def main():
    rclpy.init()
    node = ScanAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
