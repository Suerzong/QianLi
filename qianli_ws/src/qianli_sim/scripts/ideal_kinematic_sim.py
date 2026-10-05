#!/usr/bin/env python3
"""Drive Gazebo's ideal body velocity from the official controller's odometry.

This adapter has no wheel kinematics and publishes no odometry/TF.
Robot collisions are disabled in this explicitly ideal simulation mode.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class IdealKinematicSim(Node):
    def __init__(self):
        super().__init__('ideal_kinematic_sim')
        self.last = None
        self.command = Twist()
        self.publisher = self.create_publisher(Twist, '/simulation/body_velocity', 10)
        self.create_subscription(Odometry, '/odom', self.odometry, 10)
        self.create_timer(.02, self.update)
        self.get_logger().info('ideal_kinematic_sim: imposed body motion, no wheel-ground physics validation')

    def odometry(self, msg):
        t = msg.twist.twist
        if msg.child_frame_id != 'base_footprint' or not all(math.isfinite(v) for v in (t.linear.x, t.linear.y, t.angular.z)):
            return
        self.command = Twist()
        self.command.linear.x, self.command.linear.y, self.command.angular.z = t.linear.x, t.linear.y, t.angular.z
        self.last = self.get_clock().now()

    def update(self):
        stale = self.last is None or (self.get_clock().now()-self.last).nanoseconds > 500_000_000
        self.publisher.publish(Twist() if stale else self.command)


def main():
    rclpy.init()
    node = IdealKinematicSim()
    try:
        rclpy.spin(node)
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
