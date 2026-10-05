#!/usr/bin/env python3
"""W/S forward/back, A/D strafe, Q/E yaw, space stop, X exit. TwistStamped."""
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import TwistStamped


def main():
    rclpy.init()
    node = rclpy.create_node('qianli_teleop')
    publisher = node.create_publisher(TwistStamped, '/cmd_vel', 10)
    settings = termios.tcgetattr(sys.stdin)
    keys = {'w': (.2, 0., 0.), 's': (-.2, 0., 0.), 'a': (0., .2, 0.),
            'd': (0., -.2, 0.), 'q': (0., 0., .4), 'e': (0., 0., -.4)}
    command, last = (0., 0., 0.), 0.

    def publish(cmd):
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = cmd
        publisher.publish(msg)

    print(__doc__, flush=True)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], .02)[0]:
                key = sys.stdin.read(1).lower()
                if key in ('x', '\x03'):
                    break
                command, last = keys.get(key, (0., 0., 0.)), time.monotonic()
            if time.monotonic()-last > .35:
                command = (0., 0., 0.)
            publish(command)
            rclpy.spin_once(node, timeout_sec=0.)
    finally:
        for _ in range(5):
            publish((0., 0., 0.))
            time.sleep(.02)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
