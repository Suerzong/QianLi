#!/usr/bin/env python3
"""Independent contact-geometry test; --live validates official controller on mock hardware."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import yaml

CASES = {'forward': (.2, 0., 0.), 'strafe_left': (0., .2, 0.),
         'rotate_ccw': (0., 0., .4), 'diagonal': (.2, .2, 0.)}


def geometry():
    path = Path(get_package_share_directory('qianli_description')) / 'urdf/qianli.urdf.xacro'
    root = ET.fromstring(subprocess.check_output(['xacro', str(path)], text=True))
    props = ET.parse(path.parent / 'common.xacro').getroot()
    p = {e.get('name'): float(e.get('value')) for e in props
         if e.tag.endswith('property') and not e.get('value').startswith('${')}
    cfg = yaml.safe_load((Path(get_package_share_directory('qianli_control')) / 'config/controllers.yaml').read_text())
    cfg = cfg['omni_base_controller']['ros__parameters']
    assert cfg['wheel_names'] == [n+'_wheel_joint' for n in ('front_left', 'rear_left', 'rear_right', 'front_right')]
    assert math.isclose(cfg['robot_radius'], math.hypot(p['wheel_x'], p['wheel_y']), abs_tol=1e-12)
    assert math.isclose(cfg['wheel_radius'], p['wheel_radius'])
    joints = {j.get('name'): j for j in root.findall('joint')}
    bases = {}
    for name in cfg['wheel_names']:
        j = joints[name]
        x, y, _ = map(float, j.find('origin').get('xyz').split())
        roll, pitch, yaw = map(float, j.find('origin').get('rpy').split())
        assert abs(roll) < 1e-12 and math.isclose(pitch, math.pi/2)
        assert j.find('axis').get('xyz') == '0 0 1'
        # Outward axle a, ground contact lever (0,0,-r).
        # No slip in tangential direction t=(a_y,-a_x), gives
        # r*omega=t dot (v + wz*(-y,x)); transverse roller motion is free.
        ax, ay = math.cos(yaw), math.sin(yaw)
        assert ax*x+ay*y > 0
        bases[name] = (x, y, ay, -ax)
    return p['wheel_radius'], bases


def expected(command, radius, bases):
    vx, vy, wz = command
    return {name: (tx*(vx-wz*y)+ty*(vy+wz*x))/radius
            for name, (x, y, tx, ty) in bases.items()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live', action='store_true')
    parser.add_argument('--output', type=Path)
    args, ros_args = parser.parse_known_args()
    radius, bases = geometry()
    report = {name: expected(cmd, radius, bases) for name, cmd in CASES.items()}
    assert report['forward']['front_left_wheel_joint'] > 0
    assert report['strafe_left']['front_left_wheel_joint'] < 0
    assert all(x < 0 for x in report['rotate_ccw'].values())
    assert abs(report['diagonal']['front_left_wheel_joint']) < 1e-12
    if args.live:
        import rclpy
        from geometry_msgs.msg import TwistStamped
        from sensor_msgs.msg import JointState
        from nav_msgs.msg import Odometry
        rclpy.init(args=ros_args)
        node = rclpy.create_node('test_xdrive_kinematics')
        received = {}
        node.create_subscription(JointState, '/joint_states', lambda m: received.update(joints=m), 10)
        node.create_subscription(Odometry, '/odom', lambda m: received.update(odom=m), 10)
        publisher = node.create_publisher(TwistStamped, '/cmd_vel', 10)

        def send(cmd):
            msg = TwistStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'base_footprint'
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = cmd
            publisher.publish(msg)

        def run_for(cmd, seconds):
            deadline = time.monotonic()+seconds
            while time.monotonic() < deadline:
                send(cmd)
                until = time.monotonic()+.05
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=.005)

        try:
            deadline = time.monotonic()+20
            while time.monotonic() < deadline and (len(received) != 2 or publisher.get_subscription_count() == 0):
                rclpy.spin_once(node, timeout_sec=.1)
            assert len(received) == 2 and publisher.get_subscription_count() > 0
            for name, cmd in CASES.items():
                run_for(cmd, 1.0)
                msg = received['joints']
                actual = dict(zip(msg.name, msg.velocity))
                for joint, value in report[name].items():
                    assert abs(actual[joint]-value) < .03, (name, joint, actual, report[name])
                odom = received['odom']
                assert odom.header.frame_id == 'odom' and odom.child_frame_id == 'base_footprint'
                actual_twist = [odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z]
                assert all(abs(a-e) < .02 for a, e in zip(actual_twist, cmd))
                report[name] = {'expected': report[name], 'mock_mirrored_commands': actual, 'odom_twist': actual_twist}
                run_for((0., 0., 0.), .2)
            # Verify command timeout without sending a replacement command.
            run_for((.2, 0., 0.), .3)
            deadline = time.monotonic()+.8
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
            assert all(abs(v) < .01 for v in received['joints'].velocity)
            report['timeout_stop'] = 'PASS'
        finally:
            run_for((0., 0., 0.), .3)
            node.destroy_node()
            rclpy.shutdown()
    report['result'] = 'PASS' if args.live else 'PASS: contact geometry and controller parameter contract'
    text = json.dumps(report, indent=2)
    print(text)
    if args.output:
        args.output.write_text(text)


if __name__ == '__main__':
    main()
