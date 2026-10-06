#!/usr/bin/env python3
"""使能扭矩 → 保持当前姿态 → 抬高指定量，并实时报告"夹爪最低点 vs 桌面"间隙

安全设计
--------
* 只朝**向上**走（远离桌面），不做任何下降动作；
* 低速（驱动侧 max_joint_speed=0.3 rad/s）；
* 使能时驱动会先读当前位置再写回作为目标，所以不会跳变；
* 持续以 10Hz 发目标，避免 1s 指令看门狗把扭矩断掉（那样会突然失力下垂）；
* 每一步都用 gripper_model 的 FK 独立算一遍夹爪最低点，报告与桌面的间隙。

用法::

    ~/mj/bin/python arm_lift_demo.py --dz 0.015 --table-z -0.06909
    # 会一直保持，直到 Ctrl+C 或外部 kill
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gripper_model import GripperModel, JOINTS  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dz', type=float, default=0.015, help='抬高量（米）')
    ap.add_argument('--table-z', type=float, default=-0.06909)
    ap.add_argument('--hold-s', type=float, default=12.0,
                    help='抬到位后保持多少秒（之后进入持续保持直到被杀）')
    ap.add_argument('--rate', type=float, default=10.0)
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from std_srvs.srv import SetBool
    from tf2_ros import Buffer, TransformListener

    model = GripperModel(stride=12)
    fails = model.selftest()
    if fails:
        print('❌ FK 自检未通过：', fails)
        return 1

    rclpy.init()
    node = Node('arm_lift_demo')
    buf = Buffer()
    TransformListener(buf, node)
    pub = node.create_publisher(PoseStamped, '/ik_target', 10)
    cli = node.create_client(SetBool, '/arm/enable')
    joints = {}

    def on_joints(m):
        if len(m.name) == len(m.position):
            joints.clear()
            joints.update(dict(zip(m.name, m.position)))

    node.create_subscription(JointState, '/joint_states', on_joints, 10)

    def spin(sec):
        end = time.monotonic() + sec
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def tcp_pose():
        try:
            tr = buf.lookup_transform('base_link', 'tcp_link', rclpy.time.Time())
        except Exception:
            return None
        t, q = tr.transform.translation, tr.transform.rotation
        return (np.array([t.x, t.y, t.z]), np.array([q.x, q.y, q.z, q.w]))

    def report(tag):
        if len(joints) < 6:
            print(f'{tag}: 还没有关节反馈')
            return None
        low, link = model.lowest_point(joints)
        tcp = model.tcp(joints)
        gap = (low[2] - args.table_z) * 1000
        print(f'{tag}: 夹爪最低点 z={low[2]*1000:+7.1f}mm [{link[:12]}]  '
              f'TCP z={tcp[2]*1000:+7.1f}mm  '
              f'距桌面 {gap:+6.1f}mm')
        return low, tcp

    print('等待 TF / 关节反馈 …')
    t0 = time.monotonic()
    while rclpy.ok() and (len(joints) < 6 or tcp_pose() is None):
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.monotonic() - t0 > 15:
            print('❌ 超时：拿不到 TF 或关节反馈')
            return 1
    print('就绪\n')

    before = report('使能前')

    print('\n=== 使能扭矩（驱动会先读当前位置写回作为目标，不跳变）===')
    if not cli.wait_for_service(timeout_sec=5.0):
        print('❌ /arm/enable 服务不可用')
        return 1
    req = SetBool.Request()
    req.data = True
    fut = cli.call_async(req)
    spin(2.0)
    try:
        print('  ', fut.result().message)
    except Exception as exc:  # noqa: BLE001
        print('  调用失败:', exc)
        return 1
    spin(1.5)
    after = report('使能后')

    base = tcp_pose()
    if base is None:
        print('❌ 读不到当前 TCP 姿态')
        return 1
    pos, quat = base
    target = pos.copy()
    target[2] += args.dz
    print(f'\n=== 目标：TCP z {pos[2]*1000:+.1f} → {target[2]*1000:+.1f} mm '
          f'(抬高 {args.dz*1000:.1f} mm)，姿态保持不变 ===')

    msg = PoseStamped()
    msg.header.frame_id = 'base_link'
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = target
    (msg.pose.orientation.x, msg.pose.orientation.y,
     msg.pose.orientation.z, msg.pose.orientation.w) = quat

    t_end = time.monotonic() + args.hold_s
    t_next = 0.0
    while rclpy.ok():
        now = time.monotonic()
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.02)
        if now >= t_next:
            t_next = now + 2.0
            p = tcp_pose()
            if p is not None and len(joints) == 6:
                low, _ = model.lowest_point(joints)
                gap = (low[2] - args.table_z) * 1000
                print(f'  t={now-(t_end-args.hold_s):5.1f}s  '
                      f'TCP z={p[0][2]*1000:+7.1f}mm  '
                      f'夹爪最低点 z={low[2]*1000:+7.1f}mm  '
                      f'距桌面 {gap:+6.1f}mm  '
                      f'目标误差 {(p[0][2]-target[2])*1000:+5.1f}mm')
        if now >= t_end and not args.hold_s < 0:
            break
        time.sleep(max(0.0, 1.0 / args.rate - 0.02))

    final = report('\n抬到位后')

    # 保持：继续发目标，直到被外部杀掉（否则 1s 后会失力下垂）
    print('\n=== 保持中（Ctrl+C 或 kill 才结束；结束后会失力下垂）===')
    try:
        while rclpy.ok():
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        pass
    print('已停止发布目标 —— 扭矩会在约 1s 后由看门狗断开，机械臂将失力')
    try:
        node.destroy_node()
    except Exception:  # noqa: BLE001
        pass
    if rclpy.ok():
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
