#!/usr/bin/env python3
"""当前姿态下夹爪各关键点的 base_link 高度 —— 说明"TCP 高度"≠"接触高度"

"让爪子碰桌子、读 TCP 的 Z" 这个做法不成立：

* 碰到桌子的是**夹爪几何最低点**，不是 TCP；
* TCP 是爪口里的抓取中心，沿工具轴在爪尖**上方**几个毫米；
* TCP 还沿工具轴横向偏约 7mm，工具倾斜 θ 时额外贡献 7·sin(θ)；
* 倾斜更大时，最低点会从爪尖换成夹爪机身。

本脚本用 GripperModel（带 FK 自检）给出真实数值。

用法::

    ~/mj/bin/python gripper_clearance.py
    ~/mj/bin/python gripper_clearance.py --joints shoulder_pan=0,shoulder_lift=1.5,...
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

from gripper_model import GripperModel, JOINTS


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--stride', type=int, default=6)
    ap.add_argument('--joints', help='手动指定关节角，如 '
                                     'shoulder_pan=0,shoulder_lift=1.5,...')
    args = ap.parse_args()

    model = GripperModel(stride=args.stride)
    fails = model.selftest()
    if fails:
        print('❌ FK 自检未通过，数值不可信：')
        for f in fails:
            print('   ·', f)
        return 1

    if args.joints:
        joints = {n: 0.0 for n in JOINTS}
        for item in args.joints.split(','):
            k, v = item.split('=')
            joints[k.strip()] = float(v)
        source = '命令行'
    else:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        rclpy.init()
        node = Node('gripper_clearance')
        got = {}

        def cb(msg):
            if len(msg.name) == len(msg.position):
                got.clear()
                got.update(dict(zip(msg.name, msg.position)))

        node.create_subscription(JointState, '/joint_states', cb, 10)
        t0 = time.time()
        while rclpy.ok() and len(got) < 6 and time.time() - t0 < 10:
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(got) < 6:
            print('❌ 收不到 6 个关节的 /joint_states —— 驱动在跑吗？')
            return 1
        joints = {k: float(got[k]) for k in JOINTS}
        source = '/joint_states'
        node.destroy_node()
        rclpy.try_shutdown()

    T = model.solve(joints)
    tcp = T['tcp_link'][:3, 3]
    flange = T['gripper_link'][:3, 3]
    low, low_link = model.lowest_point(joints)
    tool = T['tcp_link'][:3, 2]
    tilt = math.degrees(math.acos(min(1.0, abs(float(tool[2])))))

    print(f'关节角来源 {source}')
    print('  ' + ', '.join(f'{k}={joints[k]:+.4f}' for k in JOINTS))
    print()
    print('=' * 70)
    print('  当前姿态下各关键点的 base_link Z')
    print('=' * 70)
    print(f'  TCP (tcp_link)          {tcp[2]*1000:+9.2f} mm')
    print(f'  gripper_link            {flange[2]*1000:+9.2f} mm')
    print(f'  夹爪几何最低点          {low[2]*1000:+9.2f} mm   [{low_link}]')
    print(f'      最低点 XY           ({low[0]*1000:+.1f}, {low[1]*1000:+.1f}) mm')
    print()
    print(f'  → TCP 比夹爪最低点高 {(tcp[2]-low[2])*1000:+.2f} mm')
    print(f'  → 爪尖碰到桌面时，读到的 TCP Z = 桌面高度 + '
          f'{(tcp[2]-low[2])*1000:.2f} mm，**不是**桌面高度')
    print(f'  → 工具轴 {np.round(tool,3).tolist()}，偏离竖直 {tilt:.1f}°')
    print(f'     横向偏置 7mm 造成的额外高度差 ≈ '
          f'{7.0*math.sin(math.radians(tilt)):.2f} mm')
    print()
    print('  结论：TCP 的 Z 会随姿态漂移，不能当桌面高度的尺子；')
    print('        桌面标定要记录**夹爪最低点**（姿态无关）。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
