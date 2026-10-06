#!/usr/bin/env python3
"""夹爪开合角对"夹爪最低点"的影响 —— 桌面标定到底该不该锁夹爪角

背景
----
桌面标定是用"夹爪最低点碰桌面"做的，但**最低点是谁**取决于夹爪开合角：
  固定爪属于 gripper_link，活动爪是 moving_jaw，两者随开合角交换高低。

实测现象：标定时 6 个点的最低点都在 gripper_link 上，
而现在同样的"贴桌面"姿态最低点却在 moving_jaw 上，差了 5.7mm。

本脚本在当前臂姿态下扫夹爪角，看最低点 z 能变多少。
"""

from __future__ import annotations

import argparse
import math
import sys

import numpy as np

from gripper_model import GripperModel, JOINTS, JAW_LINK


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--q', nargs=6, type=float,
                    help='臂关节角（rad）；缺省读 /joint_states')
    ap.add_argument('--table-z', type=float, default=-0.06909)
    args = ap.parse_args()

    model = GripperModel(stride=6)
    fails = model.selftest()
    if fails:
        print('❌ FK 自检未通过：', fails)
        return 1

    if args.q:
        base = dict(zip(JOINTS, args.q))
    else:
        import rclpy
        import time
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        rclpy.init()
        node = Node('jaw_sweep')
        got = {}
        node.create_subscription(
            JointState, '/joint_states',
            lambda m: got.update(dict(zip(m.name, m.position)))
            if len(m.name) == len(m.position) else None, 10)
        t0 = time.time()
        while rclpy.ok() and len(got) < 6 and time.time() - t0 < 8:
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(got) < 6:
            print('❌ 读不到 /joint_states')
            return 1
        base = {k: float(got[k]) for k in JOINTS}
        node.destroy_node()
        rclpy.try_shutdown()

    print('臂关节角: ' + ', '.join(f'{k}={base[k]:+.3f}' for k in JOINTS[:5]))
    print(f'当前夹爪角 = {base["gripper"]:+.3f} rad '
          f'({math.degrees(base["gripper"]):+.1f}°)')
    print()
    print(f'{"gripper(rad)":>13}{"gripper(°)":>11}{"最低点 z(mm)":>14}'
          f'{"相对当前(mm)":>14}  最低点部件')
    print('-' * 78)

    joints = dict(base)
    ref = None
    rows = []
    for g in np.arange(-0.13, 1.75, 0.125):
        joints['gripper'] = float(g)
        low, link = model.lowest_point(joints)
        if ref is None:
            ref = low[2]
        rows.append((float(g), low[2], link))
        print(f'{g:>13.3f}{math.degrees(g):>11.1f}{low[2]*1000:>14.2f}'
              f'{(low[2]-ref)*1000:>14.2f}  {link}')

    zs = [r[1] for r in rows]
    print()
    print(f'最低点 z 的变化范围: {min(zs)*1000:+.2f} ~ {max(zs)*1000:+.2f} mm '
          f'（跨度 {(max(zs)-min(zs))*1000:.2f} mm）')
    links = sorted({r[2] for r in rows})
    print(f'出现过的"最低点"部件: {links}')
    print()
    if (max(zs) - min(zs)) * 1000 > 1.0:
        print('  ⚠️ 夹爪开合角对"最低点"的影响不可忽略。')
        print('     桌面标定必须**锁定夹爪角**（或逐个点记录并换算），')
        print('     否则拟合出来的桌面高度会被这个变化污染。')
    else:
        print('  ✅ 夹爪开合角影响很小，标定可以忽略。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
