#!/usr/bin/env python3
"""把夹爪尖端安全地下降到桌面附近 —— 自带 IK、限位检查与整机净空检查

为什么不用 ik_node
------------------
ik_node 走的是"发笛卡尔目标 → 它自己解 → 够不到就静默拒绝"。实测：
目标不可达时它保持旧 q_target 不变，机械臂**原地不动也不报错**，
而 elbow_flex 其实已经顶在 +86.57° 的机械死点上了。

本脚本自己算，好处是每一步都能：
  · 按当前姿态**重新**算 TCP 与夹爪最低点的差（那个量随姿态从 5mm 变到 100mm，
    固定目标必然出错）；
  · 显式检查关节限位（从 driver_params.yaml 读实测限位）；
  · 显式检查**整机**离桌面的净空，而不只是夹爪；
  · 不可达就停下并说明，而不是装作在动。

控制律：位置型 DLS，只约束 TCP 位置（不锁姿态），每拍限步。

用法::

    ~/mj/bin/python arm_descend_to_table.py --table-z -0.06909 \
        --gap 30 --step-mm 4 --min-gap 8
"""

from __future__ import annotations

from project_paths import driver_params_path

import argparse
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gripper_model import GripperModel, JOINTS, JAW_LINK  # noqa: E402

ARM_JOINTS = JOINTS[:5]          # 前 5 个是臂关节，gripper 单独保持
DRIVER_CFG = os.path.expanduser(
    driver_params_path())


def read_joint_limits(path=DRIVER_CFG):
    """从 driver_params.yaml 读实测限位，换算成关节角上下限（rad）。"""
    zero = rmin = rmax = None
    for line in open(path):
        line = line.strip()
        if line.startswith('zero_raw:'):
            zero = [int(v) for v in line.split('[', 1)[1].rstrip(']').split(',')]
        elif line.startswith('raw_min:'):
            rmin = [int(v) for v in line.split('[', 1)[1].rstrip(']').split(',')]
        elif line.startswith('raw_max:'):
            rmax = [int(v) for v in line.split('[', 1)[1].rstrip(']').split(',')]
    if not (zero and rmin and rmax):
        raise RuntimeError(f'读不到限位: {path}')
    out = {}
    for i, name in enumerate(JOINTS):
        a = (rmin[i] - zero[i]) * math.tau / 4096
        b = (rmax[i] - zero[i]) * math.tau / 4096
        out[name] = (min(a, b), max(a, b))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--table-z', type=float, default=-0.06909)
    ap.add_argument('--gap', type=float, default=30.0, help='目标间隙(mm)')
    ap.add_argument('--min-gap', type=float, default=8.0, help='硬地板(mm)')
    ap.add_argument('--step-mm', type=float, default=4.0, help='每拍最大步长(mm)')
    ap.add_argument('--rate', type=float, default=15.0)
    ap.add_argument('--clearance-mm', type=float, default=3.0,
                    help='整机任何部件离桌面的最小允许净空(mm)')
    ap.add_argument('--max-ticks', type=int, default=3000)
    args = ap.parse_args()

    if args.gap < args.min_gap:
        print(f'❌ 目标间隙 {args.gap}mm 低于硬地板 {args.min_gap}mm')
        return 1

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_srvs.srv import SetBool

    limits = read_joint_limits()
    print('实测关节限位（rad）:')
    for k in JOINTS:
        print(f'  {k:<14} [{limits[k][0]:+.3f}, {limits[k][1]:+.3f}]'
              f'  = [{math.degrees(limits[k][0]):+7.2f}, '
              f'{math.degrees(limits[k][1]):+7.2f}]°')

    model = GripperModel(stride=14)
    fails = model.selftest()
    if fails:
        print('❌ FK 自检未通过：', fails)
        return 1
    # 整机净空检查用的点云（所有会动的 link）
    clearance_parts = {}
    for link in ('shoulder_link', 'upper_arm_link', 'lower_arm_link',
                 'wrist_link', 'gripper_link', JAW_LINK):
        p = model.link_points(link)
        if len(p):
            clearance_parts[link] = p[::60]

    rclpy.init()
    node = Node('arm_descend')
    joints = {}
    node.create_subscription(
        JointState, '/joint_states',
        lambda m: joints.update(dict(zip(m.name, m.position)))
        if len(m.name) == len(m.position) else None, 10)
    pub = node.create_publisher(JointState, '/joint_commands', 10)
    cli = node.create_client(SetBool, '/arm/enable')

    def spin(sec):
        end = time.monotonic() + sec
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    t0 = time.monotonic()
    while rclpy.ok() and len(joints) < 6 and time.monotonic() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.05)
    if len(joints) < 6:
        print('❌ 收不到 /joint_states')
        return 1

    if not cli.wait_for_service(timeout_sec=5.0):
        print('❌ /arm/enable 不可用')
        return 1
    req = SetBool.Request()
    req.data = True
    fut = cli.call_async(req)
    spin(2.0)
    print('\n使能:', fut.result().message if fut.done() else '?')
    spin(1.5)

    q = np.array([joints[k] for k in JOINTS], dtype=float)
    q_fixed_gripper = q[5]

    def fk_state(qq):
        d = dict(zip(JOINTS, qq))
        T = model.solve(d)
        tcp = T['tcp_link'][:3, 3]
        low, link = model.lowest_point(d)
        zmin = None
        for lk, pts in clearance_parts.items():
            M = T.get(lk)
            if M is None:
                continue
            world = (M[:3, :3] @ pts.T).T + M[:3, 3]
            z = world[:, 2].min()
            zmin = z if zmin is None else min(zmin, z)
        return T, tcp, low, link, zmin

    def tcp_of(qq):
        d = dict(zip(JOINTS, qq))
        return model.solve(d)['tcp_link'][:3, 3]

    T, tcp, low, link, zmin = fk_state(q)
    print(f'起始：TCP z={tcp[2]*1000:+.1f}mm  夹爪最低点 z={low[2]*1000:+.1f}mm '
          f'[{link[:12]}]  整机最低 z={zmin*1000:+.1f}mm')
    print(f'      距桌面 {(low[2]-args.table_z)*1000:+.1f}mm   '
          f'桌面 z={args.table_z*1000:+.1f}mm')

    xy = tcp[:2].copy()
    step = args.step_mm / 1000.0
    lam = 2e-4
    dt = 1.0 / args.rate

    print(f'\n=== 下降到夹爪最低点距桌面 {args.gap:.0f}mm（硬地板 '
          f'{args.min_gap:.0f}mm，每拍 ≤{args.step_mm:.0f}mm）===')
    last_report = 0.0
    stall = 0
    for tick in range(args.max_ticks):
        if not rclpy.ok():
            break
        T, tcp, low, link, zmin = fk_state(q)
        gap_now = low[2] - args.table_z
        # 整机净空保护
        if zmin - args.table_z < args.clearance_mm / 1000.0:
            print(f'  ⛔ 整机净空只剩 {(zmin-args.table_z)*1000:.1f}mm，停止下降')
            break
        if gap_now <= args.gap / 1000.0 + 1e-4:
            print(f'  ✅ 到位：夹爪最低点距桌面 {gap_now*1000:+.1f}mm')
            break
        # 目标 TCP：让"最低点"落在 gap 上 —— 每拍按当前姿态重算偏移
        offset = tcp[2] - low[2]
        want_tcp_z = args.table_z + args.gap / 1000.0 + offset
        target = np.array([xy[0], xy[1], want_tcp_z])
        # 每拍限步
        delta = target - tcp
        if np.linalg.norm(delta) > step:
            delta = delta / np.linalg.norm(delta) * step
        # DLS
        eps = 1e-4
        J = np.zeros((3, 5))
        for i in range(5):
            qq = q.copy()
            qq[i] += eps
            if not (limits[JOINTS[i]][0] <= qq[i] <= limits[JOINTS[i]][1]):
                continue
            J[:, i] = (tcp_of(qq) - tcp) / eps
        dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(3), delta)
        q_new = q.copy()
        for i in range(5):
            lo, hi = limits[JOINTS[i]]
            q_new[i] = min(max(q[i] + dq[i], lo), hi)
        moved = float(np.linalg.norm(tcp_of(q_new) - tcp))
        if moved < 1e-5:
            stall += 1
            if stall > 30:
                print(f'  ⛔ 连续 {stall} 拍没有有效位移 —— 该方向已达运动学边界'
                      f'（当前间隙 {gap_now*1000:+.1f}mm）')
                break
        else:
            stall = 0
        q = q_new
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = list(JOINTS)
        msg.position = [float(v) for v in q]
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.005)
        now = time.monotonic()
        if now - last_report > 1.5:
            last_report = now
            print(f'    t={now-t0:5.1f}s  TCP z={tcp[2]*1000:+7.1f}  '
                  f'最低点距桌面 {gap_now*1000:+6.1f}mm  '
                  f'整机净空 {(zmin-args.table_z)*1000:+6.1f}mm')
        time.sleep(dt)

    # 保持
    print('\n=== 保持中（Ctrl+C / kill 后约 1s 看门狗会断扭矩，机械臂失力）===')
    try:
        while rclpy.ok():
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = list(JOINTS)
            msg.position = [float(v) for v in q]
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        pass
    try:
        node.destroy_node()
    except Exception:  # noqa: BLE001
        pass
    if rclpy.ok():
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
