#!/usr/bin/env python3
"""诊断：外标定点到底落在固定爪还是活动爪/机身？

回答用户的核心问题："标定点在不在固定爪的顶端"。

方法：
1. 用当前 /joint_states 的真实臂姿态 + gripper_model 的正运动学；
2. 分别算：TCP / 整机最低点(脚本现在记录的) / 固定爪(gripper_link)最低点 /
   活动爪(moving_jaw)最低点，以及它们之间的水平错开；
3. 扫夹爪角：看"最低点"从固定爪切换到活动爪的临界开度；
4. 对已记录的标定 marks：把 contact_m 的 z 和板面 z 对比，
   判断哪些点其实记录的是"低于固定爪尖的活动爪/机身"。

用法: ~/mj/bin/python diag_fixed_jaw.py [--marks /tmp/extrinsic_marks.json]
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.expanduser('~/QianLi/qianli_ws/src/qianli_vision/scripts'))
from gripper_model import GripperModel, JOINTS, JAW_LINK, FLANGE_LINK

BOARD_Z = -0.06859   # 桌面 -0.06909 + 棋盘纸 0.5mm


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--marks', default='/tmp/extrinsic_marks.json')
    ap.add_argument('--q', nargs=6, type=float, default=None)
    args = ap.parse_args()

    model = GripperModel(stride=6)
    fails = model.selftest()
    if fails:
        print('❌ FK 自检未通过:', fails)

    # ---- 当前臂姿态 ----
    if args.q:
        joints = dict(zip(JOINTS, args.q))
    else:
        import rclpy
        import time
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        rclpy.init()
        node = Node('diag_fixed_jaw')
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
        joints = {k: float(got[k]) for k in JOINTS}
        node.destroy_node()
        rclpy.try_shutdown()

    print('臂姿态: ' + ', '.join(f'{k}={joints[k]:+.3f}' for k in JOINTS[:5]))
    print(f'夹爪角 gripper = {joints["gripper"]:+.3f} rad '
          f'({math.degrees(joints["gripper"]):+.1f}°)')
    print()

    T = model.solve(joints)

    def low_of(link):
        M = T.get(link)
        if M is None or link not in model.parts or not len(model.parts[link]):
            return None
        w = (M[:3, :3] @ model.parts[link].T).T + M[:3, 3]
        return w[np.argmin(w[:, 2])]

    tcp = model.tcp(joints)
    low_all, low_link = model.lowest_point(joints)
    low_fixed = low_of(FLANGE_LINK)   # 固定爪(gripper_link)
    low_moving = low_of(JAW_LINK)     # 活动爪(moving_jaw)

    print('=' * 78)
    print('  当前姿态下各参考点的 base_link 坐标')
    print('=' * 78)
    if tcp is not None:
        print(f'  TCP (tcp_link)      = ({tcp[0]:+.4f}, {tcp[1]:+.4f}, '
              f'{tcp[2]:+.4f})  z-板面 = {(tcp[2]-BOARD_Z)*1000:+5.2f} mm')
    print(f'  整机最低点(现记录) = ({low_all[0]:+.4f}, {low_all[1]:+.4f}, '
          f'{low_all[2]:+.4f})  属于 {low_link}  '
          f'z-板面 = {(low_all[2]-BOARD_Z)*1000:+5.2f} mm')
    if low_fixed is not None:
        print(f'  固定爪最低点       = ({low_fixed[0]:+.4f}, {low_fixed[1]:+.4f}, '
              f'{low_fixed[2]:+.4f})  z-板面 = {(low_fixed[2]-BOARD_Z)*1000:+5.2f} mm')
    if low_moving is not None:
        print(f'  活动爪最低点       = ({low_moving[0]:+.4f}, {low_moving[1]:+.4f}, '
              f'{low_moving[2]:+.4f})  z-板面 = {(low_moving[2]-BOARD_Z)*1000:+5.2f} mm')
    print()
    if low_fixed is not None and low_moving is not None:
        dh = math.hypot(low_moving[0]-low_fixed[0], low_moving[1]-low_fixed[1])
        dv = (low_moving[2]-low_fixed[2])*1000
        print(f'  活动爪最低点相对固定爪最低点: 水平 {dh*1000:.1f} mm, '
              f'垂直 {dv:+.1f} mm')
    if low_link == JAW_LINK:
        print('  ⚠️ 整机最低点现在是【活动爪】—— 用户对准固定爪，'
              '但记录的是活动爪，水平差 = 爪口开度！')
    elif low_link == FLANGE_LINK:
        print('  ✅ 整机最低点现在是【固定爪】')
    print()

    # ---- 扫夹爪角：最低点在固定爪/活动爪之间切换 ----
    print('=' * 78)
    print('  扫夹爪角: 最低点落在哪个部件 (其余关节不变)')
    print('=' * 78)
    print(f'  {"gripper(rad)":>12}{"gripper(°)":>11}{"最低点z(mm)":>13}'
          f'{"z-板面(mm)":>12}  部件')
    switched = []
    prev = None
    for g in np.arange(-0.13, 1.75, 0.0625):
        j = dict(joints)
        j['gripper'] = float(g)
        low, link = model.lowest_point(j)
        if prev is not None and link != prev:
            switched.append((g, link))
        prev = link
        mark = ''
        if link == JAW_LINK:
            mark = '  ← 活动爪'
        print(f'{g:>12.3f}{math.degrees(g):>11.1f}{low[2]*1000:>13.2f}'
              f'{(low[2]-BOARD_Z)*1000:>12.2f}  {link}{mark}')
    print()
    if switched:
        print(f'  切换点: {[(round(g,3), l) for g, l in switched]}')
        print('  ⚠️ 夹爪开度越大，最低点越容易落到活动爪 → '
              '标定参考点随开度在固定/活动爪之间跳！')
    print()

    # ---- 已记录的标定点检查 ----
    if os.path.exists(args.marks):
        marks = json.load(open(args.marks))
        print('=' * 78)
        print(f'  已记录标定点检查 ({len(marks)} 点): contact z vs 板面 {BOARD_Z}')
        print('=' * 78)
        print(f'  {"#":>3}{"grid(cm)":>11}{"接触点z(mm)":>13}'
              f'{"z-板面(mm)":>12}  判定')
        bad = 0
        for i, m in enumerate(marks, 1):
            cz = m['contact_m'][2]
            d = (cz - BOARD_Z) * 1000
            if abs(d) > 2.0:
                verdict = '❌ 最低点在板面下/上 → 记录的不是接触点'
                bad += 1
            else:
                verdict = '✅ 在板面附近'
            print(f'{i:>3}{(str(m["grid_cm"])):>11}{cz*1000:>13.2f}{d:>12.2f}  {verdict}')
        print(f'\n  ⚠️ {bad}/{len(marks)} 点的"接触点"离板面超过 2mm '
              f'→ 这些点记录的坐标不是用户对准的固定爪尖。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
