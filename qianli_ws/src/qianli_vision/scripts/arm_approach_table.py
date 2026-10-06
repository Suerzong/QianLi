#!/usr/bin/env python3
"""让夹爪尖端分档接近桌面并停住 —— 验证"我是否真的知道桌面在哪"

为什么不能直接命令 TCP 的 Z
----------------------------
实测过：**TCP 相对夹爪最低点的高度随姿态从 +5.1mm 变到 +100.6mm**。
所以要控制的是"**夹爪最低点**距桌面的间隙"，不是 TCP 的 Z。

本脚本做闭环：
    1. 用 gripper_model 的 FK 实时算出当前夹爪最低点；
    2. 把"TCP 目标 Z"按 (期望最低点 - 实际最低点) 修正；
    3. 迭代几轮，直到实际最低点落在期望间隙 ±1mm。

安全
----
* `--min-gap` 是硬地板，任何情况下都不会命令低于它（默认 6mm）；
* 只走小步，每档停住观察；
* 全程 10Hz 发目标，避免 1s 看门狗断扭矩导致失力下垂；
* 从当前位置若已在硬地板以下，**先抬到安全高度再开始下降**。

用法::

    ~/mj/bin/python arm_approach_table.py --table-z -0.06909 \
        --steps 30,20,12,8 --min-gap 6
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gripper_model import GripperModel, JOINTS  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--table-z', type=float, default=-0.06909)
    ap.add_argument('--steps', default='30,20,12,8',
                    help='依次要停住的间隙（mm），从大到小')
    ap.add_argument('--min-gap', type=float, default=6.0,
                    help='硬地板：允许的最小间隙（mm）')
    ap.add_argument('--settle', type=float, default=2.5,
                    help='每档停多久（秒）')
    ap.add_argument('--rate', type=float, default=10.0)
    args = ap.parse_args()

    steps = [float(s) for s in args.steps.split(',')]
    steps = [s for s in steps if s >= args.min_gap]
    if not steps:
        print(f'❌ 所有档位都低于硬地板 {args.min_gap}mm，拒绝执行')
        return 1

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from std_srvs.srv import SetBool
    from tf2_ros import Buffer, TransformListener

    model = GripperModel(stride=12)
    fails = model.selftest()
    if fails:
        print('❌ FK 自检未通过，拒绝运动：')
        for f in fails:
            print('   ·', f)
        return 1

    rclpy.init()
    node = Node('arm_approach_table')
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

    def lowest_gap():
        if len(joints) < 6:
            return None
        low, link = model.lowest_point(joints)
        return low[2] - args.table_z, low, link

    # ---------------- 使能前 ----------------
    print('等待 TF / 关节反馈 …')
    t0 = time.monotonic()
    while rclpy.ok() and (len(joints) < 6 or tcp_pose() is None):
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.monotonic() - t0 > 15:
            print('❌ 超时')
            return 1
    g = lowest_gap()
    print(f'就绪。当前：TCP z={tcp_pose()[0][2]*1000:+.1f}mm  '
          f'夹爪最低点距桌面 {g[0]*1000:+.1f}mm  桌面 z={args.table_z*1000:+.1f}mm')

    print('\n=== 使能扭矩（驱动先读当前位置写回作目标，不跳变）===')
    if not cli.wait_for_service(timeout_sec=5.0):
        print('❌ /arm/enable 不可用')
        return 1
    req = SetBool.Request()
    req.data = True
    fut = cli.call_async(req)
    spin(2.0)
    try:
        print('  ', fut.result().message)
    except Exception as exc:  # noqa: BLE001
        print('  失败:', exc)
        return 1
    spin(1.5)
    g = lowest_gap()
    print(f'  使能后：夹爪最低点距桌面 {g[0]*1000:+.1f}mm')

    pose = tcp_pose()
    pos, quat = pose
    msg = PoseStamped()
    msg.header.frame_id = 'base_link'
    (msg.pose.orientation.x, msg.pose.orientation.y,
     msg.pose.orientation.z, msg.pose.orientation.w) = quat
    msg.pose.position.x, msg.pose.position.y = pos[0], pos[1]

    def hold_for(seconds, tcp_z, stop_flag=None):
        """持续朝 tcp_z 走并保持 seconds 秒；返回最终 (tcp_z, gap)。"""
        msg.pose.position.z = float(tcp_z)
        end = time.monotonic() + seconds
        nxt = 0.0
        while rclpy.ok() and time.monotonic() < end:
            if stop_flag is not None and stop_flag[0]:
                break
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if now >= nxt:
                nxt = now + 1.0
                p = tcp_pose()
                gg = lowest_gap()
                if p is not None and gg is not None:
                    print(f'    TCP z={p[0][2]*1000:+7.1f}mm  '
                          f'最低点距桌面 {gg[0]*1000:+6.1f}mm  '
                          f'[{gg[2][:12]}]')
            time.sleep(max(0.0, 1.0 / args.rate - 0.02))
        return tcp_pose()[0][2], lowest_gap()[0]

    # ---------------- 先抬到安全高度 ----------------
    cur_gap = g[0]
    safe_gap = max(steps) / 1000.0
    if cur_gap < safe_gap:
        print(f'\n=== 当前间隙 {cur_gap*1000:+.1f}mm < {max(steps):.0f}mm，'
              f'先抬到 {max(steps):.0f}mm ===')
        # 用闭环：TCP 目标 = 当前 TCP + (期望最低点 - 当前最低点)
        tz = pos[2] + (safe_gap - cur_gap)
        tz, gg = hold_for(6.0, tz)
        print(f'  抬升完成：最低点距桌面 {gg*1000:+.1f}mm')

    # ---------------- 分档下降 ----------------
    for i, gapmm in enumerate(steps, 1):
        want = gapmm / 1000.0
        print(f'\n=== 第 {i}/{len(steps)} 档：目标间隙 {gapmm:.0f} mm ===')
        # 闭环修正：最多 4 轮，每轮按 (期望 - 实际) 调 TCP 目标
        tz = tcp_pose()[0][2]
        for it in range(1, 5):
            actual = lowest_gap()[0]
            err = want - actual
            tz = tz + err
            print(f'  第 {it} 轮：实际 {actual*1000:+.1f}mm，'
                  f'误差 {err*1000:+.1f}mm → TCP 目标 {tz*1000:+.1f}mm')
            tz_now, actual = hold_for(args.settle, tz)
            if abs(actual - want) <= 0.001:
                print(f'  ✅ 收敛：最低点距桌面 {actual*1000:+.1f} mm')
                break
        else:
            print(f'  ⚠️ 未收敛到 ±1mm，实际 {actual*1000:+.1f} mm')
        # 硬地板复查（保险）
        if actual < args.min_gap / 1000.0 - 1e-6:
            print(f'  ⚠️ 低于硬地板 {args.min_gap}mm！立即抬回')
            tz = tcp_pose()[0][2] + (args.min_gap / 1000.0 - actual + 0.005)
            hold_for(4.0, tz)

    print('\n=== 全部档位完成，保持在最后一档 ===')
    print('（保持中；Ctrl+C 或 kill 后约 1s 扭矩会被看门狗断开，机械臂失力）')
    tz_final = tcp_pose()[0][2]
    try:
        while rclpy.ok():
            msg.pose.position.z = tz_final
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        pass
    print('已停止发布目标')
    try:
        node.destroy_node()
    except Exception:  # noqa: BLE001
        pass
    if rclpy.ok():
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
