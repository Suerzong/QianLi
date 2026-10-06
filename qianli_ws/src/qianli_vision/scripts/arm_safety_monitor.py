#!/usr/bin/env python3
"""基于模型的实时安全判定：整条臂 vs 实测桌面平面

为什么必须"算"而不是"看"
------------------------
撞桌子的是**几何最低点**，不是 TCP，也不一定是夹爪 —— 肘部、前臂在低姿态
下都可能先碰到。而且实测：
  · TCP 相对夹爪最低点的高度随姿态从 +5.1mm 变到 +100.6mm；
  · 夹爪开合角本身就能让最低点移动 6.25mm（张开>43° 时最低点是固定爪，
    闭合时活动爪甩到下面）。
所以肉眼判断不可靠，必须用 FK 扫全部 link 的网格。

判定
----
    clearance = min(所有臂 link 的最低 z) - 桌面 z
    safe      = clearance >= margin

输出
----
  /arm_safety   std_msgs/String, JSON:
      {"safe":bool, "clearance_mm":float, "limiting_link":str,
       "tcp_z_mm":float, "table_z_mm":float, "margin_mm":float, ...}
  /safety_markers  MarkerArray: 最低点小球 + 到桌面的竖线 + 文字

用法::

    ~/mj/bin/python arm_safety_monitor.py --table-z -0.06909 --margin-mm 5
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gripper_model import GripperModel, JOINTS  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--table-z', type=float, default=-0.06909)
    ap.add_argument('--margin-mm', type=float, default=5.0,
                    help='安全裕度：净空 >= 这个值才算安全')
    ap.add_argument('--stride', type=int, default=40)
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray

    model = GripperModel(stride=args.stride)
    fails = model.selftest()
    if fails:
        print('❌ FK 自检未通过：', fails)
        return 1
    model.prepare_clearance(stride=args.stride)
    print(f'已加载 {len(model._clr)} 个 link、'
          f'{sum(len(v) for v in model._clr.values())} 个净空采样点')

    rclpy.init()
    node = Node('arm_safety_monitor')
    joints = {}

    def on_joints(m):
        if len(m.name) == len(m.position):
            joints.clear()
            joints.update(dict(zip(m.name, m.position)))

    node.create_subscription(JointState, '/joint_states', on_joints, 10)
    pub_status = node.create_publisher(String, '/arm_safety', 10)
    pub_mark = node.create_publisher(MarkerArray, '/safety_markers', 1)

    print(f'安全监视启动：桌面 z={args.table_z*1000:+.2f}mm  '
          f'裕度 {args.margin_mm:.1f}mm', flush=True)
    n = 0
    warn_state = None
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        if len(joints) < 6:
            continue
        j = {k: float(joints[k]) for k in JOINTS}
        low, link = model.lowest_over_all(j)
        tcp = model.tcp(j)
        clearance = low[2] - args.table_z
        safe = clearance >= args.margin_mm / 1000.0

        msg = String()
        msg.data = json.dumps({
            'safe': bool(safe),
            'clearance_mm': round(clearance * 1000, 2),
            'margin_mm': args.margin_mm,
            'limiting_link': link,
            'limiting_point_m': np.round(low, 5).tolist(),
            'tcp_z_mm': round(float(tcp[2]) * 1000, 2),
            'table_z_mm': round(args.table_z * 1000, 2),
            'gripper_rad': round(j['gripper'], 4),
        })
        pub_status.publish(msg)

        arr = MarkerArray()
        m = Marker()
        m.header.frame_id = 'base_link'
        m.ns = 'safety'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (
            float(low[0]), float(low[1]), float(low[2]))
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.006
        col = (0.1, 0.95, 0.25, 1.0) if safe else (1.0, 0.1, 0.1, 1.0)
        m.color.r, m.color.g, m.color.b, m.color.a = col
        arr.markers.append(m)

        ln = Marker()
        ln.header.frame_id = 'base_link'
        ln.ns = 'safety_line'
        ln.id = 1
        ln.type = Marker.LINE_LIST
        ln.action = Marker.ADD
        ln.pose.orientation.w = 1.0
        ln.scale.x = 0.0022
        ln.color.r, ln.color.g, ln.color.b, ln.color.a = col
        from geometry_msgs.msg import Point
        ln.points = [Point(x=float(low[0]), y=float(low[1]),
                           z=float(args.table_z)),
                     Point(x=float(low[0]), y=float(low[1]), z=float(low[2]))]
        arr.markers.append(ln)

        tx = Marker()
        tx.header.frame_id = 'base_link'
        tx.ns = 'safety_text'
        tx.id = 2
        tx.type = Marker.TEXT_VIEW_FACING
        tx.action = Marker.ADD
        tx.pose.position.x, tx.pose.position.y = float(low[0]), float(low[1])
        tx.pose.position.z = float(low[2]) + 0.030
        tx.pose.orientation.w = 1.0
        tx.scale.z = 0.013
        tx.color.r, tx.color.g, tx.color.b, tx.color.a = col
        tx.text = (f'{"SAFE" if safe else "UNSAFE"}  '
                   f'clearance {clearance*1000:+.1f}mm  [{link}]')
        arr.markers.append(tx)
        pub_mark.publish(arr)

        n += 1
        if n % 50 == 0 or (warn_state is None) or (safe != warn_state):
            warn_state = safe
            print(f'  {"✅ SAFE  " if safe else "⛔ UNSAFE"}  '
                  f'净空 {clearance*1000:+7.2f}mm  '
                  f'受限部件 {link:<28} TCP z={tcp[2]*1000:+7.1f}mm  '
                  f'夹爪 {math.degrees(j["gripper"]):+6.1f}°', flush=True)

    try:
        node.destroy_node()
    except Exception:  # noqa: BLE001
        pass
    if rclpy.ok():
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
