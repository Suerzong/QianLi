#!/usr/bin/env python3
"""在 RViz 里标出 TCP / 旧参考点 / 夹爪最低点 / 桌面平面

为什么需要
----------
URDF 里的 `tcp_link` 只是坐标系，RViz 默认只画一小段轴，根本看不清它
落在爪子的哪个位置。这里用 Marker 画成球 + 文字，一眼就能核对：

  · 红球  tcp_link              —— 新定义的抓取点
  · 蓝球  gripper_frame_link    —— 旧参考点
  · 绿球  夹爪几何最低点        —— 决定"离桌面多高"的那个点
  · 灰板  实测桌面平面 z=-69.09mm

姿态来源：
  --source ros   跟随 /joint_states（真机关节角）
  --source gui   跟随 joint_state_publisher 的关节角
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gripper_model import GripperModel, JOINTS  # noqa: E402

TABLE_Z = -0.06909


def sphere(mid, ns, pos, color, scale, text=None):
    m = Marker()
    m.header.frame_id = 'base_link'
    m.ns = ns
    m.id = mid
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = (
        float(pos[0]), float(pos[1]), float(pos[2]))
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = scale
    m.color.r, m.color.g, m.color.b, m.color.a = color
    return m


def label(mid, ns, pos, color, text, scale=0.012):
    m = Marker()
    m.header.frame_id = 'base_link'
    m.ns = ns
    m.id = mid
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = (
        float(pos[0]), float(pos[1]), float(pos[2]) + scale * 1.6)
    m.pose.orientation.w = 1.0
    m.scale.z = scale
    m.color.r, m.color.g, m.color.b, m.color.a = color
    m.text = text
    return m


def box(mid, ns, center, size, color):
    m = Marker()
    m.header.frame_id = 'base_link'
    m.ns = ns
    m.id = mid
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = center
    m.pose.orientation.w = 1.0
    m.scale.x, m.scale.y, m.scale.z = size
    m.color.r, m.color.g, m.color.b, m.color.a = color
    return m


def line(mid, ns, p0, p1, color, width=0.002):
    """从 p0 到 p1 的线段，用来把"间隙"画出来。"""
    m = Marker()
    m.header.frame_id = 'base_link'
    m.ns = ns
    m.id = mid
    m.type = Marker.LINE_LIST
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = width
    m.color.r, m.color.g, m.color.b, m.color.a = color
    from geometry_msgs.msg import Point
    m.points = [Point(x=float(p0[0]), y=float(p0[1]), z=float(p0[2])),
                Point(x=float(p1[0]), y=float(p1[1]), z=float(p1[2]))]
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--table-z', type=float, default=TABLE_Z)
    ap.add_argument('--stride', type=int, default=16)
    ap.add_argument('--gripper-ref', type=float, default=1.745,
                    help='桌面标定时夹爪的角度（rad，张开）。因为"最低点"'
                         '随开合角能变 6mm，安全判断必须用同一口径。')
    args, _unknown = ap.parse_known_args()

    model = GripperModel(stride=args.stride)
    fails = model.selftest()
    if fails:
        print('❌ FK 自检未通过：', fails)
        return 1

    rclpy.init()
    node = Node('tcp_markers')
    joints = {}

    def on_joints(m):
        if len(m.name) == len(m.position):
            joints.clear()
            joints.update(dict(zip(m.name, m.position)))

    node.create_subscription(JointState, '/joint_states', on_joints, 10)
    pub = node.create_publisher(MarkerArray, '/tcp_markers', 1)

    print(f'标记节点已启动（桌面 z={args.table_z*1000:+.2f}mm）；'
          f'等待 /joint_states …', flush=True)
    got = False
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if len(joints) < 6:
            continue
        if not got:
            print(f'  收到关节角: '
                  f'{ {k: round(joints[k],3) for k in JOINTS} }', flush=True)
            got = True
        T = model.solve(joints)
        tcp = T['tcp_link'][:3, 3]
        flange = T['gripper_frame_link'][:3, 3]
        low, link = model.lowest_point(joints)
        # 同一姿态、把夹爪角换成标定时的角度再算一次：
        # "最低点"随开合角能变 6mm（活动爪闭合时甩到固定爪下面），
        # 只有同一口径才能和桌面平面比较。
        ref_joints = dict(joints)
        ref_joints['gripper'] = args.gripper_ref
        low_ref, link_ref = model.lowest_point(ref_joints)
        gap = (low_ref[2] - args.table_z) * 1000

        arr = MarkerArray()
        # 桌面板画薄、半透明，免得把下面的东西吞掉
        arr.markers.append(box(0, 'table', [0.30, 0.0, args.table_z - 0.0015],
                              [0.8, 0.8, 0.003], (0.60, 0.60, 0.64, 0.55)))
        arr.markers.append(sphere(1, 'tcp', tcp, (1.0, 0.0, 0.75, 1.0), 0.016))
        arr.markers.append(label(2, 'tcp', tcp, (1.0, 0.3, 0.85, 1.0),
                                 f'TCP  z={tcp[2]*1000:+.1f}mm', 0.016))
        arr.markers.append(sphere(3, 'flange', flange,
                                  (0.15, 0.55, 1.0, 1.0), 0.012))
        arr.markers.append(label(4, 'flange', flange, (0.4, 0.7, 1.0, 1.0),
                                 f'gripper_frame_link  z={flange[2]*1000:+.1f}mm',
                                 0.013))
        # 最低点：小球 + 一条竖线画间隙（青=在上方，红=在下方）。
        # 用青色是因为机械臂是不透明的黄色，青黄对比度最高；
        # 而标记要"戳出"网格外面才看得见，所以半径给得比爪口还大一圈。
        gcol = (0.0, 1.0, 1.0, 1.0) if gap >= 0 else (1.0, 0.1, 0.1, 1.0)
        arr.markers.append(sphere(5, 'lowest_ref', low_ref, gcol, 0.011))
        arr.markers.append(line(8, 'gap', [low_ref[0], low_ref[1], args.table_z],
                                [low_ref[0], low_ref[1], low_ref[2]],
                                gcol, 0.003))
        arr.markers.append(label(6, 'lowest_ref', low_ref, gcol,
                                 f'lowest @gripper={args.gripper_ref:.2f}rad  '
                                 f'z={low_ref[2]*1000:+.1f}mm  gap {gap:+.1f}mm'
                                 f'  [{link_ref[:12]}]', 0.013))
        # 实际夹爪角下的最低点（灰色小球，仅作对照）
        arr.markers.append(sphere(9, 'lowest_now', low,
                                  (0.85, 0.85, 0.85, 0.9), 0.008))
        arr.markers.append(label(10, 'lowest_now', low, (0.9, 0.9, 0.9, 1.0),
                                 f'lowest @now gripper={joints["gripper"]:.2f}rad'
                                 f'  z={low[2]*1000:+.1f}mm  [{link[:12]}]',
                                 0.011))
        arr.markers.append(label(7, 'info', [0.0, 0.0, 0.40],
                                 (1.0, 1.0, 1.0, 1.0),
                                 f'table z={args.table_z*1000:+.1f}mm   '
                                 f'TCP-lowest(ref)={(tcp[2]-low_ref[2])*1000:+.1f}mm',
                                 0.016))
        pub.publish(arr)

    try:
        node.destroy_node()
    except Exception:  # noqa: BLE001
        pass
    if rclpy.ok():
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
