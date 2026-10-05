#!/usr/bin/env python3
"""末端姿态工具：让夹爪"完全朝下 + 指定偏航角"，并验证姿态方向

背景：
  之前的抓取只发位置（/arm/target_position），IK 自由选姿态 →
  每次爪子朝向不确定，可能夹空。工业做法是**指定姿态**。

坐标推导（来自 URDF）：
  gripper_frame_joint: gripper_link → gripper_frame_link，
    origin z=-0.0981（在腕部下方 9.8cm），rpy=(0, π, 0)
  所以 gripper_frame_link 的 +Z 轴 = gripper_link 的 -Z 轴
  = 夹爪的"指向指尖/接近方向"（tool axis）
  活动爪（moving_jaw）枢轴在 gripper_link 的 (0.0202, 0.0188, -0.0234)：
  换算到 frame 坐标 ≈ (-0.028, +0.019, -0.075) → **活动爪在 frame 的 -X 侧**
  因此 frame 的 +X 侧 = 固定爪。

目标姿态（base_link 下）：
  ① 爪子朝下：frame 的 +Z 指向 base 的 -Z   → 绕 X 轴转 180°
  ② 固定爪在右：frame 的 +X 指向 base 的 -Y（+Y 是左，-Y 是右）→ 再绕 Z 转 -90°
  即 R = Rz(yaw) · Rx(π)，默认 yaw = -90°

用法：
  python3 tool_pose.py --yaw -90            # 发一次姿态目标（需 ik_node 在跑）
  python3 tool_pose.py --yaw -90 --verify   # 发目标并读回实际姿态方向
"""

import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

TCP = 'gripper_frame_link'


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def tool_down_yaw(yaw_deg):
    """返回 (R, quaternion)。R = Rz(yaw)·Rx(π)：爪子朝下 + 固定爪朝 base -Y。"""
    R = rot_z(math.radians(yaw_deg)) @ rot_x(math.pi)

    def q_from_R(m):
        tr = m[0, 0] + m[1, 1] + m[2, 2]
        if tr > 0:
            s = math.sqrt(tr + 1.0) * 2
            w = 0.25 * s
            x = (m[2, 1] - m[1, 2]) / s
            y = (m[0, 2] - m[2, 0]) / s
            z = (m[1, 0] - m[0, 1]) / s
        elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
            w = (m[2, 1] - m[1, 2]) / s
            x = 0.25 * s
            y = (m[0, 1] + m[1, 0]) / s
            z = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
            w = (m[0, 2] - m[2, 0]) / s
            x = (m[0, 1] + m[1, 0]) / s
            y = 0.25 * s
            z = (m[1, 2] + m[2, 1]) / s
        else:
            s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
            w = (m[1, 0] - m[0, 1]) / s
            x = (m[0, 2] + m[2, 0]) / s
            y = (m[1, 2] + m[2, 1]) / s
            z = 0.25 * s
        return (x, y, z, w)

    return R, q_from_R(R)


class PoseTool(Node):
    def __init__(self):
        super().__init__('tool_pose')
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.pub = self.create_publisher(PoseStamped, '/ik_target', 10)

    def send(self, xyz, quat):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
        (m.pose.orientation.x, m.pose.orientation.y,
         m.pose.orientation.z, m.pose.orientation.w) = quat
        self.pub.publish(m)

    def tcp(self, retries=40):
        for _ in range(retries):
            try:
                tr = self.buf.lookup_transform('base_link', TCP,
                                               rclpy.time.Time())
                t = tr.transform.translation
                q = tr.transform.rotation
                return (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
        return None, None


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--yaw', type=float, default=-90.0, help='偏航角（度）')
    ap.add_argument('--x', type=float, default=0.30)
    ap.add_argument('--y', type=float, default=-0.02)
    ap.add_argument('--z', type=float, default=0.08)
    ap.add_argument('--verify', action='store_true')
    ap.add_argument('--walk', type=int, default=0,
                    help='分 N 步从当前位置走到目标（给 IK 好种子）')
    ap.add_argument('--per-step', type=float, default=2.5,
                    help='每步停留秒数')
    a = ap.parse_args()

    R, q = tool_down_yaw(a.yaw)
    print(f'目标姿态 R = Rz({a.yaw}°)·Rx(180°)  四元数='
          f'({q[0]:.4f}, {q[1]:.4f}, {q[2]:.4f}, {q[3]:.4f})')
    print('  期望各轴指向（base_link 下）：')
    print(f'    frame +X (固定爪) → {np.round(R @ [1,0,0], 3)}')
    print(f'    frame +Y         → {np.round(R @ [0,1,0], 3)}')
    print(f'    frame +Z (接近方向) → {np.round(R @ [0,0,1], 3)}')

    rclpy.init()
    n = PoseTool()
    time.sleep(1.0)
    goal = (a.x, a.y, a.z)

    if a.walk > 0:
        # 从当前 TCP 位置分步插值到目标：IK 是局部优化，
        # 一步跨太远容易收敛失败，分小步走能自然给出好种子
        cur, _ = n.tcp()
        if cur is None:
            print('❌ 读不到当前 TCP')
            n.destroy_node(); rclpy.shutdown(); return
        print(f'\n当前 TCP = ({cur[0]:.4f}, {cur[1]:.4f}, {cur[2]:.4f})')
        for i in range(1, a.walk + 1):
            t = i / a.walk
            p = tuple(c + (g - c) * t for c, g in zip(cur, goal))
            t0 = time.time()
            while time.time() - t0 < a.per_step:
                n.send(p, q)
                rclpy.spin_once(n, timeout_sec=0.05)
            got, _ = n.tcp()
            err = math.dist(got, p) if got else float('nan')
            print(f'  步 {i}/{a.walk} 目标=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}) '
                  f'→ 实际=({got[0]:.3f},{got[1]:.3f},{got[2]:.3f}) '
                  f'误差 {err*1000:.0f}mm')
    else:
        print(f'\n发送目标 位置={goal}')
        t0 = time.time()
        while time.time() - t0 < 8:
            n.send(goal, q)
            rclpy.spin_once(n, timeout_sec=0.1)

    t, got = n.tcp()
    if t and a.verify:
        Rg = quat_to_R(got)
        print(f'\n最终 TCP 位置 = ({t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f})')
        print('实际各轴指向：')
        print(f'    frame +X → {np.round(Rg @ [1,0,0], 3)}')
        print(f'    frame +Y → {np.round(Rg @ [0,1,0], 3)}')
        print(f'    frame +Z → {np.round(Rg @ [0,0,1], 3)}')
        err = math.acos(max(-1, min(1, (np.trace(R.T @ Rg) - 1) / 2)))
        print(f'姿态误差 = {math.degrees(err):.2f}°')
        pos_err = math.dist(t, goal)
        print(f'位置误差 = {pos_err*1000:.1f} mm')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
