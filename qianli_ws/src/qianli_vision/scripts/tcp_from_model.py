#!/usr/bin/env python3
"""从模型直接定 TCP：找两爪"最接近处"→ 爪口抓取点 → 生成 tcp_link

思路（不靠手工测量，也不靠手感）
--------------------------------
SO-101 是"一片活动爪 + 一片固定爪"的夹爪。把活动爪转到**闭合**位置时，
两片爪子会互相接触 —— 那块接触面对应的就是"零尺寸物块"该待的位置，
也就是抓取点所在。于是：

1. 把 gripper_link（固定爪）与 moving_jaw（活动爪）的网格都变换到
   `gripper_frame_link` 坐标系；
2. 体素降采样后求两片爪之间的**最近点对**；
3. 取最近点附近一小片区域的**中点**作为抓取点（避免单个顶点噪声）；
4. 输出该点在 `gripper_link` / `gripper_frame_link` 下的坐标，
   并生成可直接插进 URDF 的 `tcp_link` 片段。

用法::

    ~/mj/bin/python tcp_from_model.py --angle -0.174533
    ~/mj/bin/python tcp_from_model.py --angle -0.174533 --emit-urdf /tmp/tcp_link.urdf
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

DEFAULT_URDF = os.path.expanduser(
    '~/legacy/arm/arm-final/ros2_ws/install/so101_bringup/share/so101_bringup'
    '/urdf/so101.urdf')


def load_stl(path):
    with open(path, 'rb') as fh:
        fh.read(80)
        count = struct.unpack('<I', fh.read(4))[0]
        data = np.frombuffer(fh.read(count * 50), dtype=np.uint8).reshape(count, 50)
        return data[:, 12:48].copy().view('<f4').reshape(-1, 3).astype(float)


def rpy_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def xyz(text, default=(0, 0, 0)):
    return (np.array([float(v) for v in text.split()]) if text
            else np.array(default, float))


class Model:
    def __init__(self, path):
        self.root = ET.parse(path).getroot()
        self.assets = Path(path).parent / 'assets'
        self.links = {l.get('name'): l for l in self.root.findall('link')}
        self.joints = {j.get('name'): j for j in self.root.findall('joint')}

    def tf(self, name, angle=0.0):
        j = self.joints[name]
        o = j.find('origin')
        t = xyz(o.get('xyz') if o is not None else None)
        r = xyz(o.get('rpy') if o is not None else None)
        R = rpy_matrix(*r)
        if j.get('type') == 'revolute':
            R = R @ rot_z(angle)
        return R, t

    def points(self, link):
        chunks = []
        for vis in self.links[link].findall('visual'):
            mesh = vis.find('geometry/mesh')
            if mesh is None:
                continue
            p = self.assets / os.path.basename(mesh.get('filename'))
            if not p.exists():
                continue
            o = vis.find('origin')
            t = xyz(o.get('xyz') if o is not None else None)
            r = xyz(o.get('rpy') if o is not None else None)
            v = load_stl(p)
            chunks.append((rpy_matrix(*r) @ v.T).T + t)
        return np.vstack(chunks) if chunks else np.zeros((0, 3))


def voxel_downsample(pts, size):
    if len(pts) == 0:
        return pts
    keys = np.round(pts / size).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[np.sort(idx)]


def nearest_pairs(a, b, chunk=2000):
    """分块求 a 中每个点到 b 的最近距离与最近点。"""
    best_d = np.full(len(a), np.inf)
    best_j = np.zeros(len(a), dtype=np.int64)
    for start in range(0, len(a), chunk):
        block = a[start:start + chunk]
        d = np.linalg.norm(block[:, None, :] - b[None, :, :], axis=2)
        j = np.argmin(d, axis=1)
        best_d[start:start + chunk] = d[np.arange(len(block)), j]
        best_j[start:start + chunk] = j
    return best_d, best_j


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--urdf', default=DEFAULT_URDF)
    ap.add_argument('--angle', type=float, default=-0.174533,
                    help='gripper 关节角（rad），默认取 URDF 下限=闭合')
    ap.add_argument('--voxel', type=float, default=0.0015)
    ap.add_argument('--band', type=float, default=0.0015,
                    help='取最近点附近这么宽的一片取中点')
    ap.add_argument('--front-mm', type=float, default=25.0,
                    help='只在"距爪尖这么近"的前端区域找最近点。'
                         '不限制的话最近点会落在爪子的铰链处（整片 gripper_link '
                         '外壳离活动爪最近的地方是转轴，不是夹取面）')
    ap.add_argument('--object-mm', type=float, default=14.0,
                    help='目标物块宽度（mm）。TCP 横向 = 固定爪内侧面 - W/2')
    ap.add_argument('--emit-urdf', help='把 tcp_link 片段写到该文件')
    args = ap.parse_args()

    m = Model(args.urdf)
    R_f, t_f = m.tf('gripper_frame_joint')          # gripper_link -> frame
    R_j, t_j = m.tf('gripper', args.angle)          # gripper_link -> moving jaw

    fixed_l = m.points('gripper_link')
    jaw_l = m.points('moving_jaw_so101_v1_link')
    jaw_in_link = (R_j @ jaw_l.T).T + t_j

    fixed_f = voxel_downsample((R_f.T @ (fixed_l - t_f).T).T, args.voxel)
    jaw_f = voxel_downsample((R_f.T @ (jaw_in_link - t_f).T).T, args.voxel)

    # 只在靠近爪尖的前端区域比较 —— 否则最近点必然是铰链
    tip_z = max(fixed_f[:, 2].max(), jaw_f[:, 2].max())
    z_cut = tip_z - args.front_mm / 1000.0
    fixed_front = fixed_f[fixed_f[:, 2] >= z_cut]
    jaw_front = jaw_f[jaw_f[:, 2] >= z_cut]
    print(f'爪尖平面 z = {tip_z*1000:+.2f} mm；'
          f'只在 z >= {z_cut*1000:+.1f} mm 的前端 {args.front_mm:.0f}mm 内找最近点')
    print(f'前端区域点数：固定爪 {len(fixed_front)}，活动爪 {len(jaw_front)}')
    if len(fixed_front) < 10 or len(jaw_front) < 10:
        print('❌ 前端区域点太少，把 --front-mm 调大一点')
        return 1

    print(f'降采样后：固定爪 {len(fixed_f)} 点，活动爪 {len(jaw_f)} 点')
    print(f'按 gripper 关节角 {args.angle:.6f} rad 求两爪最接近处 …')

    d, j = nearest_pairs(jaw_front, fixed_front)
    k = int(np.argmin(d))
    dmin = float(d[k])
    close = d <= dmin + args.band
    jaw_side = jaw_front[close]
    fixed_side = fixed_front[j[close]]

    print(f'两爪最小间距 = {dmin*1000:.3f} mm（闭合时本应接近 0；'
          f'若明显偏大说明该角度还没夹紧）')
    print(f'参与取中点的点数：活动爪 {len(jaw_side)}，固定爪 {len(fixed_side)}')

    grasp_f = (jaw_side.mean(axis=0) + fixed_side.mean(axis=0)) / 2.0
    print()
    print('━' * 70)
    print(f'抓取点（gripper_frame_link 下）= '
          f'({grasp_f[0]*1000:+.2f}, {grasp_f[1]*1000:+.2f}, '
          f'{grasp_f[2]*1000:+.2f}) mm')

    # 换到 gripper_link：p_link = R_f @ p_frame + t_f
    grasp_link = R_f @ grasp_f + t_f
    print(f'抓取点（gripper_link 下）      = '
          f'({grasp_link[0]*1000:+.2f}, {grasp_link[1]*1000:+.2f}, '
          f'{grasp_link[2]*1000:+.2f}) mm')
    print(f'  → 相对 gripper_frame_link 原点的偏移 '
          f'= ({grasp_f[0]*1000:+.2f}, {grasp_f[1]*1000:+.2f}, '
          f'{grasp_f[2]*1000:+.2f}) mm，|偏移| = '
          f'{np.linalg.norm(grasp_f)*1000:.2f} mm')

    # 参考量：爪尖平面
    print()
    print(f'参考：爪尖平面在 frame z = {tip_z*1000:+.2f} mm '
          f'（抓取点在它后方 {tip_z*1000 - grasp_f[2]*1000:.2f} mm）')

    # 爪口开度（在最接近处所在的 z 层附近量两爪 X 跨度）
    z0 = grasp_f[2]
    band = 0.004
    sel_f = np.abs(fixed_front[:, 2] - z0) < band
    sel_j = np.abs(jaw_front[:, 2] - z0) < band
    if sel_f.sum() and sel_j.sum():
        fmin, fmax = fixed_front[sel_f, 0].min(), fixed_front[sel_f, 0].max()
        jmin, jmax = jaw_front[sel_j, 0].min(), jaw_front[sel_j, 0].max()
        print(f'该处固定爪 X 范围 [{fmin*1000:+.1f}, {fmax*1000:+.1f}] mm')
        print(f'该处活动爪 X 范围 [{jmin*1000:+.1f}, {jmax*1000:+.1f}] mm')

    if args.emit_urdf:
        # ---- 干净模型：固定爪内侧面在 X≈X0，开口中心 = X0 - W/2 ----
        # 实测（三个开度都一致）：固定爪内侧面基本落在 frame X = 0，
        # 活动爪随开度往 -X 走。所以"物块贴着固定爪"与"开口中心"是同一个点。
        band = 0.006
        sel = np.abs(fixed_front[:, 2] - grasp_f[2]) < band
        X0 = float(fixed_front[sel, 0].min()) if sel.any() else 0.0
        tcp_x = X0 - args.object_mm / 2000.0
        tcp_y = 0.0
        tcp_z = float(grasp_f[2])
        print()
        print('━' * 70)
        print(f'  按目标物块宽度 {args.object_mm:.1f} mm 定 TCP：')
        print(f'    固定爪内侧面 X0 = {X0*1000:+.2f} mm')
        print(f'    TCP = ({tcp_x*1000:+.2f}, {tcp_y*1000:+.2f}, '
              f'{tcp_z*1000:+.2f}) mm  (相对 gripper_frame_link)')
        print(f'    即 X0 - W/2，Z 取两爪接触深度')

        snippet = f'''  <!-- ===== TCP：爪口抓取点（tcp_from_model.py 由网格算出） =====
       由两爪最接近处求得；固定爪内侧面落在 frame X≈0，
       开口中心 = X0 - W/2，W 为目标物块宽度。
       本次按 W = {args.object_mm:.1f} mm 生成。
       父节点选 gripper_frame_link，使 tcp_link 成为链末端，
       IK 的 tip 自然就是真正的抓取点。 -->
  <link name="tcp_link">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1e-9"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
    </inertial>
  </link>

  <joint name="tcp_joint" type="fixed">
    <origin xyz="{tcp_x:.6f} {tcp_y:.6f} {tcp_z:.6f}" rpy="0 0 0"/>
    <parent link="gripper_frame_link"/>
    <child link="tcp_link"/>
    <axis xyz="0 0 0"/>
  </joint>
'''
        with open(args.emit_urdf, 'w') as fh:
            fh.write(snippet)
        print(f'\n📄 tcp_link 片段已写入 {args.emit_urdf}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
