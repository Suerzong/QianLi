#!/usr/bin/env python3
"""从 URDF + STL 精确算夹爪几何：爪口开度剖面 → 真正的抓取点(TCP)在哪

为什么不能凭感觉取 TCP
--------------------
`gripper_frame_link` 是 URDF 里的虚拟工具坐标系，它的原点在爪尖后方约 7mm。
把 IK 目标设成这个原点，物块实际会落在爪口的哪一段、能不能夹住，取决于
**两片爪内侧面沿工具轴的开度剖面**。这个剖面只能从网格算。

方法
----
1. 解析 URDF，把 gripper_link / moving_jaw 的网格变换到 `gripper_frame_link`；
2. 沿工具轴(frame +Z)按 1mm 切片；
3. 每片内：活动爪在 -X 侧，取 max(X) 作内侧面；固定爪在 +X 侧，取 min(X)；
   两者之差就是该深度的**爪口净开度**；
4. 于是可以回答："要夹住 Dmm 的物块，工具轴该停在哪一段"。

用法::

    ~/mj/bin/python tcp_geometry.py
    ~/mj/bin/python tcp_geometry.py --angles 0,0.6,1.745
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
        data = np.frombuffer(fh.read(count * 50), dtype=np.uint8)
        data = data.reshape(count, 50)
        verts = data[:, 12:48].copy().view('<f4').reshape(-1, 3)
    return verts.astype(float)


def rpy_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx          # URDF: rpy = Rz(y)Ry(p)Rx(r)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def parse_xyz(text, default=(0, 0, 0)):
    if not text:
        return np.array(default, dtype=float)
    return np.array([float(v) for v in text.split()], dtype=float)


def parse_rpy(text, default=(0, 0, 0)):
    return parse_xyz(text, default)


class Model:

    def __init__(self, urdf_path):
        self.root = ET.parse(urdf_path).getroot()
        self.assets = Path(urdf_path).parent / 'assets'
        self.links = {l.get('name'): l for l in self.root.findall('link')}
        self.joints = {j.get('name'): j for j in self.root.findall('joint')}

    def joint_tf(self, name, angle=0.0):
        j = self.joints[name]
        o = j.find('origin')
        t = parse_xyz(o.get('xyz') if o is not None else None)
        r = parse_rpy(o.get('rpy') if o is not None else None)
        R = rpy_matrix(*r)
        if j.get('type') == 'revolute':
            R = R @ rot_z(angle)
        return R, t

    def link_points(self, link_name):
        """该 link 的全部 visual 网格顶点，在其自身坐标系下。"""
        link = self.links[link_name]
        chunks = []
        for vis in link.findall('visual'):
            mesh = vis.find('geometry/mesh')
            if mesh is None:
                continue
            path = self.assets / os.path.basename(mesh.get('filename'))
            if not path.exists():
                continue
            o = vis.find('origin')
            t = parse_xyz(o.get('xyz') if o is not None else None)
            r = parse_rpy(o.get('rpy') if o is not None else None)
            R = rpy_matrix(*r)
            v = load_stl(path)
            chunks.append((R @ v.T).T + t)
        return np.vstack(chunks) if chunks else np.zeros((0, 3))


def to_frame(points_link, R_l2f, t_l2f):
    """gripper_link 坐标 → gripper_frame_link 坐标。"""
    return (R_l2f.T @ (points_link - t_l2f).T).T


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--urdf', default=DEFAULT_URDF)
    ap.add_argument('--angles', default='0,0.6,1.745',
                    help='gripper 关节角（rad），逗号分隔')
    ap.add_argument('--slice-mm', type=float, default=1.0)
    ap.add_argument('--obj-mm', type=float, default=14.0,
                    help='想夹住的物块边长（mm）')
    args = ap.parse_args()

    model = Model(args.urdf)
    R_f, t_f = model.joint_tf('gripper_frame_joint')

    # gripper_link → moving_jaw（含 gripper 关节角）
    print(f'URDF: {args.urdf}')
    print(f'gripper_frame_joint origin = {np.round(t_f, 6)}  '
          f'(到 gripper_link 的距离 {np.linalg.norm(t_f)*1000:.1f} mm)')
    print()

    fixed_all = model.link_points('gripper_link')
    jaw_all = model.link_points('moving_jaw_so101_v1_link')

    fixed_f = to_frame(fixed_all, R_f, t_f)
    print(f'固定爪（gripper_link {len(fixed_all)} 顶点）在 frame 下：')
    print(f'  沿工具轴 +Z: {fixed_f[:,2].min()*1000:+.1f} ~ '
          f'{fixed_f[:,2].max()*1000:+.1f} mm')

    for angle in [float(a) for a in args.angles.split(',')]:
        R_j, t_j = model.joint_tf('gripper', angle)
        jaw_link = (R_j @ jaw_all.T).T + t_j
        jaw_f = to_frame(jaw_link, R_f, t_f)
        z_min = max(fixed_f[:, 2].min(), jaw_f[:, 2].min())
        z_max = min(fixed_f[:, 2].max(), jaw_f[:, 2].max())
        print()
        print(f'━━ gripper 关节角 {angle:.3f} rad ({math.degrees(angle):.1f}°) ━━')
        print(f'  活动爪沿工具轴: {jaw_f[:,2].min()*1000:+.1f} ~ '
              f'{jaw_f[:,2].max()*1000:+.1f} mm')
        print(f'  沿工具轴开度剖面（{args.slice_mm:.0f}mm 一片）：')
        print(f'    {"深度z(mm)":>10}{"净开度(mm)":>12}{"活动爪内面X":>13}'
              f'{"固定爪内面X":>13}   能否夹住 {args.obj_mm:.0f}mm')
        step = args.slice_mm / 1000.0
        z = math.floor(z_min / step) * step
        usable = []
        while z <= z_max:
            m = (jaw_f[:, 2] >= z) & (jaw_f[:, 2] < z + step)
            f = (fixed_f[:, 2] >= z) & (fixed_f[:, 2] < z + step)
            if m.any() and f.any():
                moving_inner = jaw_f[m, 0].max()      # 活动爪在 -X 侧
                fixed_inner = fixed_f[f, 0].min()     # 固定爪在 +X 侧
                gap = (fixed_inner - moving_inner) * 1000
                fits = gap >= args.obj_mm
                if fits:
                    usable.append(z * 1000)
                print(f'    {z*1000:>10.1f}{gap:>12.1f}'
                      f'{moving_inner*1000:>13.1f}{fixed_inner*1000:>13.1f}'
                      f'   {"✅" if fits else "—"}')
            z += step
        if usable:
            print(f'  → 能容下 {args.obj_mm:.0f}mm 物块的深度范围: '
                  f'{min(usable):+.1f} ~ {max(usable):+.1f} mm '
                  f'(相对 gripper_frame_link 原点，沿工具轴 +Z)')
            center = (min(usable) + max(usable)) / 2.0
            print(f'  → 该范围中点 = {center:+.1f} mm '
                  f'（可作为"抓取中心"候选 TCP 偏移）')
        else:
            print(f'  ⚠️ 没有任何深度能容下 {args.obj_mm:.0f}mm —— '
                  f'物块太大或爪口太窄')
    return 0


if __name__ == '__main__':
    sys.exit(main())
