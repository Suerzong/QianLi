#!/usr/bin/env python3
"""从网格算夹爪"爪口"的真实几何：空腔中心与开度（按切片做占位分析）

为什么重写
----------
之前那版对每个 z 切片取 min/max X 当"内侧面"，但网格顶点包含整个夹爪外壳，
min/max 会取到外表面，于是画出 25mm 开度、中心偏 7mm 这种可疑结果。

正确做法：在一个切片里，把两片爪的顶点 X 坐标做**占位直方图**，
两爪之间那段**最大的空区间**就是爪口 —— 它的中心和中宽才是要的答案。
"""

from __future__ import annotations

from project_paths import so101_path

import argparse
import math
import os
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

DEFAULT_URDF = os.path.expanduser(
    so101_path('urdf/so101.urdf'))


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
    return np.array([float(v) for v in text.split()]) if text else np.array(default, float)


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


def to_frame(pts, R, t):
    return (R.T @ (pts - t).T).T


def biggest_gap(xs, bin_mm=0.4, min_gap_mm=2.0):
    """在占位直方图里找最大的空区间（= 两爪之间的开口）。"""
    if len(xs) < 2:
        return None
    lo, hi = xs.min(), xs.max()
    if hi - lo < min_gap_mm / 1000:
        return None
    edges = np.arange(lo, hi + bin_mm / 1000, bin_mm / 1000)
    hist, _ = np.histogram(xs, bins=edges)
    occupied = hist > 0
    idx = np.nonzero(occupied)[0]
    if len(idx) < 2:
        return None
    best = None
    for a, b in zip(idx[:-1], idx[1:]):
        if b - a <= 1:
            continue
        width = (b - a - 1) * bin_mm
        if width < min_gap_mm:
            continue
        start = edges[a + 1]
        end = edges[b]
        if best is None or width > best[0]:
            best = (width, (start + end) / 2.0)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--urdf', default=DEFAULT_URDF)
    ap.add_argument('--angles', default='0,0.6,1.745')
    ap.add_argument('--slice-mm', type=float, default=2.0)
    args = ap.parse_args()

    m = Model(args.urdf)
    R_f, t_f = m.tf('gripper_frame_joint')
    fixed = to_frame(m.points('gripper_link'), R_f, t_f)
    jaw0 = m.points('moving_jaw_so101_v1_link')

    print(f'gripper_frame_link 原点相对 gripper_link = {np.round(t_f,6)}')
    print(f'固定爪在 frame 下 z 范围: {fixed[:,2].min()*1000:+.1f} ~ '
          f'{fixed[:,2].max()*1000:+.1f} mm')
    print()

    for angle in [float(a) for a in args.angles.split(',')]:
        R_j, t_j = m.tf('gripper', angle)
        jaw = to_frame((R_j @ jaw0.T).T + t_j, R_f, t_f)
        tips = max(fixed[:, 2].max(), jaw[:, 2].max()) * 1000
        print('━' * 74)
        print(f'gripper 关节角 {angle:.3f} rad ({math.degrees(angle):.1f}°)   '
              f'爪尖平面 z = {tips:+.1f} mm')
        print(f'  {"z(mm)":>8}{"爪口净开度(mm)":>16}{"爪口中心X(mm)":>16}')
        step = args.slice_mm / 1000.0
        zlo = max(fixed[:, 2].min(), jaw[:, 2].min())
        z = math.floor(zlo / step) * step
        rows = []
        while z <= tips:
            sel_f = (fixed[:, 2] >= z) & (fixed[:, 2] < z + step)
            sel_j = (jaw[:, 2] >= z) & (jaw[:, 2] < z + step)
            if sel_f.sum() > 3 and sel_j.sum() > 3:
                xs = np.concatenate([fixed[sel_f, 0], jaw[sel_j, 0]])
                g = biggest_gap(xs)
                if g:
                    rows.append((z * 1000, g[0], g[1] * 1000))
                    print(f'  {z*1000:>8.1f}{g[0]:>16.1f}{g[1]*1000:>16.1f}')
            z += step
        if rows:
            usable = [r for r in rows if r[1] >= 12.0]
            if usable:
                zs = [r[0] for r in usable]
                print(f'  → 净开度 ≥12mm 的深度段: {min(zs):+.1f} ~ {max(zs):+.1f} mm')
                print(f'  → 该段中心 z = {(min(zs)+max(zs))/2:+.1f} mm（相对爪尖 {tips:+.1f}）')
                centers = [r[2] for r in usable]
                print(f'  → 爪口中心 X 中位数 = {np.median(centers):+.2f} mm '
                      f'(散布 {min(centers):+.2f} ~ {max(centers):+.2f})')
        print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
