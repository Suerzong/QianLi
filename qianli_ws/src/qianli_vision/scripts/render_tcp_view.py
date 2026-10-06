#!/usr/bin/env python3
"""把 TCP（爪口抓取点）画在夹爪上 —— 用 PIL 软件渲染网格，标出那个点

用 PIL 而不是 matplotlib：虚拟机里 matplotlib 与 numpy 2.x 不兼容（导入即崩），
PIL 可用且够用。做法是标准画家的算法：三角面按深度排序、Lambert 打光后
用多边形填充。

用法::

    ~/mj/bin/python render_tcp_view.py --out /tmp/tcp_view.png
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
from PIL import Image, ImageDraw, ImageFont

DEFAULT_URDF = os.path.expanduser(
    '~/legacy/arm/arm-final/ros2_ws/install/so101_bringup/share/so101_bringup'
    '/urdf/so101.urdf')
FONT_CANDIDATES = [
    '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
    '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
]


def load_tris(path):
    with open(path, 'rb') as fh:
        fh.read(80)
        count = struct.unpack('<I', fh.read(4))[0]
        data = np.frombuffer(fh.read(count * 50), dtype=np.uint8).reshape(count, 50)
        v = data[:, 12:48].copy().view('<f4').reshape(count, 3, 3)
    return v.astype(float)


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

    def tris(self, link):
        out = []
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
            R = rpy_matrix(*r)
            v = load_tris(p)                       # (n,3,3)
            out.append(np.einsum('ij,nkj->nki', R, v) + t)
        return np.concatenate(out, axis=0) if out else np.zeros((0, 3, 3))


def to_frame(tris, R, t):
    return np.einsum('ij,nkj->nki', R.T, tris - t)


VIEWS = {
    'side': np.array([[1.0, 0, 0], [0, 0, 1.0], [0, -1.0, 0]]),   # X 右, Z 上
    'iso': None,
}


def make_view_iso():
    a = math.radians(-40)
    b = math.radians(22)
    Rz = np.array([[math.cos(a), -math.sin(a), 0],
                   [math.sin(a), math.cos(a), 0], [0, 0, 1]])
    Rx = np.array([[1, 0, 0], [0, math.cos(b), -math.sin(b)],
                   [0, math.sin(b), math.cos(b)]])
    return Rx @ Rz


def draw_panel(draw, tris, colors, view_R, box, title, markers, font, font_s,
               light=np.array([0.4, -0.6, 0.7]), bounds=None):
    x0, y0, x1, y1 = box
    pts = np.einsum('ij,nkj->nki', view_R, tris)
    if bounds is None:
        flat = pts.reshape(-1, 3)
        lo, hi = flat.min(axis=0), flat.max(axis=0)
        span = max(hi[0] - lo[0], hi[1] - lo[1])
        margin = 0.12 * span + 1e-6
        lo, hi = lo - margin, hi + margin
    else:
        lo = np.array([bounds[0], bounds[2]], dtype=float)
        hi = np.array([bounds[1], bounds[3]], dtype=float)
    scale = min((x1 - x0) / (hi[0] - lo[0]), (y1 - y0) / (hi[1] - lo[1]))
    cx, cy = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2
    px, py = (x0 + x1) / 2, (y0 + y1) / 2

    def project(p):
        return (px + (p[0] - cx) * scale, py - (p[1] - cy) * scale)

    # 深度排序（画家算法）：相机在 +z 方向看，远的先画
    order = np.argsort(pts[:, :, 2].mean(axis=1))
    light_n = light / np.linalg.norm(light)
    for idx in order:
        tri = pts[idx]
        # 视锥外直接跳过（放大视图时能省掉大部分三角形）
        if (tri[:, 0].max() < lo[0] or tri[:, 0].min() > hi[0]
                or tri[:, 1].max() < lo[1] or tri[:, 1].min() > hi[1]):
            continue
        ab, ac = tri[1] - tri[0], tri[2] - tri[0]
        n = np.cross(ab, ac)
        nn = np.linalg.norm(n)
        if nn < 1e-12:
            continue
        n = n / nn
        if n[2] < 0:
            n = -n
        shade = 0.35 + 0.65 * max(0.0, float(n @ light_n))
        base = np.array(colors[idx])
        col = tuple(int(np.clip(c * shade, 0, 255)) for c in base)
        draw.polygon([project(tri[0]), project(tri[1]), project(tri[2])], fill=col)

    # 标记（注意：markers 给的是 frame 坐标，要先过视图旋转再投影）
    for p, col, label, r, dy in markers:
        sx, sy = project(view_R @ np.asarray(p, dtype=float))
        if not (x0 - 40 < sx < x1 + 40 and y0 - 40 < sy < y1 + 40):
            continue
        draw.ellipse([sx - r, sy - r, sx + r, sy + r], fill=col,
                     outline=(240, 240, 240), width=2)
        draw.text((sx + r + 6, sy + dy), label, fill=col, font=font_s)

    draw.rectangle(box, outline=(120, 130, 140), width=1)
    draw.text((x0 + 8, y0 + 6), title, fill=(20, 30, 40), font=font)

    # 比例尺 10mm
    sb = 0.010 * scale
    sx0, sy0 = x1 - sb - 24, y1 - 26
    draw.line([sx0, sy0, sx0 + sb, sy0], fill=(20, 20, 20), width=3)
    draw.text((sx0, sy0 - 20), '10 mm', fill=(20, 20, 20), font=font_s)

    # 返回"frame 坐标 -> 屏幕坐标"的投影函数，供调用方画标注
    return lambda p: project(view_R @ np.asarray(p, dtype=float))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--urdf', default=DEFAULT_URDF)
    ap.add_argument('--angle', type=float, default=-0.0233)
    ap.add_argument('--object-mm', type=float, default=14.0)
    ap.add_argument('--tcp', default='-0.007000,0.000000,0.002373',
                    help='TCP 在 gripper_frame_link 下的 xyz（米）')
    ap.add_argument('--out', default='/tmp/tcp_view.png')
    ap.add_argument('--zoom-mm', type=float, default=16.0,
                    help='放大视图的半宽（mm）')
    args = ap.parse_args()

    m = Model(args.urdf)
    R_f, t_f = m.tf('gripper_frame_joint')
    R_j, t_j = m.tf('gripper', args.angle)

    fixed = to_frame(m.tris('gripper_link'), R_f, t_f)
    jaw = to_frame(np.einsum('ij,nkj->nki', R_j, m.tris('moving_jaw_so101_v1_link'))
                   + t_j, R_f, t_f)

    tcp = np.array([float(v) for v in args.tcp.split(',')])
    tip = fixed.reshape(-1, 3)[np.argmax(fixed.reshape(-1, 3)[:, 2])]

    allpts = np.concatenate([fixed.reshape(-1, 3), jaw.reshape(-1, 3)])
    print(f'固定爪 {len(fixed)} 面, 活动爪 {len(jaw)} 面')
    print(f'TCP(相对 gripper_frame_link) = {np.round(tcp*1000,3).tolist()} mm')
    print(f'几何 z 范围 {allpts[:,2].min()*1000:+.1f} ~ {allpts[:,2].max()*1000:+.1f} mm')

    try:
        font_path = next(p for p in FONT_CANDIDATES if os.path.exists(p))
        font = ImageFont.truetype(font_path, 20)
        font_s = ImageFont.truetype(font_path, 15)
    except Exception:  # noqa: BLE001
        font = ImageFont.load_default()
        font_s = font

    W, H = 1820, 660
    img = Image.new('RGB', (W, H), (247, 249, 251))
    draw = ImageDraw.Draw(img)

    colors = np.concatenate([
        np.tile(np.array([196, 200, 206]), (len(fixed), 1)),   # 固定爪：灰
        np.tile(np.array([235, 170, 60]), (len(jaw), 1)),      # 活动爪：琥珀
    ])
    tris = np.concatenate([fixed, jaw], axis=0)
    markers = [
        (tcp, (220, 40, 40), 'TCP = grasp point', 9, 18),
        (np.zeros(3), (40, 90, 220), 'gripper_frame_link origin', 7, 38),
        (tip, (30, 150, 70), 'jaw tip', 6, -26),
    ]

    draw_panel(draw, tris, colors, VIEWS['side'], (24, 44, 470, 620),
               'Whole gripper (side)', markers, font, font_s)

    # 放大到爪口：让固定爪内侧面 + 开口都进画面
    zoom = args.zoom_mm / 1000.0
    bounds = (tcp[0] - 0.65 * zoom, tcp[0] + 1.25 * zoom,
              tcp[2] - 0.85 * zoom, tcp[2] + 1.15 * zoom)
    project = draw_panel(draw, tris, colors, VIEWS['side'], (490, 44, 1260, 620),
                         f'ZOOM on the grasp point  (±{args.zoom_mm:.0f} mm)',
                         markers, font, font_s, bounds=bounds)

    # 在 TCP 处画一个目标物块（边长 object-mm）—— 一眼看出这个点是干嘛的
    half = args.object_mm / 2000.0
    corners = [(tcp[0] - half, tcp[2] - half), (tcp[0] + half, tcp[2] - half),
               (tcp[0] + half, tcp[2] + half), (tcp[0] - half, tcp[2] + half)]
    poly = [project(np.array([c[0], 0.0, c[1]])) for c in corners]
    draw.polygon(poly, outline=(220, 40, 40))
    for i in range(4):
        draw.line([poly[i], poly[(i + 1) % 4]], fill=(220, 40, 40), width=2)
    draw.text((poly[0][0] - 4, poly[3][1] - 20),
              f'{args.object_mm:.0f} mm object', fill=(220, 40, 40), font=font_s)

    # 放大视图里画尺寸标注：TCP 相对 gripper_frame_link 原点的偏移
    ox, oy = project(np.zeros(3))
    tx, ty = project(tcp)
    draw.line([ox, oy, ox, ty], fill=(40, 90, 220), width=2)
    draw.line([ox, ty, tx, ty], fill=(220, 40, 40), width=2)
    draw.text(((ox + tx) / 2 - 22, ty - 22), f'{-tcp[0]*1000:.1f} mm',
              fill=(220, 40, 40), font=font_s)
    draw.text((ox + 6, (oy + ty) / 2), f'{tcp[2]*1000:.1f} mm',
              fill=(40, 90, 220), font=font_s)

    draw_panel(draw, tris, colors, make_view_iso(), (1280, 44, 1796, 620),
               '3/4 view', markers, font, font_s)

    draw.text((24, 12), 'SO-101 gripper — TCP from mesh  '
                        '(grey = fixed jaw / amber = moving jaw).  '
                        'TCP = jaw-opening centre for a 14 mm object',
              fill=(30, 40, 50), font=font)
    img.save(args.out)
    print(f'📄 {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
