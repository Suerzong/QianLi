#!/usr/bin/env python3
"""整机场景渲染（PIL 软件渲染，无需 OpenGL）

为什么不用 MuJoCo 渲染
----------------------
虚拟机上 libOSMesa 缺失、EGL 也建不起上下文，`mujoco.Renderer` 在 SSH 里
直接 FatalError。而 PIL 是软件渲染，无头也能跑，且足够看清几何关系。

画什么
------
整条机械臂（URDF 各 link 的网格）+ 实测桌面 + 垫台 + 夹爪最低点标记，
用来肉眼确认"桌面到底在哪、夹爪离桌面多远"。

用法::

    ~/mj/bin/python render_scene.py --table-z -0.06909 --q 0 1.2 -0.9 1.3 1.7 -0.13
    ~/mj/bin/python render_scene.py --from-ros --out /tmp/scene.png
"""

from __future__ import annotations

import argparse
import math
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from gripper_model import GripperModel, JOINTS, JAW_LINK

FONT_CANDIDATES = [
    '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
    '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
]
ARM_LINKS = ['base_link', 'shoulder_link', 'upper_arm_link', 'lower_arm_link',
             'wrist_link', 'gripper_link', JAW_LINK]
ARM_COLORS = {
    'base_link': (170, 175, 182),
    'shoulder_link': (238, 192, 60),
    'upper_arm_link': (238, 192, 60),
    'lower_arm_link': (238, 192, 60),
    'wrist_link': (238, 192, 60),
    'gripper_link': (196, 200, 206),
    JAW_LINK: (235, 170, 60),
}


def box_tris(center, size):
    cx, cy, cz = center
    sx, sy, sz = size
    v = np.array([[cx + a * sx / 2, cy + b * sy / 2, cz + c * sz / 2]
                  for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)])
    faces = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1),
             (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    return np.array([[v[a], v[b], v[c]] for a, b, c in faces])


def draw_scene(draw, tris, colors, view_R, box, markers, font, font_s,
               bounds=None, light=np.array([0.35, -0.5, 0.79])):
    x0, y0, x1, y1 = box
    pts = np.einsum('ij,nkj->nki', view_R, tris)
    if bounds is None:
        flat = pts.reshape(-1, 3)
        lo, hi = flat.min(axis=0), flat.max(axis=0)
        margin = 0.05 * max(hi[0] - lo[0], hi[1] - lo[1])
        lo, hi = lo - margin, hi + margin
    else:
        lo = np.array([bounds[0], bounds[2]], float)
        hi = np.array([bounds[1], bounds[3]], float)
    scale = min((x1 - x0) / (hi[0] - lo[0]), (y1 - y0) / (hi[1] - lo[1]))
    cx, cy = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2
    px, py = (x0 + x1) / 2, (y0 + y1) / 2

    def project(p):
        return (px + (p[0] - cx) * scale, py - (p[1] - cy) * scale)

    order = np.argsort(pts[:, :, 2].mean(axis=1))
    ln = light / np.linalg.norm(light)
    for idx in order:
        tri = pts[idx]
        if (tri[:, 0].max() < lo[0] or tri[:, 0].min() > hi[0]
                or tri[:, 1].max() < lo[1] or tri[:, 1].min() > hi[1]):
            continue
        n = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        nn = np.linalg.norm(n)
        if nn < 1e-12:
            continue
        n = n / nn
        if n[2] < 0:
            n = -n
        shade = 0.42 + 0.58 * max(0.0, float(n @ ln))
        col = tuple(int(np.clip(c * shade, 0, 255)) for c in colors[idx])
        draw.polygon([project(tri[0]), project(tri[1]), project(tri[2])], fill=col)

    for p, col, label, r in markers:
        sx, sy = project(view_R @ np.asarray(p, float))
        draw.ellipse([sx - r, sy - r, sx + r, sy + r], fill=col,
                     outline=(250, 250, 250), width=2)
        draw.text((sx + r + 6, sy - r - 4), label, fill=col, font=font_s)

    draw.rectangle(box, outline=(120, 130, 140), width=1)
    sb = 0.05 * scale
    draw.line([x1 - sb - 24, y1 - 24, x1 - 24, y1 - 24], fill=(20, 20, 20), width=3)
    draw.text((x1 - sb - 24, y1 - 44), '50 mm', fill=(20, 20, 20), font=font_s)
    return lambda p: project(view_R @ np.asarray(p, float))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--q', nargs=6, type=float, help='6 个关节角（rad）')
    ap.add_argument('--from-ros', action='store_true', help='从 /joint_states 读')
    ap.add_argument('--table-z', type=float, default=-0.06909)
    ap.add_argument('--pedestal-bottom', type=float, default=-0.0024)
    ap.add_argument('--stride', type=int, default=9)
    ap.add_argument('--out', default='/tmp/scene.png')
    args = ap.parse_args()

    joints = {n: 0.0 for n in JOINTS}
    if args.q:
        for n, v in zip(JOINTS, args.q):
            joints[n] = float(v)
    elif args.from_ros:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        import time
        rclpy.init()
        node = Node('render_scene')
        got = {}
        node.create_subscription(
            JointState, '/joint_states',
            lambda m: got.update(dict(zip(m.name, m.position))), 10)
        t0 = time.time()
        while rclpy.ok() and len(got) < 6 and time.time() - t0 < 8:
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(got) < 6:
            print('❌ 读不到 /joint_states')
            return 1
        joints.update({k: float(got[k]) for k in JOINTS})

    model = GripperModel(stride=args.stride)
    fails = model.selftest()
    if fails:
        print('❌ FK 自检未通过，图不可信：')
        for f in fails:
            print('   ·', f)
        return 1

    T = model.solve(joints)
    tris, colors = [], []
    for link in ARM_LINKS:
        pts = model.link_points(link)
        if not len(pts):
            continue
        M = T.get(link)
        if M is None:
            continue
        world = (M[:3, :3] @ pts.T).T + M[:3, 3]
        # 点云 → 三角面太麻烦，这里把点做成"小三角"当作点云面包：
        # 直接按点画会太稀，改用点云整体当三角化的近似——用相邻点构造面
        # 简化处理：把网格点当作三角形顶点流（STL 已按面存储）
        # link_points 返回的是展平顶点，按每 3 个一组重建面
        n3 = (len(world) // 3) * 3
        tri = world[:n3].reshape(-1, 3, 3)
        tris.append(tri)
        colors.append(np.tile(np.array(ARM_COLORS[link], float), (len(tri), 1)))

    # 桌面 + 垫台
    table_top = args.table_z
    table = box_tris([0.30, 0.0, table_top - 0.15], [0.8, 0.8, 0.30])
    tris.append(table)
    colors.append(np.tile(np.array([150, 152, 158], float), (len(table), 1)))
    ped_h = args.pedestal_bottom - table_top
    ped = box_tris([0.0, 0.0, (table_top + args.pedestal_bottom) / 2],
                   [0.09, 0.10, max(ped_h, 1e-4)])
    tris.append(ped)
    colors.append(np.tile(np.array([92, 92, 98], float), (len(ped), 1)))

    tris = np.concatenate(tris, axis=0)
    colors = np.concatenate(colors, axis=0)

    low, low_link = model.lowest_point(joints)
    tcp = model.tcp(joints)
    print(f'关节角 ' + ', '.join(f'{joints[k]:+.3f}' for k in JOINTS))
    print(f'夹爪最低点 z = {low[2]*1000:+.1f} mm  [{low_link}]')
    print(f'TCP z       = {tcp[2]*1000:+.1f} mm')
    print(f'桌面 z      = {args.table_z*1000:+.1f} mm')
    print(f'→ 最低点到桌面间隙 {(low[2]-args.table_z)*1000:+.1f} mm')

    try:
        fp = next(p for p in FONT_CANDIDATES if __import__('os').path.exists(p))
        font = ImageFont.truetype(fp, 20)
        font_s = ImageFont.truetype(fp, 15)
    except Exception:  # noqa: BLE001
        font = ImageFont.load_default()
        font_s = font

    markers = [
        (low, (220, 40, 40), f'gripper lowest point  z={low[2]*1000:+.1f}mm', 8),
        (tcp, (40, 90, 220), f'TCP  z={tcp[2]*1000:+.1f}mm', 7),
    ]

    W, H = 1500, 720
    img = Image.new('RGB', (W, H), (245, 247, 250))
    draw = ImageDraw.Draw(img)
    draw.text((20, 12), f'SO-101 + table (measured z = {args.table_z*1000:+.1f} mm)'
                        f'  —  grey table / dark pedestal', fill=(30, 40, 50),
              font=font)

    # 侧视：水平方向取 X，垂直取 Z（最能看出高度关系）
    side = np.array([[1.0, 0, 0], [0, 0, 1.0], [0, -1.0, 0]])
    draw.text((30, 44), 'Side view (X right / Z up)', fill=(30, 40, 50), font=font)
    draw_scene(draw, tris, colors, side, (24, 70, 760, 700), markers, font, font_s)

    # 3/4 视
    a, b = math.radians(-40), math.radians(20)
    Rz = np.array([[math.cos(a), -math.sin(a), 0],
                   [math.sin(a), math.cos(a), 0], [0, 0, 1]])
    Rx = np.array([[1, 0, 0], [0, math.cos(b), -math.sin(b)],
                   [0, math.sin(b), math.cos(b)]])
    draw.text((790, 44), '3/4 view', fill=(30, 40, 50), font=font)
    draw_scene(draw, tris, colors, Rx @ Rz, (784, 70, 1476, 700), markers,
               font, font_s)

    # 高度标注
    draw.text((30, 690), f'table {args.table_z*1000:+.1f} mm   '
                         f'pedestal top {args.pedestal_bottom*1000:+.1f} mm   '
                         f'gap {((low[2]-args.table_z)*1000):+.1f} mm',
              fill=(30, 40, 50), font=font_s)
    img.save(args.out)
    print(f'📄 {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
