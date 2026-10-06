#!/usr/bin/env python3
"""外参标定（多点法）：grid → base_link，替换旧的"两点法"

旧两点法的问题（见 docs/GRASP_REAL_AUDIT.md）
-------------------------------------------
* 只用 2 个点：只能定 θ，没有冗余、没有残差、无法发现摆错；
* 记录的是 ``gripper_frame_link`` 的位置，而人是按**爪口**去对格点的，
  两者差几十毫米 → 系统性误差；
* 实测 A→B 距离 34.6mm vs 格宽 33mm，两个内角点 Z 还差 9.5mm。

本工具
------
1. 用**已标定的 TCP**（/tmp/tcp_calib.txt）而不是 gripper_frame_link；
2. 按提示依次触碰 **N 个已知格点**（默认 5 个，L 形铺开）；
3. 闭式最小二乘（Umeyama，固定比例）解 θ 与平移；
4. 输出**残差**、**反推的格宽**——这两个数直接告诉你标定可不可信。

不需要使能扭矩：松扭矩后用手把爪口摆到格点上即可，零运动风险。

用法::

    # 先跑 tcp_calibrate.py 得到 /tmp/tcp_calib.txt
    ~/mj/bin/python extrinsic_calib_multi.py --cell-cm 3.3
    # 按提示摆好每个格点后： touch /tmp/grid_mark
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np

TRIGGER = '/tmp/grid_mark'
TCP_CALIB = '/tmp/tcp_calib.txt'
OUT = '/tmp/extrinsic.txt'
OUT_JSON = '/tmp/extrinsic_marks.json'
DEFAULT_POINTS_CM = [(0.0, 0.0), (9.9, 0.0), (0.0, 6.6), (9.9, 6.6), (3.3, 3.3)]


def read_kv(path):
    values = {}
    try:
        for line in open(path):
            line = line.strip()
            if '=' in line and not line.startswith('#'):
                key, value = line.split('=', 1)
                try:
                    values[key.strip()] = float(value.strip())
                except ValueError:
                    pass
    except OSError:
        pass
    return values


def solve_planar(grid_xy, base_xy):
    """闭式解：base = Rz(θ)·grid + t（比例固定为 1）。

    返回 (theta_rad, t(2,), residuals_m)
    """
    G = np.asarray(grid_xy, dtype=float)
    B = np.asarray(base_xy, dtype=float)
    gc, bc = G.mean(axis=0), B.mean(axis=0)
    Gd, Bd = G - gc, B - bc
    # θ 使 R(θ)Gd 最贴近 Bd
    num = float(np.sum(Gd[:, 0] * Bd[:, 1] - Gd[:, 1] * Bd[:, 0]))
    den = float(np.sum(Gd[:, 0] * Bd[:, 0] + Gd[:, 1] * Bd[:, 1]))
    theta = math.atan2(num, den)
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, -s], [s, c]])
    t = bc - R @ gc
    predicted = (R @ G.T).T + t
    residuals = np.linalg.norm(predicted - B, axis=1)
    return theta, t, residuals, predicted


def implied_cell_size(grid_xy, base_xy, pairs):
    """用 base 系下的实测距离反推格宽，独立校验比例是否正确。"""
    out = []
    for i, j in pairs:
        g = math.dist(grid_xy[i], grid_xy[j])
        b = math.dist(base_xy[i], base_xy[j])
        if g > 1e-9:
            out.append(b / g)
    return out


def collect(args, points):
    import rclpy
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener

    calib = read_kv(args.tcp)
    offset = np.array([calib.get('offset_x', 0.0), calib.get('offset_y', 0.0),
                       calib.get('offset_z', 0.0)])
    has_tcp = bool(calib)
    flange = calib.get('flange', 'gripper_link') if has_tcp else 'gripper_frame_link'

    class Collector(Node):
        def __init__(self):
            super().__init__('extrinsic_calib_multi')
            self.buf = Buffer()
            self.listener = TransformListener(self.buf, self)
            self.marks = []

        def tcp_base(self):
            """爪口(已标定 TCP) 在 base_link 下的位置。"""
            try:
                tr = self.buf.lookup_transform('base_link', flange,
                                               rclpy.time.Time())
            except Exception:
                return None
            t = tr.transform.translation
            q = tr.transform.rotation
            n = math.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2)
            x, y, z, w = q.x / n, q.y / n, q.z / n, q.w / n
            R = np.array([
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
            p = np.array([t.x, t.y, t.z])
            if has_tcp:
                p = p + R @ offset
            return p

    rclpy.init()
    node = Collector()
    if not has_tcp:
        print(f'⚠️ 没找到 {args.tcp}，退回用 gripper_frame_link —— '
              f'这会引入系统性偏移，建议先跑 tcp_calibrate.py')
    print(f'使用参考点：{flange}' +
          (f' + TCP 偏移 ({offset[0]*1000:+.1f}, {offset[1]*1000:+.1f}, '
           f'{offset[2]*1000:+.1f}) mm' if has_tcp else ''))
    print()

    for index, (gx, gy) in enumerate(points, 1):
        prompt = (f'▶ 第 {index}/{len(points)} 点：把爪口对准 grid '
                  f'({gx:.1f}, {gy:.1f}) cm')
        print(prompt)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if os.path.exists(TRIGGER):
                try:
                    os.remove(TRIGGER)
                except OSError:
                    pass
                p = node.tcp_base()
                if p is None:
                    print('   ⚠️ 读不到 TF，本次忽略，请重试')
                    continue
                node.marks.append({'grid_cm': [gx, gy],
                                   'base_m': [float(p[0]), float(p[1]), float(p[2])],
                                   'at': time.strftime('%H:%M:%S')})
                print(f'   ✅ 记录 base_link = ({p[0]:.4f}, {p[1]:.4f}, '
                      f'{p[2]:.4f})')
                break
    marks = node.marks
    node.destroy_node()
    if rclpy.ok():
        rclpy.try_shutdown()
    with open(args.json, 'w') as fh:
        json.dump(marks, fh, indent=2)
    return marks


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cell-cm', type=float, default=3.3)
    ap.add_argument('--tcp', default=TCP_CALIB)
    ap.add_argument('--json', default=OUT_JSON)
    ap.add_argument('--out', default=OUT)
    ap.add_argument('--solve-only')
    args = ap.parse_args()

    if args.solve_only:
        with open(args.solve_only) as fh:
            marks = json.load(fh)
    else:
        marks = collect(args, DEFAULT_POINTS_CM)

    if len(marks) < 2:
        print(f'❌ 至少需要 2 个点（现在 {len(marks)}）')
        return 1

    grid_m = [np.array(m['grid_cm']) / 100.0 for m in marks]
    base_xy = [np.array(m['base_m'][:2]) for m in marks]
    base_z = [m['base_m'][2] for m in marks]

    theta, t, residuals, predicted = solve_planar(grid_m, base_xy)
    theta_deg = math.degrees(theta)

    # 反推格宽：用相邻点距离比
    pairs = [(i, i + 1) for i in range(len(marks) - 1)]
    scales = implied_cell_size(grid_m, base_xy, pairs)
    implied = [s * args.cell_cm for s in scales]

    print()
    print('=' * 74)
    print('  外参标定结果（grid → base_link）')
    print('=' * 74)
    print(f'  打点数            {len(marks)}')
    print(f'  旋转 θ            {theta_deg:+.3f}°')
    print(f'  平移              ({t[0]:+.4f}, {t[1]:+.4f}) m')
    print(f'  桌面高度 z        {np.mean(base_z):+.4f} m   '
          f'(各点极差 {((max(base_z)-min(base_z))*1000):.1f} mm)')
    print()
    print(f'  残差 RMS          {float(np.sqrt(np.mean(residuals**2)))*1000:.2f} mm'
          f'   最大 {float(np.max(residuals))*1000:.2f} mm')
    print(f'  单点残差(mm)      {np.round(residuals*1000, 2).tolist()}')
    if implied:
        print(f'  反推格宽          {np.round(implied, 2).tolist()} mm '
              f'(标称 {args.cell_cm*10:.1f} mm)')
        deviation = abs(np.mean(implied) - args.cell_cm * 10)
        print(f'  格宽偏差          {deviation:.2f} mm '
              f'({"✅ 比例一致" if deviation < 1.0 else "⚠️ 比例不符，检查标称格宽或 TCP"})')
    rms = float(np.sqrt(np.mean(residuals ** 2))) * 1000
    zspread = (max(base_z) - min(base_z)) * 1000
    ok = rms < 3.0 and zspread < 8.0
    print()
    print(f'  质量              {"✅ 可信" if ok else "⚠️ 残差或 Z 落差偏大，重摆打点"}'
          f'   (阈值 RMS<3mm, Z极差<8mm)')

    lines = [
        '# 外参（grid → base_link）：多点最小二乘实测',
        f'# 打点数 {len(marks)}   残差 RMS {rms:.3f} mm   最大 '
        f'{float(np.max(residuals))*1000:.3f} mm',
        f'# 桌面各点 z 极差 {zspread:.2f} mm',
        f'# 反推格宽 {np.round(implied,3).tolist() if implied else "n/a"} mm '
        f'(标称 {args.cell_cm*10:.1f} mm)',
        f'grid_origin_x={t[0]:.4f}',
        f'grid_origin_y={t[1]:.4f}',
        f'grid_origin_z={np.mean(base_z):.4f}',
        f'grid_theta_deg={theta_deg:.2f}',
        f'cell_cm={args.cell_cm}',
    ]
    with open(args.out, 'w') as fh:
        fh.write('\n'.join(lines) + '\n')
    print(f'\n📄 结果已写入 {args.out}')
    return 0 if ok else 2


if __name__ == '__main__':
    sys.exit(main())
