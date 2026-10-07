#!/usr/bin/env python3
"""从 /tmp/blocks_located.json 挑最像方块的目标，并用棋盘单应实测其边长。

输出: <x> <y> # px=... span=... size_mm=.. grid=.. on_board=.. radius=..
"""
import argparse
import json
import os

import numpy as np

CFG = os.path.expanduser('~/QianLi/qianli_ws/config')

ap = argparse.ArgumentParser()
ap.add_argument('--color', default='yellow')
ap.add_argument('--half-mm', type=float, default=20.0)
ap.add_argument('--clearance-mm', type=float, default=6.0)
a = ap.parse_args()

fr = json.load(open(os.path.join(CFG, 'board_frame.json')))
H = np.array(fr['H'])          # H: 像素 -> 棋盘 mm


def size_mm(px, span_px):
    """用棋盘单应在该像素处的局部尺度，把像素边长换算成真实 mm。"""
    def g_of(p):
        v = H @ np.array([p[0], p[1], 1.0])
        return v[:2] / v[2] / 10.0            # 棋盘 cm
    c = np.array(px, float)
    sx = np.linalg.norm(g_of(c + [5, 0]) - g_of(c - [5, 0])) / 10.0
    sy = np.linalg.norm(g_of(c + [0, 5]) - g_of(c - [0, 5])) / 10.0
    return span_px * (sx + sy) / 2 * 10.0     # mm


blocks = json.load(open('/tmp/blocks_located.json'))
cand = []
for b in blocks:
    if b['color'] != a.color:
        continue
    if not 20 <= b['span_px'] <= 80:
        continue
    sz = size_mm(b['px'], b['span_px'])
    if not 15 <= sz <= 60:                    # 方块量级（排除臂上黄件/工具）
        continue
    score = abs(sz - 33) + (0 if b['on_board'] else 60)
    cand.append((score, b, sz))
if not cand:
    print('NONE')
    raise SystemExit(1)
cand.sort(key=lambda t: t[0])
_, b, sz = cand[0]
print(f"{b['base_m'][0]:.4f} {b['base_m'][1]:.4f} "
      f"# px=({b['px'][0]:.0f},{b['px'][1]:.0f}) span={b['span_px']}px "
      f"size_mm={sz:.1f} grid=({b['grid_cm'][0]:.2f},{b['grid_cm'][1]:.2f})cm "
      f"on_board={b['on_board']} radius={b['radius_mm']:.0f}mm")
