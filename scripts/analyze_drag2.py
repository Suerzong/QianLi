#!/usr/bin/env python3
"""分析拖拽轨迹（可指定文件）：最长直段的方向与格边长。"""
import json
import math
import sys

import numpy as np

path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/drag_line.json'
cells = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
d = json.load(open(path))
P = np.array(d['samples'])
XY = P[:, :2]
print(f'{path}: {len(P)} 样本')
print(f'XY 范围 x {XY[:,0].min():.4f}..{XY[:,0].max():.4f}  '
      f'y {XY[:,1].min():.4f}..{XY[:,1].max():.4f}')
best = None
N = len(XY)
for i in range(0, N, 4):
    for j in range(min(N, i + 40), N, 4):
        Q = XY[i:j]
        c = Q.mean(0)
        U, S, Vt = np.linalg.svd(Q - c, full_matrices=False)
        t = (Q - c) @ Vt[0]
        perp = float(np.sqrt(np.mean(((Q - c) @ Vt[1]) ** 2)) * 1000)
        length = float((t.max() - t.min()) * 1000)
        if perp > 2.5:
            continue
        if best is None or length > best[0]:
            best = (length, perp, Vt[0].copy(), i, j, Q[0].copy(), Q[-1].copy())
if best is None:
    print('没找到直段')
    raise SystemExit(0)
length, perp, dirv, i, j, p0, p1 = best
print(f'最长直段: 样本 {i}..{j}  长度 {length:.1f} mm  垂直残差 {perp:.2f} mm')
print(f'  方向 = ({dirv[0]:+.5f},{dirv[1]:+.5f})  '
      f'{math.degrees(math.atan2(dirv[1], dirv[0])):+.2f}°')
print(f'  起点 ({p0[0]:.4f},{p0[1]:.4f})  终点 ({p1[0]:.4f},{p1[1]:.4f})')
for n in (3, 6):
    print(f'  若 {n} 格 -> 格边长 {length/n:.2f} mm')
