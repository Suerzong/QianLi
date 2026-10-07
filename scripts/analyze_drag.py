#!/usr/bin/env python3
"""在拖拽轨迹里找最长直段，量出方向与格边长。"""
import json
import math

import numpy as np

d = json.load(open('/tmp/drag_line.json'))
P = np.array(d['samples'])
XY = P[:, :2]
seg = np.linalg.norm(np.diff(XY, axis=0), axis=1) * 1000
print(f'样本 {len(P)}  轨迹总长 {seg.sum():.0f} mm   '
      f'单步中位 {np.median(seg):.2f} mm')
print(f'XY 范围 x {XY[:,0].min():.4f}..{XY[:,0].max():.4f}  '
      f'y {XY[:,1].min():.4f}..{XY[:,1].max():.4f}')

# 滑动窗口找最长的"直且长"段
best = None
N = len(XY)
for i in range(0, N, 5):
    for j in range(min(N, i + 60), N, 5):
        Q = XY[i:j]
        c = Q.mean(0)
        U, S, Vt = np.linalg.svd(Q - c, full_matrices=False)
        t = (Q - c) @ Vt[0]
        perp = np.sqrt(np.mean(((Q - c) @ Vt[1]) ** 2)) * 1000
        length = (t.max() - t.min()) * 1000
        if perp > 2.5:
            continue
        score = length
        if best is None or score > best[0]:
            best = (score, length, perp, Vt[0].copy(), i, j,
                    Q[0].copy(), Q[-1].copy())
if best is None:
    print('没找到足够直的段')
    raise SystemExit(0)
score, length, perp, dirv, i, j, p0, p1 = best
print(f'\n最长直段: 样本 {i}..{j}  长度 {length:.1f} mm   '
      f'垂直残差 {perp:.2f} mm')
print(f'  方向(base XY 单位向量) = ({dirv[0]:+.5f}, {dirv[1]:+.5f})  '
      f'= {math.degrees(math.atan2(dirv[1], dirv[0])):+.2f}°')
print(f'  起点 ({p0[0]:.4f},{p0[1]:.4f})  终点 ({p1[0]:.4f},{p1[1]:.4f})')
for n in (5, 6, 7):
    print(f'  若这段走了 {n} 格 -> 格边长 {length/n:.2f} mm')
