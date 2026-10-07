#!/usr/bin/env python3
"""分析 5 点标定的四边形几何：哪个角点摆歪了、扭曲多大。"""
import json

import numpy as np

marks = json.load(open('/tmp/extrinsic_marks.json'))
pts = {tuple(m['grid_cm']): np.array(m['contact_m'][:2]) for m in marks}

order = [(0.0, 0.0), (9.9, 0.0), (9.9, 6.6), (0.0, 6.6)]  # 顺时针四角
print('四角几何 (grid cm -> base m):')
for g in order:
    p = pts.get(g)
    print(f'  grid {g!s:>12} -> ({p[0]:.4f}, {p[1]:.4f})')

print()
print('边长 vs 标称 (3.3cm 格, 应为 99/66/99/66 mm):')
edges = [(order[i], order[(i + 1) % 4]) for i in range(4)]
names = ['上边 (0,0)→(9.9,0)', '右边 (9.9,0)→(9.9,6.6)',
         '下边 (9.9,6.6)→(0,6.6)', '左边 (0,6.6)→(0,0)']
nominal = [99.0, 66.0, 99.0, 66.0]
for (g1, g2), name, nom in zip(edges, names, nominal):
    d = np.linalg.norm(pts[g1] - pts[g2]) * 1000
    print(f'  {name:>24}: {d:6.1f} mm  (标称 {nom:.0f}, 偏差 {d-nom:+.1f})')

print()
print('对角线 (应 119.0 mm):')
d1 = np.linalg.norm(pts[(0.0, 0.0)] - pts[(9.9, 6.6)]) * 1000
d2 = np.linalg.norm(pts[(9.9, 0.0)] - pts[(0.0, 6.6)]) * 1000
print(f'  (0,0)→(9.9,6.6): {d1:.1f} mm    (9.9,0)→(0,6.6): {d2:.1f} mm')

print()
print('中心点 (3.3,3.3) 与四角矩形中心的偏差 (应为 ~0):')
rect_c = np.mean([pts[g] for g in order], axis=0)
c = pts[(3.3, 3.3)]
print(f'  四角中心 = ({rect_c[0]:.4f}, {rect_c[1]:.4f})')
print(f'  中心点    = ({c[0]:.4f}, {c[1]:.4f})')
print(f'  偏差 = {np.linalg.norm(rect_c - c)*1000:.1f} mm')

print()
print('理想矩形拟合: 用四角点与已知格宽反推每个点的位置误差:')
# 以 (0,0) 为原点, 按实测边向量建坐标系, 看 9.9/6.6 交点应在哪
p00 = pts[(0.0, 0.0)]
ex = pts[(9.9, 0.0)] - p00       # x 方向实测边
ey = pts[(0.0, 6.6)] - p00       # y 方向实测边
pred = {g: p00 + ex * (g[0] / 9.9) + ey * (g[1] / 6.6) for g in order}
for g in order:
    err = np.linalg.norm(pts[g] - pred[g]) * 1000
    print(f'  grid {g!s:>12}: 预测 vs 实测差 {err:5.1f} mm')
