#!/usr/bin/env python3
"""用实测方向+真实格边长重建正交映射，只拟合原点，看旧标定点残差。"""
import json
import math

import numpy as np

# 拖拽实测
DR = json.load(open('/tmp/drag_line.json'))
d = np.array(DR['direction_xy'])
LEN = 187.0
for n in (5, 6, 7):
    print(f'  直段 {LEN:.1f}mm / {n} 格 = {LEN/n:.2f} mm/格')
CELL = LEN / 6.0
ex = d / np.linalg.norm(d)
print(f'\n实测方向 ex = ({ex[0]:+.5f},{ex[1]:+.5f})  '
      f'{math.degrees(math.atan2(ex[1], ex[0])):+.2f}°')
print(f'格边长 CELL = {CELL:.3f} mm（取 6 格）')

# 旧标定 5 点（标签是"33mm 格"的单位）
MARKS = [((0.0, 0.0), (0.30270909465110973, 0.03176689026538958)),
         ((9.9, 0.0), (0.3047753911280389, -0.0615763312629367)),
         ((0.0, 6.6), (0.2189, 0.0198)),
         ((9.9, 6.6), (0.2309, -0.0696)),
         ((3.3, 3.3), (0.2582, -0.0086))]

# 两种 ey 取法（垂直方向，符号按仿射的 y 列定）
aff = np.array([[0.07298559870136852, -1.1941346564477962],
                [-0.9239523684165104, -0.15094334179736751]])
ay = aff[:, 1] / np.linalg.norm(aff[:, 1])
ey = np.array([-ex[1], ex[0]])
if np.dot(ey, ay) < 0:
    ey = -ey
print(f'正交 ey = ({ey[0]:+.5f},{ey[1]:+.5f})  '
      f'（与仿射 y 列同向，夹角 '
      f'{math.degrees(math.acos(abs(np.dot(ey,ay)))):.2f}°）')


def fit_origin(cell_y_m):
    """给定 y 向格边长(米)，用 5 点最小二乘求原点(米)。"""
    G = []
    B = []
    cell_x_m = CELL / 1000.0
    for (gx, gy), b in MARKS:
        cx, cy = gx / 3.3, gy / 3.3          # 换算成"格数"
        G.append(cx * cell_x_m * ex + cy * cell_y_m * ey)
        B.append(b)
    G, B = np.array(G), np.array(B)
    origin = (B - G).mean(axis=0)
    res = (G + origin) - B
    rms = float(np.sqrt(np.mean(np.sum(res ** 2, axis=1)))) * 1000
    return origin, res, rms


print('\n=== 假设格子正方形（y 向也是 %.2fmm）===' % CELL)
o, res, rms = fit_origin(CELL / 1000.0)
print(f'  原点 ({o[0]:.4f},{o[1]:.4f})  残差 RMS {rms:.2f} mm')
for (g, b), r in zip(MARKS, res):
    print(f'    grid {g!s:>12}  残差 ({r[0]*1000:+7.2f},{r[1]*1000:+7.2f}) mm')

print('\n=== 若 y 向格边长也当未知量扫描 ===')
for cyl in (31.16, 33.0, 35.0, 37.0, 39.0, 42.0):
    _, _, rms = fit_origin(cyl / 1000.0)
    print(f'  y 格 {cyl:5.2f}mm -> 残差 RMS {rms:6.2f} mm')
best = None
for cyl in np.arange(25.0, 50.0, 0.1):
    _, _, rms = fit_origin(cyl / 1000.0)
    if best is None or rms < best[0]:
        best = (rms, cyl)
print(f'\n最佳: y 格 {best[1]:.2f}mm -> RMS {best[0]:.2f} mm  '
      f'(x 格 {CELL:.2f}mm)')
o, res, rms = fit_origin(best[1] / 1000.0)
print(f'  原点 ({o[0]:.4f},{o[1]:.4f})')
for (g, b), r in zip(MARKS, res):
    print(f'    grid {g!s:>12}  残差 ({r[0]*1000:+7.2f},{r[1]*1000:+7.2f}) mm')
