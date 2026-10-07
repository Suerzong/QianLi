#!/usr/bin/env python3
"""用正确的 4 点 (0,0),(9.9,0),(0,6.6),(3.3,3.3) 重新拟合外参。
去掉摆歪的 (9.9,6.6)。输出棋盘位姿 + 残差 + 格宽验证。"""
import json
import math

import numpy as np

marks = json.load(open('/tmp/extrinsic_marks_merged.json'))
DROP = (9.9, 6.6)
sel = [m for m in marks if tuple(m['grid_cm']) != DROP]
print(f'使用 {len(sel)} 点（剔除 {DROP}）: '
      f'{sorted(tuple(m["grid_cm"]) for m in sel)}')

G = np.array([m['grid_cm'] for m in sel], float) / 100.0
B = np.array([m['contact_m'][:2] for m in sel], float)
gc, bc = G.mean(axis=0), B.mean(axis=0)
Gd, Bd = G - gc, B - bc
num = float(np.sum(Gd[:, 0] * Bd[:, 1] - Gd[:, 1] * Bd[:, 0]))
den = float(np.sum(Gd[:, 0] * Bd[:, 0] + Gd[:, 1] * Bd[:, 1]))
th = math.atan2(num, den)
c, s = math.cos(th), math.sin(th)
R = np.array([[c, -s], [s, c]])
t = bc - R @ gc
pred = (R @ G.T).T + t
res = pred - B
rms = np.sqrt(np.mean(np.linalg.norm(res, axis=1) ** 2)) * 1000

print(f'θ = {math.degrees(th):+.3f}°  origin = ({t[0]:.4f}, {t[1]:.4f})')
print(f'RMS = {rms:.2f} mm')
for m, r in zip(sel, res):
    print(f'  grid {m["grid_cm"]!s:>10}: 残差 ({r[0]*1000:+6.2f}, '
          f'{r[1]*1000:+6.2f}) mm')

# 反推格宽（用相邻格点距离验证尺度）
print()
print('=== 4 点几何自洽 ===')
ex = B[1] - B[0]  # (0,0)->(9.9,0)
ey = B[2] - B[0]  # (0,0)->(0,6.6)
print(f'上边长度 {np.linalg.norm(ex)*1000:.1f} mm (应 99)')
print(f'左边长度 {np.linalg.norm(ey)*1000:.1f} mm (应 66)')
print(f'中心点(3.3,3.3)应位 vs 实测:')
center_expect = B[0] + ex / 3 + ey / 3
center_actual = [m['contact_m'][:2] for m in marks
                 if tuple(m['grid_cm']) == (3.3, 3.3)][0]
d = np.linalg.norm(np.array(center_actual) - center_expect) * 1000
print(f'  预测 ({center_expect[0]:.4f},{center_expect[1]:.4f}) '
      f'实测 ({center_actual[0]:.4f},{center_actual[1]:.4f})  差 {d:.1f} mm')
