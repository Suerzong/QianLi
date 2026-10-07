#!/usr/bin/env python3
"""决定性验证：
1. 用 marks 存的完整关节角，离线重算每点"固定爪顶端"（模型自洽检查）
2. 三点拟合：(0,0),(9.9,0),(0,6.6) —— 若 RMS 小，问题只在右下点；
   若 RMS 大，左上也不准。
3. 残差方向分解：每点残差的 (x,y) 分量，看系统性模式。
"""

from project_paths import calibration_path, project_path
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.expanduser(project_path('qianli_ws/src/qianli_vision/scripts')))
from gripper_model import GripperModel, JOINTS

marks = json.load(open(calibration_path('extrinsic_marks_merged.json')))
model = GripperModel(stride=14)

print('=== 1) 模型自洽检查: 用 joints 重算固定爪顶端 vs 记录的 contact_m ===')
for m in marks:
    j = {k: float(m['joints'][k]) for k in JOINTS}
    T = model.solve(j)
    M = T['gripper_link']
    pts = model.parts['gripper_link']
    world = (M[:3, :3] @ pts.T).T + M[:3, 3]
    k = int(np.argmin(world[:, 2]))
    rec = np.array(m['contact_m'])
    d = np.linalg.norm(world[k] - rec) * 1000
    print(f'  grid {m["grid_cm"]!s:>10}: 重算 vs 记录差 {d:.3f} mm')

print()
print('=== 2) 三点拟合 (0,0),(9.9,0),(0,6.6) ===')


def solve_planar(grid_xy, base_xy):
    G = np.asarray(grid_xy, float)
    B = np.asarray(base_xy, float)
    gc, bc = G.mean(axis=0), B.mean(axis=0)
    Gd, Bd = G - gc, B - bc
    num = float(np.sum(Gd[:, 0] * Bd[:, 1] - Gd[:, 1] * Bd[:, 0]))
    den = float(np.sum(Gd[:, 0] * Bd[:, 0] + Gd[:, 1] * Bd[:, 1]))
    theta = math.atan2(num, den)
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, -s], [s, c]])
    t = bc - R @ gc
    pred = (R @ G.T).T + t
    return theta, t, pred - B


sel = [m for m in marks if tuple(m['grid_cm']) in [(0.0, 0.0), (9.9, 0.0), (0.0, 6.6)]]
grid = [np.array(m['grid_cm']) / 100.0 for m in sel]
base = [np.array(m['contact_m'][:2]) for m in sel]
th, t, res = solve_planar(grid, base)
print(f'  三点拟合: θ={math.degrees(th):+.2f}°  RMS='
      f'{np.sqrt(np.mean(np.linalg.norm(res,axis=1)**2))*1000:.2f} mm')
for m, r in zip(sel, res):
    print(f'    grid {m["grid_cm"]!s:>10}: 残差 ({r[0]*1000:+7.2f}, '
          f'{r[1]*1000:+7.2f}) mm')

print()
print('=== 3) 五点拟合残差方向分解 ===')
grid = [np.array(m['grid_cm']) / 100.0 for m in marks]
base = [np.array(m['contact_m'][:2]) for m in marks]
th5, t5, res5 = solve_planar(grid, base)
print(f'  五点拟合: θ={math.degrees(th5):+.2f}°')
for m, r in zip(marks, res5):
    print(f'    grid {m["grid_cm"]!s:>10}: 残差 ({r[0]*1000:+7.2f}, '
          f'{r[1]*1000:+7.2f}) mm   |r|={np.linalg.norm(r)*1000:5.2f}')
