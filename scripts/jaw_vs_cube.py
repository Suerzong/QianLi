#!/usr/bin/env python3
"""用 mesh 模型算：抓取时固定爪/活动爪 相对方块(40mm立方) 的真实几何。

输出：两个爪指到方块各面的最小间距、是否有穿透、接触面在工具系的位置。
"""

from project_paths import driver_params_path, project_path
import argparse
import json
import math
import os
import sys

import numpy as np
from scipy.optimize import least_squares
import yaml
from pathlib import Path

sys.path.insert(0, os.path.expanduser(
    project_path('qianli_ws/src/qianli_vision/scripts')))
from gripper_model import GripperModel, JOINTS, FLANGE_LINK

CONFIG = os.path.expanduser(
    driver_params_path())
CFG = os.path.expanduser(project_path('config'))
TABLE_Z = -0.06485
DOWN = np.array([0.0, 0.0, -1.0])

ap = argparse.ArgumentParser()
ap.add_argument('--x', type=float, default=0.2426)
ap.add_argument('--y', type=float, default=-0.0575)
ap.add_argument('--cube-mm', type=float, default=40.0)
ap.add_argument('--offset-mm', type=float, default=-10.0)
ap.add_argument('--gap-mm', type=float, default=0.5)
ap.add_argument('--depth-mm', type=float, default=-48.0)
a = ap.parse_args()

cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
lo = (np.array(cfg['raw_min']) - zero) * direction * 2 * math.pi / 4096
hi = (np.array(cfg['raw_max']) - zero) * direction * 2 * math.pi / 4096
lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)

fr = json.load(open(os.path.join(CFG, 'board_frame.json')))
A = np.array(fr['affine'])
ey = A[:2, 1] / np.linalg.norm(A[:2, 1])
xax_w = np.array([ey[0], ey[1], 0.0])
yax_w = np.cross(DOWN, xax_w)
R_des = np.column_stack([xax_w, yax_w, DOWN])

half = a.cube_mm / 2000.0
cube = np.array([a.x, a.y, TABLE_Z + half])          # 方块中心(世界)
tcp_xy = np.array([a.x, a.y]) + (half + a.gap_mm / 1000.0
                                + a.offset_mm / 1000.0) * ey
tgt = np.array([tcp_xy[0], tcp_xy[1], a.depth_mm / 1000.0 + 0.0063])

model = GripperModel(stride=4)


def fk(q):
    return model.solve(dict(zip(JOINTS, q)))


def solve(target, grip, w=0.6):
    def residual(arm):
        F = fk(np.r_[arm, grip])['gripper_frame_link']
        return np.r_[F[:3, 3] - target, w * (F[:3, :3] - R_des).ravel()]
    best, bc = None, None
    rng = np.random.default_rng(0)
    for s in [np.zeros(5)] + [lo[:5] + rng.random(5) * (hi[:5] - lo[:5])
                              for _ in range(9)]:
        sol = least_squares(residual, np.clip(s, lo[:5] + 1e-6, hi[:5] - 1e-6),
                            bounds=(lo[:5], hi[:5]), max_nfev=400)
        c = np.r_[sol.x, grip]
        e = float(np.linalg.norm(fk(c)['gripper_frame_link'][:3, 3] - target))
        if bc is None or e < bc:
            best, bc = c, e
    return best, bc


q, e = solve(tgt, 0.58)
T = fk(q)
F = T['gripper_frame_link']
print(f'抓取位 IK 误差 {e*1000:.2f}mm  TCP {np.round(F[:3,3],4).tolist()}')
print(f'方块中心(世界) {np.round(cube,4).tolist()}  '
      f'边 {a.cube_mm}mm  中心高 {cube[2]*1000:+.1f}mm')
print(f'工具 x 轴(=合爪方向) {np.round(F[:3,:3]@np.array([1,0,0]),4).tolist()}')
print(f'棋盘 ey {np.round(xax_w,4).tolist()}\n')

# 各连杆网格 -> 世界
for link in ('gripper_link', 'moving_jaw_so101_v1_link'):
    G = T[link]
    W = (G[:3, :3] @ model.parts[link].T).T + G[:3, 3]
    d = W - cube
    # 方块是 40mm 立方：|dx|,|dy|,|dz| <= half 则在内部
    inside = np.all(np.abs(d) <= half, axis=1)
    # 到方块表面的最短距离（在方块外的点）
    excess = np.abs(d) - half
    outside = ~inside
    dist = np.linalg.norm(np.maximum(excess[outside], 0), axis=1)
    print(f'--- {link} ---')
    print(f'  网格点 {len(W)}   在方块内部 {int(inside.sum())} 个')
    if outside.any():
        k = int(np.argmin(dist))
        Wk = W[outside][k]
        print(f'  距方块最近表面 {dist.min()*1000:.2f}mm  '
              f'点 ({Wk[0]:.4f},{Wk[1]:.4f},{Wk[2]*1000:+.1f}mm)')
    if inside.any():
        # 穿透深度
        pen = np.min(half - np.abs(d[inside]), axis=1)
        print(f'  ❌ 穿透最深 {pen.max()*1000:.2f}mm')
    print(f'  世界 z 范围 {(W[:,2].min()*1000):+.1f} .. {(W[:,2].max()*1000):+.1f} mm'
          f'   离桌面最近 {(W[:,2].min()-TABLE_Z)*1000:+.2f} mm')
    # 相对方块中心的 ey 方向投影范围
    proj = (W - cube) @ xax_w
    print(f'  沿 ey 相对方块中心: {proj.min()*1000:+.1f} .. {proj.max()*1000:+.1f} mm'
          f'  (方块面在 ±{half*1000:.1f})')
