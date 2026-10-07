#!/usr/bin/env python3
"""扫描 y 偏置：用 mesh 算固定爪对方块的穿透/间隙，找出正确偏置。"""
import json
import math
import os
import sys

import numpy as np
from scipy.optimize import least_squares
import yaml
from pathlib import Path

sys.path.insert(0, os.path.expanduser(
    '~/QianLi/qianli_ws/src/qianli_vision/scripts'))
from gripper_model import GripperModel, JOINTS

CONFIG = os.path.expanduser(
    '~/legacy/arm/arm-final/ros2_ws/install/so101_bringup/share/so101_bringup'
    '/config/driver_params.yaml')
CFG = os.path.expanduser('~/QianLi/qianli_ws/config')
TABLE_Z = -0.06485
DOWN = np.array([0.0, 0.0, -1.0])
CUBE = np.array([0.2426, -0.0575, 0.0])
CUBE_MM = 40.0
DEPTH_MM = -48.0

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
half = CUBE_MM / 2000.0
cube = np.array([CUBE[0], CUBE[1], TABLE_Z + half])

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


print(f'方块中心 {np.round(cube,4).tolist()} 边 {CUBE_MM}mm')
print(f'{"偏置mm":>8} {"总偏移mm":>9} {"固定爪穿透mm":>12} {"固定爪最近面mm":>14} '
      f'{"活动爪最近面mm":>14}')
for off_mm in (-10, -5, 0, 3, 5, 7, 9, 11, 13, 16):
    tcp_xy = np.array([CUBE[0], CUBE[1]]) + (half + 0.0005
                                             + off_mm / 1000.0) * ey
    tgt = np.array([tcp_xy[0], tcp_xy[1], DEPTH_MM / 1000.0 + 0.0063])
    q, e = solve(tgt, 0.58)
    T = fk(q)
    res = []
    for link in ('gripper_link', 'moving_jaw_so101_v1_link'):
        G = T[link]
        W = (G[:3, :3] @ model.parts[link].T).T + G[:3, 3]
        d = W - cube
        inside = np.all(np.abs(d) <= half, axis=1)
        pen = 0.0
        if inside.any():
            pen = float(np.max(np.min(half - np.abs(d[inside]), axis=1)) * 1000)
        outside = ~inside
        gap = float(np.min(np.linalg.norm(
            np.maximum(np.abs(d[outside]) - half, 0), axis=1)) * 1000) \
            if outside.any() else 0.0
        res.append((pen, gap))
    print(f'{off_mm:8.1f} {half*1000+0.5+off_mm:9.1f} {res[0][0]:12.2f} '
          f'{res[0][1]:14.2f} {res[1][1]:14.2f}')
