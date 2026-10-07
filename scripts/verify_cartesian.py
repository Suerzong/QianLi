#!/usr/bin/env python3
"""数值验证笛卡尔下探：HOVER->抓取 直线IK跟踪（纯计算，不动臂）。

检验：
  1) TCP 路径相对理想竖直直线的最大偏差（应≈0）
  2) 全程 IK 误差（应 <1mm）
  3) 关节指令平滑性（速度连续）
"""
import json
import math
import os
import sys

import numpy as np
import yaml
from pathlib import Path
from scipy.optimize import least_squares

sys.path.insert(0, '/home/ros/legacy/arm/arm-final/ros2_ws/src/so101_bringup')
sys.path.insert(0, os.path.expanduser(
    '~/QianLi/qianli_ws/src/qianli_vision/scripts'))
from gripper_model import GripperModel, JOINTS

CONFIG = os.path.expanduser(
    '~/legacy/arm/arm-final/ros2_ws/install/so101_bringup/share/so101_bringup'
    '/config/driver_params.yaml')
CFG = os.path.expanduser('~/QianLi/qianli_ws/config')
cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
lo = (np.array(cfg['raw_min']) - zero) * direction * 2 * math.pi / 4096
hi = (np.array(cfg['raw_max']) - zero) * direction * 2 * math.pi / 4096
lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)

fr = json.load(open(os.path.join(CFG, 'board_frame.json')))
A = np.array(fr['affine'])
ey = A[:2, 1] / np.linalg.norm(A[:2, 1])
ey3 = np.array([ey[0], ey[1], 0.0])
DOWN = np.array([0.0, 0.0, -1.0])
R_des = np.column_stack([ey3, np.cross(DOWN, ey3), DOWN])
model = GripperModel(stride=8)


def fk(q):
    return model.solve(dict(zip(JOINTS, q)))


def solve(target, grip, ref):
    def residual(arm):
        q = np.r_[arm, grip]
        F = fk(q)['gripper_frame_link']
        return np.r_[F[:3, 3] - target, 0.6 * (F[:3, :3] - R_des).ravel()]
    best, bc = None, None
    rng = np.random.default_rng(0)
    seeds = [ref] + [lo[:5] + rng.random(5) * (hi[:5] - lo[:5])
                     for _ in range(8)]
    for s in seeds:
        ss = np.clip(s[:5], lo[:5] + 1e-6, hi[:5] - 1e-6)
        sol = least_squares(residual, ss, bounds=(lo[:5], hi[:5]),
                            max_nfev=400)
        c = np.r_[sol.x, grip]
        e = float(np.linalg.norm(fk(c)['gripper_frame_link'][:3, 3] - target))
        if bc is None or e < bc:
            best, bc = c, e
    return best, bc


# 用棋盘 (0,0) 上放的方块位置做目标（当前真实场景）
place = np.array([A[2][0], A[2][1]])
half = 33 / 2000.0
gap = 0.5 / 1000.0
tcp_xy = place + (half + gap) * ey
tip_z = -0.048
tcp_z = tip_z + 0.0063
tgt = np.array([tcp_xy[0], tcp_xy[1], tcp_z])
hover = tgt + np.array([0, 0, 0.050])

q_hi, e_hi = solve(hover, 0.58, np.zeros(6))
q_lo, e_lo = solve(tgt, 0.58, q_hi)
print(f'HOVER IK err {e_hi*1000:.2f}mm  GRASP IK err {e_lo*1000:.2f}mm')
p_start = fk(q_hi)['gripper_frame_link'][:3, 3]

N = 40
qt = q_hi.copy()
max_dev = 0.0
max_err = 0.0
for i in range(1, N + 1):
    si = 0.5 * (1 - np.cos(np.pi * i / N))
    tp = p_start + (tgt - p_start) * si
    q_c, e = solve(tp, 0.58, qt)
    F = fk(q_c)['gripper_frame_link'][:3, 3]
    # 直线偏差 = 实际TCP到理想竖直线（过tgt，沿z）的横向距离
    dev = np.linalg.norm(F[:2] - tgt[:2])
    max_dev = max(max_dev, float(dev))
    max_err = max(max_err, float(e))
    qt = q_c
print(f'直线跟踪: 最大横向偏差 {max_dev*1000:.3f}mm  '
      f'最大IK误差 {max_err*1000:.3f}mm')
# 关节速度平滑性
v = np.abs(np.diff(np.array(qt), axis=0)).max(axis=1) if False else None
# 打印几个采样点的关节差
qs = [qt]
print('关节轨迹平滑性: 相邻拍最大关节变化（末5拍）')
for j in range(6):
    diffs = np.abs(np.diff(qt[:, j]))
    print(f'  关节{j} max_step={diffs.max():.4f}rad')
