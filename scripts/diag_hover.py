#!/usr/bin/env python3
"""诊断悬停位：工具轴竖直度 + 整网格最低点 vs 固定爪尖端(p_fix)。"""

from project_paths import arm_source_path, driver_params_path, project_path
import json
import math
import os
import sys

import numpy as np
from scipy.optimize import least_squares
import yaml
from pathlib import Path

sys.path.insert(0, arm_source_path())
sys.path.insert(0, os.path.expanduser(
    project_path('qianli_ws/src/qianli_vision/scripts')))
from gripper_model import GripperModel, JOINTS, FLANGE_LINK

CONFIG = os.path.expanduser(
    driver_params_path())
TABLE_Z = -0.06485
DOWN = np.array([0.0, 0.0, -1.0])
cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
lo = (np.array(cfg['raw_min']) - zero) * direction * 2 * math.pi / 4096
hi = (np.array(cfg['raw_max']) - zero) * direction * 2 * math.pi / 4096
lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)

model = GripperModel(stride=8)
T0 = model.solve(dict(zip(JOINTS, np.zeros(6))))
F0, G0 = T0['gripper_frame_link'], T0[FLANGE_LINK]
w0 = (G0[:3, :3] @ model.parts[FLANGE_LINK].T).T + G0[:3, 3]
pf = (F0[:3, :3].T @ (w0 - F0[:3, 3]).T).T
inner = pf[np.abs(pf[:, 0]) < 0.004]
p_fix = inner[int(np.argmin(inner[:, 2]))]
print(f'p_fix(工具系, 由零位求得) = {np.round(p_fix*1000,1).tolist()} mm')
print(f'  gripper_link 网格在工具系 z 范围 {pf[:,2].min()*1000:+.1f}..'
      f'{pf[:,2].max()*1000:+.1f} mm\n')

AFF = np.array([[0.07298559870136852, -1.1941346564477962],
                [-0.9239523684165104, -0.15094334179736751]])
rd = AFF[:, 1]
rd = np.array([rd[0], rd[1], 0.0])
rd /= np.linalg.norm(rd)
BLOCK = np.array([0.2197, -0.0910, 0.0])
face = BLOCK + 0.020 * rd


def fk(q):
    return model.solve(dict(zip(JOINTS, q)))


def tip(q):
    F = fk(q)['gripper_frame_link']
    return F[:3, 3] + F[:3, :3] @ p_fix


def build(target, seed, grip, w):
    def residual(arm):
        q = np.r_[arm, grip]
        F = fk(q)['gripper_frame_link']
        p = F[:3, 3] + F[:3, :3] @ p_fix
        zax = F[:3, :3] @ np.array([0.0, 0.0, 1.0])
        xax = F[:3, :3] @ np.array([1.0, 0.0, 0.0])
        return np.r_[p - target, w * (zax - DOWN), w * (xax - rd)]
    best, bc = None, None
    for s in [seed]:
        ss = np.clip(s[:5], lo[:5] + 1e-6, hi[:5] - 1e-6)
        sol = least_squares(residual, ss, bounds=(lo[:5], hi[:5]), max_nfev=600)
        c = np.r_[sol.x, grip]
        e = float(np.linalg.norm(tip(c) - target))
        if bc is None or e < bc:
            best, bc = c, e
    return best, bc


ready = json.load(open(os.path.expanduser(
    project_path('config/ready_pose.json'))))
q_r = (np.array(ready['raw_exec']) - zero) * direction * 2 * math.pi / 4096

for w in (0.15, 0.6, 1.5, 3.0):
    q, e = build(np.array([face[0], face[1], -0.006]), q_r, 0.58, w)
    F = fk(q)['gripper_frame_link']
    zax = F[:3, :3] @ np.array([0.0, 0.0, 1.0])
    tilt = math.degrees(math.acos(float(np.clip(zax @ DOWN, -1, 1))))
    t = tip(q)
    low, lk = model.lowest_over_all(dict(zip(JOINTS, q)))
    print(f'w={w:<5} 位置误差 {e*1000:6.2f}mm  工具轴偏离竖直 {tilt:6.1f}°  '
          f'尖端 z={t[2]*1000:+7.1f}mm  最低 {lk} z={low[2]*1000:+7.2f}mm')
