#!/usr/bin/env python3
"""诊断：从折叠位出发，几种路径方案各自的最低净空。

比较：
  A) 关节空间直线 -> 悬停位 q_hi
  B) 关节空间直线 -> 示教抓取模板位（已知安全、靠近工作区）
  C) 关节空间直线 -> 先到"高抬伸前"中间位，再到 q_hi
"""
import math
import os
import sys

import numpy as np
from scipy.optimize import least_squares
import yaml
from pathlib import Path

sys.path.insert(0, '/home/ros/legacy/arm/arm-final/ros2_ws/src/so101_bringup')
sys.path.insert(0, os.path.expanduser(
    '~/QianLi/qianli_ws/src/qianli_vision/scripts'))
from gripper_model import GripperModel, JOINTS, FLANGE_LINK

CONFIG = os.path.expanduser(
    '~/legacy/arm/arm-final/ros2_ws/install/so101_bringup/share/so101_bringup'
    '/config/driver_params.yaml')
TABLE_Z = -0.06485
DOWN = np.array([0.0, 0.0, -1.0])
cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
lo = (np.array(cfg['raw_min']) - zero) * direction * 2 * math.pi / 4096
hi = (np.array(cfg['raw_max']) - zero) * direction * 2 * math.pi / 4096
lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)

Q_FOLD = np.array([2096, 795, 4099, 844, 2390, 2326])
q_fold = (Q_FOLD - zero) * direction * 2 * math.pi / 4096
TEACH = np.array([-0.11658253987930872, 0.9480001269133262,
                  -0.4586602555778067, 1.2363885150358267,
                  -1.5508545765523831, 0.3098641191528995])

model = GripperModel(stride=10)
T0 = model.solve(dict(zip(JOINTS, np.zeros(6))))
F0, G0 = T0['gripper_frame_link'], T0[FLANGE_LINK]
w0 = (G0[:3, :3] @ model.parts[FLANGE_LINK].T).T + G0[:3, 3]
pf = (F0[:3, :3].T @ (w0 - F0[:3, 3]).T).T
inner = pf[np.abs(pf[:, 0]) < 0.004]
p_fix = inner[int(np.argmin(inner[:, 2]))]


def fk(q):
    return model.solve(dict(zip(JOINTS, q)))


def tip(q):
    F = fk(q)['gripper_frame_link']
    return F[:3, 3] + F[:3, :3] @ p_fix


def lowest(q):
    return model.lowest_over_all(dict(zip(JOINTS, q)))


BLOCK = np.array([0.2197, -0.0910, 0.0])
AFF = np.array([[0.07298559870136852, -1.1941346564477962],
                [-0.9239523684165104, -0.15094334179736751]])
rd = AFF[:, 1]
rd = np.array([rd[0], rd[1], 0.0])
rd /= np.linalg.norm(rd)
face = BLOCK + 0.020 * rd
tgt_hi = np.array([face[0], face[1], -0.006])


def solve(target, seed, grip=0.58, align=True):
    w = 0.15 if align else 0.0

    def residual(arm):
        q = np.r_[arm, grip]
        F = fk(q)['gripper_frame_link']
        p = F[:3, 3] + F[:3, :3] @ p_fix
        zax = F[:3, :3] @ np.array([0.0, 0.0, 1.0])
        xax = F[:3, :3] @ np.array([1.0, 0.0, 0.0])
        return np.r_[p - target, w * (zax - DOWN), w * (xax - rd)]
    best, bc = None, None
    rng = np.random.default_rng(0)
    seeds = [seed] + [lo[:5] + rng.random(5) * (hi[:5] - lo[:5])
                      for _ in range(8)]
    for s in seeds:
        ss = np.clip(s[:5], lo[:5] + 1e-6, hi[:5] - 1e-6)
        sol = least_squares(residual, ss, bounds=(lo[:5], hi[:5]), max_nfev=400)
        c = np.r_[sol.x, grip]
        e = float(np.linalg.norm(tip(c) - target))
        if bc is None or e < bc:
            best, bc = c, e
    return best, bc


print(f'折叠位爪尖 z = {tip(q_fold)[2]*1000:+.1f}mm')
print(f'桌面 z = {TABLE_Z*1000:+.2f}mm\n')

q_hi, e_hi = solve(tgt_hi, q_fold)
print(f'悬停位 IK 误差 {e_hi*1000:.1f}mm  '
      f'爪尖 z={tip(q_hi)[2]*1000:+.1f}mm')
q_hi_high, e2 = solve(np.array([face[0], face[1], 0.12]), q_fold)
print(f'高位悬停(z=+120mm) IK 误差 {e2*1000:.1f}mm\n')


def scan(name, path):
    worst = None
    for q in path:
        low, lk = lowest(q)
        if worst is None or low[2] < worst[0]:
            worst = (low[2], lk, q)
    ok = worst[0] >= TABLE_Z + 0.006
    print(f'{name}: 最低 {worst[1]} z={worst[0]*1000:+.2f}mm  '
          f'{"✅ 安全" if ok else "❌ 低于桌面"}')
    return ok


for n in (8, 16, 30):
    scan(f'A) 关节直线 折叠->悬停 ({n}段)',
         [q_fold + (q_hi - q_fold) * i / n for i in range(1, n + 1)])
for n in (8, 16, 30):
    scan(f'B) 关节直线 折叠->示教模板 ({n}段)',
         [q_fold + (TEACH - q_fold) * i / n for i in range(1, n + 1)])
scan('C) 折叠->高位悬停->悬停',
     [q_fold + (q_hi_high - q_fold) * i / 12 for i in range(1, 13)]
     + [q_hi_high + (q_hi - q_hi_high) * i / 12 for i in range(1, 13)])
