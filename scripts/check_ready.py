#!/usr/bin/env python3
"""验算 ready_pose / home_pose 的 FK、固定爪尖端与净空。"""

from project_paths import driver_params_path, project_path
import json
import math
import os
import sys

import numpy as np
import yaml
from pathlib import Path

sys.path.insert(0, os.path.expanduser(
    project_path('qianli_ws/src/qianli_vision/scripts')))
from gripper_model import GripperModel, JOINTS, FLANGE_LINK

CONFIG = os.path.expanduser(
    driver_params_path())
CFG = os.path.expanduser(project_path('config'))
TABLE_Z = -0.06485
cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
model = GripperModel(stride=10)
T0 = model.solve(dict(zip(JOINTS, np.zeros(6))))
F0, G0 = T0['gripper_frame_link'], T0[FLANGE_LINK]
w0 = (G0[:3, :3] @ model.parts[FLANGE_LINK].T).T + G0[:3, 3]
pf = (F0[:3, :3].T @ (w0 - F0[:3, 3]).T).T
inner = pf[np.abs(pf[:, 0]) < 0.004]
p_fix = inner[int(np.argmin(inner[:, 2]))]

for name in ('ready_pose', 'home_pose'):
    path = os.path.join(CFG, name + '.json')
    if not os.path.exists(path):
        print(f'{name}: 不存在')
        continue
    d = json.load(open(path))
    raw = np.array(d['raw_exec'])
    q = (raw - zero) * direction * 2 * math.pi / 4096
    T = model.solve(dict(zip(JOINTS, q)))
    F = T['gripper_frame_link']
    tip = F[:3, 3] + F[:3, :3] @ p_fix
    low, lk = model.lowest_over_all(dict(zip(JOINTS, q)))
    print(f'{name}: raw {raw.tolist()}')
    print(f'  关节角 {np.round(q,4).tolist()}')
    print(f'  固定爪尖端 ({tip[0]:.4f}, {tip[1]:.4f}, {tip[2]*1000:+.1f}mm)  '
          f'离桌面 {(tip[2]-TABLE_Z)*1000:+.1f}mm')
    print(f'  整臂最低点 {lk} z={low[2]*1000:+.2f}mm  '
          f'离桌面 {(low[2]-TABLE_Z)*1000:+.1f}mm')
    print(f'  工具轴竖直度: 尖端在 TCP '
          f'{"下方" if tip[2] < F[2,3] else "上方"} '
          f'{abs(tip[2]-F[2,3])*1000:.1f}mm')
    print()
