#!/usr/bin/env python3
"""离线计算给定 raw 位置对应的姿态（不碰总线）。"""

from project_paths import driver_params_path, project_path
import math
import os
import sys

import numpy as np
import yaml
from pathlib import Path

sys.path.insert(0, os.path.expanduser(
    project_path('qianli_ws/src/qianli_vision/scripts')))
from gripper_model import GripperModel, JOINTS

CONFIG = os.path.expanduser(
    driver_params_path())
cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])

RAW = [2301, 2962, 1751, 2892, 2990, 2404]
q = (np.array(RAW) - zero) * direction * 2 * math.pi / 4096
print('关节角(rad):')
for n, v in zip(JOINTS, q):
    print(f'  {n:>15} {v:+.4f}')

model = GripperModel(stride=8)
T = model.solve(dict(zip(JOINTS, q)))
for name in ('gripper_link', 'gripper_frame_link', 'tcp_link'):
    p = T[name][:3, 3]
    print(f'{name:>22} = ({p[0]:.4f}, {p[1]:.4f}, {p[2]*1000:+.1f} mm)')
low, link = model.lowest_over_all(dict(zip(JOINTS, q)))
print(f'整臂最低点: {link} z={low[2]*1000:+.2f} mm  (桌面 -69.09)')
print(f'  → 离桌面 {(low[2]+0.06909)*1000:+.2f} mm')
gl, _ = model.lowest_point(dict(zip(JOINTS, q)))
print(f'夹爪最低点: {gl[2]*1000:+.2f} mm')
