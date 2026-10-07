#!/usr/bin/env python3
"""打印关节实测限位与当前角度的对比，判断是否已到限位。"""

from project_paths import driver_params_path
import math
from pathlib import Path

import numpy as np
import yaml

CONFIG = (driver_params_path())
cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
lo = (np.array(cfg['raw_min']) - zero) * direction * 2 * math.pi / 4096
hi = (np.array(cfg['raw_max']) - zero) * direction * 2 * math.pi / 4096
lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)

cur = np.array([-0.0660, 1.2164, -0.9449, 1.2226, -2.7167, 0.5292])
names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
         'wrist_roll', 'gripper']
print(f'{"关节":>16}{"当前":>10}{"下限":>10}{"上限":>10}  余量')
for n, c, a, b in zip(names, cur, lo, hi):
    margin = min(c - a, b - c)
    flag = '  <-- 贴限位' if abs(margin) < 0.05 else ''
    print(f'{n:>16}{c:>10.3f}{a:>10.3f}{b:>10.3f}  {margin:+.3f}{flag}')
print()
print('raw_min =', cfg['raw_min'])
print('raw_max =', cfg['raw_max'])
print('zero_raw =', cfg['zero_raw'])
print('calibrated =', cfg['calibrated'])
