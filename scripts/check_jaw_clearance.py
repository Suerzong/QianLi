#!/usr/bin/env python3
"""当前臂姿下，夹爪从张开到闭合，几何最低点如何变化（查桌面保护是否误触发）。"""

from project_paths import default_arm_port

from project_paths import arm_source_path, driver_params_path, project_path
import math
import os
import sys

import numpy as np
import yaml
from pathlib import Path

sys.path.insert(0, os.path.expanduser(arm_source_path()))
sys.path.insert(0, os.path.expanduser(project_path('qianli_ws/src/qianli_vision/scripts')))
from so101_bringup.servo_protocol import FeetechSerialBus
from gripper_model import GripperModel, JOINTS

CONFIG = os.path.expanduser(
    driver_params_path())
cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])

bus = FeetechSerialBus(default_arm_port(), timeout_s=0.08)
q = (np.array(bus.read_positions()) - zero) * direction * 2 * math.pi / 4096
bus.close()
model = GripperModel(stride=12)

print(f'{"夹爪(rad)":>10}{"最低z(mm)":>12}{"最低link":>34}')
for g in [0.69, 0.6, 0.5, 0.45, 0.4, 0.35, 0.3, 0.25, 0.2, 0.15, 0.1, 0.05]:
    qq = q.copy()
    qq[5] = g
    low, link = model.lowest_over_all(dict(zip(JOINTS, qq)))
    flag = '  <-- 低于桌面' if low[2] < -0.06909 + 0.0005 else ''
    print(f'{g:>10.2f}{low[2]*1000:>12.2f}{link:>34}{flag}')
print(f'\n桌面 z = -69.09mm，保护阈值 = -68.59mm')
print(f'当前夹爪 {q[5]:.4f} rad')
