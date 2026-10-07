#!/usr/bin/env python3
"""读取当前舵机状态（位置/扭矩/夹爪载荷），不写任何寄存器。"""

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
zero = np.array(cfg['zero_raw'])
direction = np.array(cfg['direction'])

bus = FeetechSerialBus(default_arm_port(), timeout_s=0.08)
raw = np.array(bus.read_positions())
torque = bus.read_torque_states()
load = bus.read_gripper_load()
q = (raw - zero) * direction * 2 * math.pi / 4096

print('关节角 (rad):')
for name, v, t in zip(JOINTS, q, torque):
    print(f'  {name:>16} {v:+.4f}   torque={t}')
print(f'夹爪载荷: {load[0]:.1f}%')
print(f'原始计数: {raw.tolist()}')

m = GripperModel(stride=14)
T = m.solve(dict(zip(JOINTS, q)))
tcp = T['tcp_link'][:3, 3]
frame = T['gripper_frame_link'][:3, 3]
print(f'TCP(gripper_frame_link) = ({frame[0]:.4f}, {frame[1]:.4f}, {frame[2]:.4f})')
low, link = m.lowest_over_all(dict(zip(JOINTS, q)))
print(f'最低点 {link} z = {low[2]*1000:+.2f} mm (桌面 -69.09 mm)')
print(f'夹爪开合 = {q[5]:.4f} rad (小=闭合)')
bus.close()
