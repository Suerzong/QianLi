#!/usr/bin/env python3
"""舵机力矩开关 + 关节回读（用于解除/恢复过载保护）。"""
import argparse
import math
import sys
import time

import numpy as np
import yaml
from pathlib import Path

sys.path.insert(0, '/home/ros/legacy/arm/arm-final/ros2_ws/src/so101_bringup')
from so101_bringup.servo_protocol import FeetechSerialBus

CONFIG = ('/home/ros/legacy/arm/arm-final/ros2_ws/install/so101_bringup/share/'
          'so101_bringup/config/driver_params.yaml')
cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])

ap = argparse.ArgumentParser()
g = ap.add_mutually_exclusive_group(required=True)
g.add_argument('--on', action='store_true')
g.add_argument('--off', action='store_true')
ap.add_argument('--wait', type=float, default=0.6)
a = ap.parse_args()

bus = FeetechSerialBus('/dev/ttyACM0', timeout_s=0.08)
if a.on:
    bus.set_torque(True)
    print('torque ON')
else:
    bus.set_torque(False)
    print('torque OFF')
time.sleep(a.wait)
print('扭矩状态:', bus.read_torque_states())
raw = np.array(bus.read_positions())
q = (raw - zero) * direction * 2 * math.pi / 4096
print('关节角(rad):', np.round(q, 4).tolist())
print('原始计数:', raw.tolist())
bus.close()
