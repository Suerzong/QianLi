#!/usr/bin/env python3
"""只读复现 quick_regrasp 抬升规划段的报错（不写寄存器）。"""
import math
import sys
import traceback

import numpy as np
from scipy.optimize import least_squares
import yaml
from pathlib import Path

sys.path.insert(0, '/home/ros/legacy/arm/arm-final/ros2_ws/src/so101_bringup')
sys.path.insert(0, '/home/ros/QianLi/qianli_ws/src/qianli_vision/scripts')
from so101_bringup.servo_protocol import FeetechSerialBus
from gripper_model import GripperModel, JOINTS

CONFIG = ('/home/ros/legacy/arm/arm-final/ros2_ws/install/so101_bringup/share/'
          'so101_bringup/config/driver_params.yaml')
TABLE_Z = -0.06909
cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
lo = (np.array(cfg['raw_min']) - zero) * direction * 2 * math.pi / 4096
hi = (np.array(cfg['raw_max']) - zero) * direction * 2 * math.pi / 4096
lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)

bus = FeetechSerialBus('/dev/ttyACM0', timeout_s=0.08)
q_hold = (np.array(bus.read_positions()) - zero) * direction * 2 * math.pi / 4096
bus.close()
print('q_hold =', np.round(q_hold, 4).tolist())

model = GripperModel(stride=12)


def frame(q):
    return model.solve(dict(zip(JOINTS, q)))['gripper_frame_link']


F = frame(q_hold)
print('type(frame) =', type(F), 'shape =', np.shape(F))
try:
    p0 = F[:3, 3].copy()
    axis = F[:3, 2].copy()
    print('p0 =', np.round(p0, 4).tolist())
except Exception:
    traceback.print_exc()

try:
    seed = q_hold[:5].copy()
    waypoints = []
    for z in np.linspace(0, 0.04, 9)[1:]:
        target = p0 + [0, 0, z]
        def residual(arm):
            f = frame(np.r_[arm, q_hold[5]])
            return np.r_[f[:3, 3] - target, 0.05 * (f[:3, 2] - axis)]
        sol = least_squares(residual, seed, bounds=(lo[:5], hi[:5]), max_nfev=100)
        cand = np.r_[sol.x, q_hold[5]]
        err = float(np.linalg.norm(frame(cand)[:3, 3] - target))
        lw, lk = model.lowest_over_all(dict(zip(JOINTS, cand)))
        print(f'z={z*1000:+.1f}mm err={err*1000:.2f}mm lowest={lw[2]*1000:+.2f}mm')
        waypoints.append(cand)
        seed = sol.x
    print('规划成功, 点数', len(waypoints))
except Exception:
    traceback.print_exc()
