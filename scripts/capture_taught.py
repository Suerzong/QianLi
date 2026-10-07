#!/usr/bin/env python3
"""记录用户示范的抓取位姿 -> config/taught_grasp.json（并拍照）。"""

from project_paths import open_video_capture

from project_paths import default_arm_port

from project_paths import arm_source_path, default_camera, driver_params_path, project_path
import json
import math
import os
import sys
import time

import numpy as np
import yaml
from pathlib import Path

sys.path.insert(0, arm_source_path())
sys.path.insert(0, os.path.expanduser(
    project_path('qianli_ws/src/qianli_vision/scripts')))
from so101_bringup.servo_protocol import FeetechSerialBus
from gripper_model import GripperModel, JOINTS, FLANGE_LINK

CONFIG = os.path.expanduser(
    driver_params_path())
OUT = os.path.expanduser(project_path('config/taught_grasp.json'))

cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
bus = FeetechSerialBus(default_arm_port(), timeout_s=0.15)
raw = np.array(bus.read_positions())
torque = bus.read_torque_states()
q = (raw - zero) * direction * 2 * math.pi / 4096
bus.close()

model = GripperModel(stride=8)
T = model.solve(dict(zip(JOINTS, q)))
F = T['gripper_frame_link']
low, lk = model.lowest_over_all(dict(zip(JOINTS, q)))
open_m = model.jaw_opening(float(q[5]))
tcp = F[:3, 3]

# 取张照片
try:
    import cv2
    cap = open_video_capture(default_camera())
    img = None
    for _ in range(10):
        ok, f = cap.read()
        if ok:
            img = f
    cap.release()
    if img is not None:
        cv2.imwrite('/tmp/taught_grasp.jpg', img)
        print('已拍 /tmp/taught_grasp.jpg')
except Exception as exc:
    print('拍照失败:', exc)

os.makedirs(os.path.dirname(OUT), exist_ok=True)
json.dump({'raw': raw.tolist(), 'q_rad': q.tolist(),
           'torque_at_capture': torque,
           'tcp_m': tcp.tolist(),
           'jaw_opening_mm': None if open_m is None else open_m * 1000,
           'lowest_mm': float(low[2] * 1000), 'lowest_link': lk,
           'at': time.strftime('%Y-%m-%d %H:%M:%S'),
           'note': '用户手拖示范的抓取位姿（爪指套住方块）'},
          open(OUT, 'w'), indent=2, ensure_ascii=False)
print(f'✅ 已保存示范位姿 → {OUT}')
print(f'  实测 raw   {raw.tolist()}')
print(f'  关节角     {np.round(q,4).tolist()}')
print(f'  TCP        ({tcp[0]:.4f},{tcp[1]:.4f},{tcp[2]*1000:+.1f}mm)')
print(f'  爪口开度   {"量不到" if open_m is None else f"{open_m*1000:.1f} mm"}')
print(f'  整臂最低   {lk} z={low[2]*1000:+.2f}mm')
print(f'  采集时力矩 {torque}')
