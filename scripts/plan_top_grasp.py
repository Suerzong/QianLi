#!/usr/bin/env python3
"""规划上方抓取：把 TCP 移到物块正上方、工具轴竖直向下、爪口罩住物块。

只做规划与可行性检查，不写寄存器。
"""
import math
import os
import sys

import numpy as np
from scipy.optimize import least_squares
import yaml
from pathlib import Path

sys.path.insert(0, os.path.expanduser('~/legacy/arm/arm-final/ros2_ws/src/so101_bringup'))
sys.path.insert(0, os.path.expanduser('~/QianLi/qianli_ws/src/qianli_vision/scripts'))
from so101_bringup.servo_protocol import FeetechSerialBus
from gripper_model import GripperModel, JOINTS

CONFIG = os.path.expanduser(
    '~/legacy/arm/arm-final/ros2_ws/install/so101_bringup/share/so101_bringup'
    '/config/driver_params.yaml')
TABLE_Z = -0.06909
BLOCK = np.array([0.346, 0.024])       # 物块中心 xy（放下时的 TCP）
BLOCK_H = 0.040                        # 4cm EVA 方块
BLOCK_TOP = TABLE_Z + BLOCK_H

cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
lo = (np.array(cfg['raw_min']) - zero) * direction * 2 * math.pi / 4096
hi = (np.array(cfg['raw_max']) - zero) * direction * 2 * math.pi / 4096
lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)

bus = FeetechSerialBus('/dev/ttyACM0', timeout_s=0.08)
q = (np.array(bus.read_positions()) - zero) * direction * 2 * math.pi / 4096
bus.close()
model = GripperModel(stride=12)


def fk(qq):
    return model.solve(dict(zip(JOINTS, qq)))


f = fk(q)['gripper_frame_link']
low, link = model.lowest_over_all(dict(zip(JOINTS, q)))
print('当前姿态:')
print('  q =', np.round(q, 4).tolist())
print(f'  TCP = {np.round(f[:3,3],4).tolist()}')
print(f'  工具轴 f[:3,2] = {np.round(f[:3,2],4).tolist()}')
print(f'  最低点 {link} z={low[2]*1000:+.2f}mm')

# 爪尖相对工具坐标系的常向量（固定几何）
c = f[:3, :3].T @ (low - f[:3, 3])
print(f'  爪尖在工具系中的常向量 c = {np.round(c*1000,2).tolist()} mm '
      f'(|c|={np.linalg.norm(c)*1000:.1f}mm)')

# 目标：工具轴竖直向下 = 爪尖在 TCP 下方 |c|
open_angle, clamped = model.angle_for_opening(0.055)
print(f'\n爪口开到 55mm 需要夹爪角 {open_angle:.4f} rad (限幅={clamped})')
print(f'物块 40mm 对应夹爪角 {model.angle_for_opening(0.033)[0]:.4f} rad')

# 目标 TCP：物块正上方，爪尖高度 = 物块顶上方 15mm
want_tip_z = BLOCK_TOP + 0.015
target_tcp = np.array([BLOCK[0], BLOCK[1], want_tip_z + abs(c[2])])
print(f'\n目标: TCP={np.round(target_tcp,4).tolist()} '
      f'(爪尖应在 z={want_tip_z*1000:+.1f}mm, 物块顶 {BLOCK_TOP*1000:+.1f}mm)')

# 解 IK：TCP 到目标位置，且工具轴竖直（c 指向 -z）
down = np.array([0.0, 0.0, -1.0])


def residual(arm):
    ff = fk(np.r_[arm, open_angle])['gripper_frame_link']
    tip_vec = ff[:3, :3] @ c
    return np.r_[ff[:3, 3] - target_tcp,
                 2.0 * (tip_vec / np.linalg.norm(tip_vec) - down)]


seed = q[:5].copy()
sol = least_squares(residual, seed, bounds=(lo[:5], hi[:5]), max_nfev=400)
cand = np.r_[sol.x, open_angle]
ff = fk(cand)['gripper_frame_link']
err = float(np.linalg.norm(ff[:3, 3] - target_tcp))
tip_vec = ff[:3, :3] @ c
ang = math.degrees(math.acos(float(np.clip(tip_vec @ down / np.linalg.norm(tip_vec), -1, 1))))
low2, link2 = model.lowest_over_all(dict(zip(JOINTS, cand)))
print(f'\nIK 结果: TCP误差 {err*1000:.1f}mm, 工具轴偏离竖直 {ang:.1f}°, '
      f'最低点 {link2} z={low2[2]*1000:+.2f}mm')
print(f'  解 q = {np.round(cand,4).tolist()}')
print(f'  关节变化 = {np.round(cand[:5]-q[:5],4).tolist()}')
print(f'  是否在限位内: {bool(np.all(cand[:5] >= lo[:5]-1e-6) and np.all(cand[:5] <= hi[:5]+1e-6))}')
print(f'  最大单关节变化 {np.max(np.abs(cand[:5]-q[:5])):.3f} rad')
