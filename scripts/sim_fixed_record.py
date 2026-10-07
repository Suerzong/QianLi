#!/usr/bin/env python3
"""用当前实时关节角模拟修复后的录点逻辑：固定爪顶端 + 板面 z 判据。

旧 marks 第 10 点 (9.9, 6.6) 记录于当前这个姿态 —— 如果修复后的逻辑
在相同姿态下给出相同/相近的固定爪顶端，且 z 判据通过，说明修复是自洽的。
"""

from project_paths import project_path
import os
import sys

import numpy as np

sys.path.insert(0, os.path.expanduser(project_path('qianli_ws/src/qianli_vision/scripts')))
from gripper_model import GripperModel, JOINTS

Q = [0.3666214083046682, 0.7378447589729934, -0.007669903939428206,
     0.9602719732164113, -1.0983302441261191, 0.7071651432152806]
joints = dict(zip(JOINTS, Q))
BOARD_Z = -0.06909 + 0.0005
Z_TOL = 2.0 / 1000.0

model = GripperModel(stride=14)
T = model.solve(joints)
pts = model.parts['gripper_link']
M = T['gripper_link']
world = (M[:3, :3] @ pts.T).T + M[:3, 3]
k = int(np.argmin(world[:, 2]))
tip = world[k]
tcp = T['tcp_link'][:3, 3]

print('修复后录点逻辑（当前实时姿态）:')
print(f'  固定爪顶端 = ({tip[0]:+.4f}, {tip[1]:+.4f}, {tip[2]:+.4f})')
print(f'  板面 z     = {BOARD_Z*1000:+.2f} mm')
z_err = tip[2] - BOARD_Z
print(f'  z-板面     = {z_err*1000:+.2f} mm  (容差 ±{Z_TOL*1000:.1f})  '
      f'{"✅ 通过，可记录" if abs(z_err) < Z_TOL else "❌ 拒绝"}')
print(f'  TCP        = ({tcp[0]:+.4f}, {tcp[1]:+.4f}, {tcp[2]:+.4f})')
print()
print('对照旧 marks 第 10 点 (9.9, 6.6):')
print(f'  contact_m  = (0.2357, -0.0674, -0.0688)   '
      f'z-板面 = {-0.23:+.2f} mm')
print(f'  差异 (新-旧) = '
      f'({(tip[0]-0.2357)*1000:+.1f}, {(tip[1]-(-0.0674))*1000:+.1f}, '
      f'{(tip[2]-(-0.0688))*1000:+.1f}) mm')
