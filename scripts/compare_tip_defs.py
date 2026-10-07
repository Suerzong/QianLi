#!/usr/bin/env python3
"""对比两种"固定爪顶端"定义在 wrist_roll 旋转下的漂移：
A) 整体网格最低点（现用，会漂移）
B) X 前端窗口最低点（爪尖，应稳定）
"""

from project_paths import project_path
import os
import sys

import numpy as np

sys.path.insert(0, os.path.expanduser(project_path('qianli_ws/src/qianli_vision/scripts')))
from gripper_model import GripperModel, JOINTS

BASE = dict(zip(JOINTS, [0.4034, 0.6519, 0.1335, 0.8744, -1.5601, 0.7056]))
model = GripperModel(stride=14)
FRONT_MM = 12.0  # 爪口前端窗口

print(f'{"wrist_roll":>10}{"A最低z(mm)":>12}{"A水平偏移":>12}'
      f'{"B爪尖z(mm)":>12}{"B水平偏移":>12}')
print('-' * 58)
for droll in np.linspace(-0.6, 0.6, 13):
    q = dict(BASE)
    q['wrist_roll'] = BASE['wrist_roll'] + droll
    T = model.solve(q)
    M = T['gripper_link']
    pts = model.parts['gripper_link']
    world = (M[:3, :3] @ pts.T).T + M[:3, 3]
    tcp = T['tcp_link'][:3, 3]

    # A: 整体最低点
    kA = int(np.argmin(world[:, 2]))
    lowA = world[kA]
    hA = np.hypot(lowA[0] - tcp[0], lowA[1] - tcp[1]) * 1000

    # B: X 前端窗口（gripper_link frame X 最大区域）
    xmax = pts[:, 0].max()
    mask = pts[:, 0] >= xmax - FRONT_MM / 1000.0
    sub = world[mask]
    if len(sub):
        kB = int(np.argmin(sub[:, 2]))
        lowB = sub[kB]
        hB = np.hypot(lowB[0] - tcp[0], lowB[1] - tcp[1]) * 1000
    else:
        lowB, hB = np.zeros(3), float('nan')
    print(f'{droll:>+10.2f}{lowA[2]*1000:>12.2f}{hA:>12.2f}'
          f'{lowB[2]*1000:>12.2f}{hB:>12.2f}')
