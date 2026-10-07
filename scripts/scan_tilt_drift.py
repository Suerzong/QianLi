#!/usr/bin/env python3
"""扫描工具倾角：gripper_link 网格最低点是否从"爪尖"漂移到爪根/侧边？

如果漂移，说明"固定爪顶端=网格最低点"在倾角大时不可靠——记录的 XY
参考点会随姿态变化而错位（这就是 RMS 60mm 的真凶候选）。
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.expanduser('~/QianLi/qianli_ws/src/qianli_vision/scripts'))
from gripper_model import GripperModel, JOINTS

BASE = dict(zip(JOINTS, [0.4034, 0.6519, 0.1335, 0.8744, -1.5601, 0.7056]))
model = GripperModel(stride=14)
tcp = model.tcp(BASE)

print(f'基准姿态 TCP = ({tcp[0]:.4f}, {tcp[1]:.4f}, {tcp[2]:.4f})')
print(f'{"倾角(°)":>8}{"最低点link":>14}{"最低点 z(mm)":>14}{"水平偏移(mm)":>14}')
print('-' * 52)

for droll in np.linspace(-0.6, 0.6, 13):
    q = dict(BASE)
    q['wrist_roll'] = BASE['wrist_roll'] + droll
    T = model.solve(q)
    M = T['gripper_link']
    pts = model.parts['gripper_link']
    world = (M[:3, :3] @ pts.T).T + M[:3, 3]
    k = int(np.argmin(world[:, 2]))
    low = world[k]
    # 工具轴（gripper_link 的 z 轴在世界系）
    zaxis = M[:3, :3] @ np.array([0.0, 0.0, 1.0])
    tilt = np.degrees(np.arccos(min(1.0, abs(zaxis[2]))))
    h = np.hypot(low[0] - tcp[0], low[1] - tcp[1]) * 1000
    print(f'{tilt:>8.1f}{"gripper_link":>14}{low[2]*1000:>14.2f}{h:>14.2f}')
