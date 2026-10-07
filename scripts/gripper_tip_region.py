#!/usr/bin/env python3
"""看 gripper_link 网格的几何范围（frame 系），确定"爪尖区域"怎么截取。"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.expanduser('~/QianLi/qianli_ws/src/qianli_vision/scripts'))
from gripper_model import GripperModel, JOINTS

BASE = dict(zip(JOINTS, [0.4034, 0.6519, 0.1335, 0.8744, -1.5601, 0.7056]))
model = GripperModel(stride=14)

# gripper_link 网格点，转到 gripper_link 自身 frame
pts = model.parts['gripper_link']
print(f'gripper_link 网格采样点: {len(pts)}')
print(f'X 范围: {pts[:,0].min():+.4f} ~ {pts[:,0].max():+.4f} m')
print(f'Y 范围: {pts[:,1].min():+.4f} ~ {pts[:,1].max():+.4f} m')
print(f'Z 范围: {pts[:,2].min():+.4f} ~ {pts[:,2].max():+.4f} m')
print()

# 爪尖 = 爪口方向最前端。SO-101 抓取时工具轴朝下，爪口朝 gripper_frame X+？
# 看 tcp_link 相对 gripper_link 的朝向：
T = model.solve(BASE)
M_tcp = T['tcp_link']
M_gr = T['gripper_link']
# tcp 在 gripper_link frame 下的位置
tcp_in_gr = np.linalg.inv(M_gr) @ np.hstack([M_tcp[:3, 3], 1.0])
print(f'TCP 在 gripper_link frame 下: {tcp_in_gr[:3]*1000}')
print()

# 找出 frame X 最大（爪口前端）区域的最低点
for x_front_mm in (8, 10, 12, 15, 20):
    xmin = pts[:, 0].max() - x_front_mm / 1000.0
    mask = pts[:, 0] >= xmin
    sub = pts[mask]
    if len(sub) == 0:
        continue
    k = int(np.argmin(sub[:, 2]))
    tip = sub[k]
    print(f'取 X 前端 {x_front_mm:>3}mm: {len(sub):>6} 点, 最低点 '
          f'({tip[0]*1000:+.1f}, {tip[1]*1000:+.1f}, {tip[2]*1000:+.1f}) mm [frame]')
