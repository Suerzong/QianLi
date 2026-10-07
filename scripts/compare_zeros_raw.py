#!/usr/bin/env python3
"""同一个实测 raw，分别按"新零点"和"旧零点"算固定爪尖端高度。
板面在 -68.59mm。谁算出来接近板面，谁的零点就是对的。"""

from project_paths import project_path
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.expanduser(
    project_path('qianli_ws/src/qianli_vision/scripts')))
from gripper_model import GripperModel, JOINTS

K = 2 * math.pi / 4096
RAW = [1993, 2796, 2412, 2963, 2025, 2472]
BOARD_Z = -0.06859
OLD_ZERO = [2078, 1980, 3076, 2035, 2033, 2030]
NEW_ZERO = [2020, 1980, 2988, 2014, 2013, 2030]

model = GripperModel(stride=10)


def tip_z(zero):
    q = (np.array(RAW) - zero) * K
    T = model.solve(dict(zip(JOINTS, q)))
    M = T['gripper_link']
    pts = model.parts['gripper_link']
    w = (M[:3, :3] @ pts.T).T + M[:3, 3]
    k = int(np.argmin(w[:, 2]))
    low, link = model.lowest_over_all(dict(zip(JOINTS, q)))
    return w[k], q, low, link


for tag, z in (('旧零点', OLD_ZERO), ('新零点', NEW_ZERO)):
    tip, q, low, link = tip_z(np.array(z))
    print(f'{tag} {z}')
    print(f'  角度 {np.round(q,4).tolist()}')
    print(f'  固定爪尖端 ({tip[0]:.4f}, {tip[1]:.4f}, {tip[2]*1000:+.2f}mm)')
    print(f'  离板面 {(tip[2]-BOARD_Z)*1000:+.2f} mm')
    print(f'  整臂最低点 {link} z={low[2]*1000:+.2f}mm '
          f'(离板面 {(low[2]-BOARD_Z)*1000:+.2f})')
    print()
print(f'板面 z = {BOARD_Z*1000:.2f} mm')
print(f'raw = {RAW}')
