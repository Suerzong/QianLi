#!/usr/bin/env python3
"""检查 stride 子采样是否让"固定爪最低点"漏掉真实最低顶点。"""

from project_paths import project_path
import os
import sys

import numpy as np

sys.path.insert(0, os.path.expanduser(project_path('qianli_ws/src/qianli_vision/scripts')))
from gripper_model import GripperModel, JOINTS

JOINTS_VAL = dict(zip(JOINTS, [0.3666, 0.7378, -0.0077, 0.9603, -1.0983, 0.7072]))

for stride in (1, 4, 8, 14):
    m = GripperModel(stride=stride)
    pts = m.parts['gripper_link']
    T = m.solve(JOINTS_VAL)
    M = T['gripper_link']
    world = (M[:3, :3] @ pts.T).T + M[:3, 3]
    k = int(np.argmin(world[:, 2]))
    print(f'stride={stride:>3}  采样点 {len(pts):>6}  固定爪最低点 '
          f'({world[k,0]:+.4f},{world[k,1]:+.4f},{world[k,2]:+.4f})  '
          f'z={world[k,2]*1000:+.3f} mm')
