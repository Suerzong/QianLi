#!/usr/bin/env python3
"""算可达范围：采样随机关节配置，统计 TCP 能达到的区域。

用于判断"方块放哪才抓得到"。
"""

from project_paths import driver_params_path, project_path
import math
import os
import sys

import numpy as np
import yaml
from pathlib import Path

sys.path.insert(0, os.path.expanduser(
    project_path('qianli_ws/src/qianli_vision/scripts')))
from gripper_model import GripperModel, JOINTS

CONFIG = os.path.expanduser(
    driver_params_path())
cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
lo = (np.array(cfg['raw_min']) - zero) * direction * 2 * math.pi / 4096
hi = (np.array(cfg['raw_max']) - zero) * direction * 2 * math.pi / 4096
lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)

model = GripperModel(stride=40)
rng = np.random.default_rng(0)
N = 60000
q = lo[:5] + rng.random((N, 5)) * (hi[:5] - lo[:5])

pts = []
for i in range(0, N, 800):
    for row in q[i:i + 800]:
        T = model.solve(dict(zip(JOINTS, np.r_[row, 0.5])))
        pts.append(T['gripper_frame_link'][:3, 3])
P = np.array(pts)
r = np.linalg.norm(P[:, :2], axis=1)
print(f'采样 {len(P)} 个可达 TCP 点')
print(f'水平半径: 最小 {r.min()*1000:.0f}mm  最大 {r.max()*1000:.0f}mm')
print(f'x 范围 {(P[:,0].min())*1000:+.0f} .. {(P[:,0].max())*1000:+.0f} mm')
print(f'y 范围 {(P[:,1].min())*1000:+.0f} .. {(P[:,1].max())*1000:+.0f} mm')
print(f'z 范围 {(P[:,2].min())*1000:+.0f} .. {(P[:,2].max())*1000:+.0f} mm')

# 桌面附近（z=-49mm，方块中心高度）可达的水平范围
sel = np.abs(P[:, 2] + 0.049) < 0.02
if sel.sum() > 20:
    Q = P[sel]
    rr = np.linalg.norm(Q[:, :2], axis=1)
    print(f'\n在 z≈-49mm（方块中心高度）可达的点: {sel.sum()} 个')
    print(f'  水平半径 {rr.min()*1000:.0f}..{rr.max()*1000:.0f} mm')
    print(f'  x {Q[:,0].min()*1000:+.0f}..{Q[:,0].max()*1000:+.0f} mm  '
          f'y {Q[:,1].min()*1000:+.0f}..{Q[:,1].max()*1000:+.0f} mm')
    ang = np.degrees(np.arctan2(Q[:, 1], Q[:, 0]))
    print(f'  方位角 {ang.min():.0f}..{ang.max():.0f} deg')
else:
    print('\nz≈-49mm 附近可达点太少')

print('\n判据: 方块 base 位置若水平半径 > 上表上限，就够不到。')
for name, x, y in [('蓝(移动后)', 0.399, -0.002), ('绿', 0.469, -0.075),
                   ('黄', 0.425, -0.071), ('红', 0.431, -0.124),
                   ('棋盘中心', 0.258, -0.009)]:
    print(f'  {name:>10}: 半径 {math.hypot(x, y)*1000:5.0f} mm')
