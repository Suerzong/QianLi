#!/usr/bin/env python3
"""对比：示范抓取位的工具姿态 vs 我算的目标姿态。"""
import json
import math
import os
import sys

import numpy as np
import yaml
from pathlib import Path

sys.path.insert(0, os.path.expanduser(
    '~/QianLi/qianli_ws/src/qianli_vision/scripts'))
from gripper_model import GripperModel, JOINTS

CFG = os.path.expanduser('~/QianLi/qianli_ws/config')
model = GripperModel(stride=8)

tg = json.load(open(os.path.join(CFG, 'taught_grasp.json')))
q_t = np.array(tg['q_rad'])
T = model.solve(dict(zip(JOINTS, q_t)))
F = T['gripper_frame_link']
zt = F[:3, :3] @ np.array([0.0, 0.0, 1.0])
xt = F[:3, :3] @ np.array([1.0, 0.0, 0.0])
yt = F[:3, :3] @ np.array([0.0, 1.0, 0.0])
print('示范抓取位:')
print(f'  TCP        ({F[0,3]:.4f},{F[1,3]:.4f},{F[2,3]*1000:+.1f}mm)')
print(f'  工具 z 轴  {np.round(zt,4).tolist()}  '
      f'偏离竖直 {math.degrees(math.acos(abs(zt[2]))):.1f}°  '
      f'({"朝下" if zt[2] < 0 else "朝上"})')
print(f'  工具 x 轴  {np.round(xt,4).tolist()}  '
      f'水平投影角 {math.degrees(math.atan2(xt[1], xt[0])):+.1f}°')
print(f'  工具 y 轴  {np.round(yt,4).tolist()}')
print(f'  爪口开度   {model.jaw_opening(float(q_t[5]))*1000:.1f} mm')

fr = json.load(open(os.path.join(CFG, 'board_frame.json')))
A = np.array(fr['affine'])
ex = A[:, 0] / np.linalg.norm(A[:, 0])
ey = A[:, 1] / np.linalg.norm(A[:, 1])
print(f'\n棋盘轴(实测):  ex {np.round(ex,4).tolist()}   '
      f'ey {np.round(ey,4).tolist()}')

for name, v in (('ex(棋盘x)', ex), ('ey(棋盘y)', ey),
                ('-ex', -ex), ('-ey', -ey)):
    a = math.degrees(math.acos(np.clip(np.dot(v[:2], xt[:2])
                                       / (np.linalg.norm(v[:2])
                                          * np.linalg.norm(xt[:2])), -1, 1)))
    print(f'  工具x轴 与 {name} 夹角 {a:6.1f}°')

for name, v in ((('竖直向下'), np.array([0, 0, -1.0])),):
    a = math.degrees(math.acos(np.clip(np.dot(v, zt), -1, 1)))
    print(f'  工具z轴 与 竖直向下 夹角 {a:.1f}°')
print(f'\n→ 我此前假设"工具z朝下、工具x沿 ey"；若上表夹角很大，就是姿态假设错了')
