#!/usr/bin/env python3
"""量爪口开度曲线（给定夹爪角 -> 开口 mm），并估方块实际尺寸。"""

from project_paths import open_video_capture

from project_paths import default_camera, project_path
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.expanduser(
    project_path('qianli_ws/src/qianli_vision/scripts')))
from gripper_model import GripperModel

m = GripperModel(stride=8)
print('夹爪角(rad)  开口(mm)')
for g in (0.16, 0.30, 0.40, 0.52, 0.58, 0.70, 0.90, 1.10, 1.30, 1.50, 1.70):
    w = m.jaw_opening(g)
    print(f'  {g:5.2f}      {w*1000:7.1f}' if w else f'  {g:5.2f}      量不到')
print()

# 方块实际尺寸：用棋盘做尺度（同平面），按方块像素边长换算
CAL = os.path.expanduser(project_path('config/board_frame.json'))
import json
import math
frame = json.load(open(CAL))
H = np.array(frame['H'])
A = np.array(frame['affine'])

cap = open_video_capture(default_camera())
img = None
for _ in range(15):
    ok, f = cap.read()
    if ok:
        img = f
cap.release()
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, np.array((20, 90, 90)), np.array((34, 255, 255)))
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
print('黄色候选的"真实尺寸"估算（用棋盘单应把像素角点映射到棋盘mm）：')
for c in sorted(cnts, key=cv2.contourArea, reverse=True)[:5]:
    a = cv2.contourArea(c)
    if a < 300:
        continue
    x, y, w, h = cv2.boundingRect(c)
    pts = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]], float)
    v = (H @ np.hstack([pts, np.ones((4, 1))]).T).T
    g = v[:, :2] / v[:, 2:3]
    wmm = float(np.linalg.norm(g[1] - g[0]))
    hmm = float(np.linalg.norm(g[3] - g[0]))
    dg = g[1] - g[0]
    print(f'  像素({x},{y},{w},{h}) 面积{a:6.0f}  ->  棋盘 {wmm:5.1f} x {hmm:5.1f} mm')
