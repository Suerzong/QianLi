#!/usr/bin/env python3
"""调试：打印方块尺寸采样时每个候选的计算值。"""

from project_paths import default_camera, project_path
import json
import os

import cv2
import numpy as np

CFG = os.path.expanduser(project_path('config'))
H = np.array(json.load(open(os.path.join(CFG, 'board_frame.json')))['H'])


def g_of(p):
    v = H @ np.array([p[0], p[1], 1.0])
    return v[:2] / v[2] / 10.0


cap = cv2.VideoCapture(default_camera())
ok, img = cap.read()
cap.release()
print('读帧', ok, None if img is None else img.shape)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, np.array((20, 90, 90)), np.array((34, 255, 255)))
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
print(f'{len(cnts)} 个连通域')
for c in sorted(cnts, key=cv2.contourArea, reverse=True)[:8]:
    A = cv2.contourArea(c)
    x, y, w, h = cv2.boundingRect(c)
    cx, cy = x + w / 2.0, y + h / 2.0
    g = g_of((cx, cy))
    c5 = np.array([cx, cy])
    sx = np.linalg.norm(g_of(c5 + [5, 0]) - g_of(c5 - [5, 0])) / 10.0
    sy = np.linalg.norm(g_of(c5 + [0, 5]) - g_of(c5 - [0, 5])) / 10.0
    span = max(w, h)
    sz = span * (sx + sy) / 2 * 10.0
    print(f'  面积{A:6.0f} 框({x},{y},{w},{h}) 棋盘cm({g[0]:+.2f},{g[1]:+.2f}) '
          f'尺度{(sx+sy)/2*10:.3f}mm/px span={span} -> {sz:.1f}mm  '
          f'宽高比{w/max(h,1):.2f}')
