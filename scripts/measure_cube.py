#!/usr/bin/env python3
"""用棋盘单应精确量方块的实际边长（mm）。"""

from project_paths import default_camera, project_path
import json
import os

import cv2
import numpy as np

CFG = os.path.expanduser(project_path('config'))
fr = json.load(open(os.path.join(CFG, 'board_frame.json')))
H = np.array(fr['H'])
Hi = np.linalg.inv(H)

cap = cv2.VideoCapture(default_camera())
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
print('黄色连通域（用棋盘单应换算成真实 mm）：')
for c in sorted(cnts, key=cv2.contourArea, reverse=True)[:6]:
    A = cv2.contourArea(c)
    if A < 300:
        continue
    x, y, w, h = cv2.boundingRect(c)
    pts = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]], float)
    v = (Hi @ np.hstack([pts, np.ones((4, 1))]).T).T
    g = v[:, :2] / v[:, 2:3] / 10.0        # 棋盘 cm
    wcm = float(np.linalg.norm(g[1] - g[0]))
    hcm = float(np.linalg.norm(g[3] - g[0]))
    print(f'  像素({x:3d},{y:3d},{w:3d},{h:3d}) 面积{A:6.0f} -> '
          f'棋盘 {wcm*10:5.1f} x {hcm*10:5.1f} mm  '
          f'(中心 棋盘 {g.mean(0)[0]:+.2f},{g.mean(0)[1]:+.2f} cm)')
print('\n注：棋盘格真实边长 31.25mm（拖拽实测）。'
      '若方块量出 ~40mm 则与假设一致。')
