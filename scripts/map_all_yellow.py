#!/usr/bin/env python3
"""把当前画面所有大黄色块映射到棋盘坐标，判断哪个在棋盘上。"""

from project_paths import open_video_capture

from project_paths import default_camera, project_path
import os

import cv2
import json
import numpy as np

fr = json.load(open(os.path.expanduser(
    project_path('config/board_frame.json'))))
H = np.array(fr['H'])
cols, rows = fr['cols'], fr['rows']
cell = fr['cell_mm']
board_x = cols * cell
board_y = rows * cell
print(f'棋盘范围 mm: x 0..{board_x:.0f}  y 0..{board_y:.0f} '
      f'({cols}x{rows}格, {cell}mm)')

cap = open_video_capture(default_camera())
for _ in range(20):
    ok, f = cap.read()
ok, f = cap.read()
cap.release()
hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
m = cv2.inRange(hsv, np.array((15, 60, 60)), np.array((40, 255, 255)))
m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
print(f'黄色块 {len(cnts)} 个（area>800 过滤后）：')
for c in sorted(cnts, key=cv2.contourArea, reverse=True):
    a = cv2.contourArea(c)
    if a < 800:
        continue
    x, y, w, h = cv2.boundingRect(c)
    cx, cy = x + w / 2, y + h / 2
    v = H @ np.array([cx, cy, 1.0])
    bx, by = v[0] / v[2], v[1] / v[2]
    onb = '棋盘上 ✅' if (0 <= bx <= board_x and 0 <= by <= board_y) \
        else '棋盘外 ⚠'
    print(f'  像素({cx:.0f},{cy:.0f}) {w}x{h}px area={a:.0f} '
          f'-> 棋盘mm({bx:.1f},{by:.1f}) {onb}')
