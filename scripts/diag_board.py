#!/usr/bin/env python3
"""检测当前画面里棋盘的位置 vs 标定值。"""

from project_paths import default_camera
import os

import cv2
import numpy as np

cap = cv2.VideoCapture(default_camera())
for _ in range(8):
    ok, f = cap.read()
cap.release()
gray = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
# 找棋盘格角点
found, corners = cv2.findChessboardCorners(gray, (7, 5), None)
if found:
    print(f'棋盘格角点找到: 首 {corners[0][0].round(1)} 末 {corners[-1][0].round(1)}')
    # 首角=棋盘(0,0)?? 取决于方向；打印四个角
    xs = corners[:, 0, 0]
    print('x范围: %.0f..%.0f  y范围: %.0f..%.0f'
          % (xs[:, 0].min(), xs[:, 0].max(), xs[:, 1].min(), xs[:, 1].max()))
else:
    print('棋盘格角点未找到（可能被遮挡/不在画面/格子数不符）')
# 标定时的像素范围
print('标定 pixel_extent: [284, 476, 236, 368]')
# 黄色方块位置（复测）
hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
m = cv2.inRange(hsv, np.array((20, 90, 90)), np.array((34, 255, 255)))
cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
if cnts:
    big = max(cnts, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(big)
    print(f'黄色方块 bbox=({x},{y}) {w}x{h}px 中心=({x+w//2},{y+h//2})')
