#!/usr/bin/env python3
"""用棋盘内角点的像素间距（=31.25mm，拖拽实测值）来量方块真实边长。

不依赖任何单应/标定，纯粹"方块像素尺寸 / 格子像素尺寸 × 31.25mm"。
"""
import cv2
import numpy as np

CELL_MM = 31.25          # 拖拽实测的棋盘格真实边长

cap = cv2.VideoCapture(0)
img = None
for _ in range(15):
    ok, f = cap.read()
    if ok:
        img = f
cap.release()
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ok, corners = cv2.findChessboardCorners(gray, (7, 5), None)
print('棋盘检测:', ok)
if not ok:
    raise SystemExit(1)
C = corners.reshape(5, 7, 2)             # 5行7列内角点
# 相邻内角点间距（横向 6 段、纵向 4 段），在方块附近（右侧）取样
hx = [np.linalg.norm(C[r, c + 1] - C[r, c]) for r in range(5)
      for c in range(6)]
hy = [np.linalg.norm(C[r + 1, c] - C[r, c]) for r in range(4)
      for c in range(7)]
print(f'横向格距(px): 中位 {np.median(hx):.2f}  范围 {min(hx):.2f}..{max(hx):.2f}')
print(f'纵向格距(px): 中位 {np.median(hy):.2f}  范围 {min(hy):.2f}..{max(hy):.2f}')
gx = float(np.median(hx))
gy = float(np.median(hy))

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, np.array((20, 90, 90)), np.array((34, 255, 255)))
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
print('\n黄色候选（用格子像素尺寸换算真实 mm）：')
for c in sorted(cnts, key=cv2.contourArea, reverse=True)[:5]:
    A = cv2.contourArea(c)
    if A < 300:
        continue
    x, y, w, h = cv2.boundingRect(c)
    # 该框中心到最近内角点的距离（判断是否在棋盘上）
    ctr = np.array([x + w / 2.0, y + h / 2.0])
    d = np.min(np.linalg.norm(C.reshape(-1, 2) - ctr, axis=1))
    print(f'  框({x},{y},{w},{h}) 面积{A:6.0f} 离最近内角点 {d:5.1f}px  ->  '
          f'{w/gx*CELL_MM:5.1f} x {h/gy*CELL_MM:5.1f} mm')
print(f'\n（格子像素尺寸 横 {gx:.2f}px 纵 {gy:.2f}px = {CELL_MM}mm）')
