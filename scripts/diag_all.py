#!/usr/bin/env python3
"""全面诊断：连续多帧的黄色分布 + 棋盘多尺寸检测 + 帧间差异。"""

from project_paths import default_camera
import cv2
import numpy as np

cap = cv2.VideoCapture(default_camera())
frames = []
for _ in range(15):
    ok, f = cap.read()
    if ok:
        frames.append(f)
cap.release()
print(f'取到 {len(frames)} 帧')
f0 = frames[0]
# 帧间差异（视图是否稳定）
for i in (1, 5, 10, 14):
    d = cv2.absdiff(f0, frames[i]).mean()
    print(f'帧0 vs 帧{i} 平均差异: {d:.2f}')

# 所有黄色轮廓
hsv = cv2.cvtColor(f0, cv2.COLOR_BGR2HSV)
m = cv2.inRange(hsv, np.array((20, 90, 90)), np.array((34, 255, 255)))
m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
print(f'黄色轮廓 {len(cnts)} 个:')
for c in sorted(cnts, key=cv2.contourArea, reverse=True)[:6]:
    x, y, w, h = cv2.boundingRect(c)
    print(f'  bbox=({x},{y}) {w}x{h}px area={cv2.contourArea(c):.0f}')
cv2.imwrite('/tmp/diag_all.jpg', f0)

# 棋盘多尺寸
gray = cv2.cvtColor(f0, cv2.COLOR_BGR2GRAY)
for (c, r) in [(7, 5), (6, 5), (7, 4), (6, 4), (8, 5)]:
    found, corners = cv2.findChessboardCorners(gray, (c, r), None)
    if found:
        xs = corners[:, 0, 0]
        print(f'棋盘 {c}x{r} 找到: x {xs[:,0].min():.0f}..{xs[:,0].max():.0f} '
              f'y {xs[:,1].min():.0f}..{xs[:,1].max():.0f}')
    else:
        print(f'棋盘 {c}x{r} 未找到')
