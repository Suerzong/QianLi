#!/usr/bin/env python3
"""验证 (0,0) 检测窗口：投影位置、参考掩码面积、当前重合度。"""
import json
import os

import cv2
import numpy as np

CFG = os.path.expanduser('~/QianLi/qianli_ws/config')
fr = json.load(open(os.path.join(CFG, 'board_frame.json')))
H = np.array(fr['H'])
Hi = np.linalg.inv(H)
v = Hi @ np.array([0.0, 0.0, 1.0])
PX = v[:2] / v[2]
print(f'棋盘 (0,0) 投影像素 = ({PX[0]:.0f}, {PX[1]:.0f})')

W = 70
cap = cv2.VideoCapture(0)
img = None
for _ in range(12):
    ok, f = cap.read()
    if ok:
        img = f
cap.release()
h, w = img.shape[:2]
x0, y0 = max(0, int(PX[0]) - W), max(0, int(PX[1]) - W)
x1, y1 = min(w, int(PX[0]) + W), min(h, int(PX[1]) + W)
win = img[y0:y1, x0:x1]
print(f'窗口 = 像素 x[{x0},{x1}) y[{y0},{y1})  尺寸 {win.shape[1]}x{win.shape[0]}')
hsv = cv2.cvtColor(win, cv2.COLOR_BGR2HSV)
m = cv2.inRange(hsv, np.array((20, 90, 90)), np.array((34, 255, 255)))
m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
print(f'当前窗口内黄掩码面积 = {cv2.countNonZero(m)} px')
ref = cv2.imread(os.path.join(CFG, 'at00_mask.png'), cv2.IMREAD_GRAYSCALE)
if ref is None:
    print('参考掩码不存在')
else:
    print(f'参考掩码面积 = {cv2.countNonZero(ref)} px  (shape {ref.shape})')
    if ref.shape == m.shape:
        inter = cv2.countNonZero(cv2.bitwise_and(m, ref))
        ra = max(1, cv2.countNonZero(ref))
        print(f'重合 = {inter} px  = 参考面积的 {inter/ra*100:.1f}%')
cv2.imwrite('/tmp/at00_window.jpg', win)
print('窗口截图 -> /tmp/at00_window.jpg')
