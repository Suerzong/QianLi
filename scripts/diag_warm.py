#!/usr/bin/env python3
"""仔细诊断：预热后单帧全面分析（棋盘暗格分布 + 宽范围黄色 + 亮度）。"""
import cv2
import numpy as np

cap = cv2.VideoCapture(0)
for _ in range(25):          # 预热，让自动曝光/白平衡稳定
    ok, f = cap.read()
frames = []
for _ in range(5):
    ok, f = cap.read()
    if ok:
        frames.append(f)
cap.release()
f = frames[-1]
cv2.imwrite('/tmp/diag_warm.jpg', f)
gray = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
hsv = cv2.cvtColor(f, cv2.COLOR_HSV2BGR if False else cv2.COLOR_BGR2HSV)
print(f'帧 {f.shape} 亮度mean={f.mean():.1f} std={f.std():.1f}')

# 棋盘暗格：二值化找暗区域
_, dark = cv2.threshold(gray, 90, 255, cv2.THRESH_BINARY_INV)
dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
cnts, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
if cnts:
    for c in sorted(cnts, key=cv2.contourArea, reverse=True)[:6]:
        x, y, w, h = cv2.boundingRect(c)
        print(f'暗区 bbox=({x},{y}) {w}x{h}px area={cv2.contourArea(c):.0f}')
else:
    print('无暗区')

# 黄色：宽范围 (15,60,60)-(40,255,255)
m = cv2.inRange(hsv, np.array((15, 60, 60)), np.array((40, 255, 255)))
m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
print(f'黄色(宽范围) 轮廓 {len(cnts)}:')
for c in sorted(cnts, key=cv2.contourArea, reverse=True)[:6]:
    x, y, w, h = cv2.boundingRect(c)
    print(f'  bbox=({x},{y}) {w}x{h}px area={cv2.contourArea(c):.0f}')

# 中值亮度（排除极端）
print(f'中值亮度={np.median(f):.1f}  HSV-H均值={hsv[:,:,0].mean():.1f} '
      f'H中位数={np.median(hsv[:,:,0]):.1f}')
