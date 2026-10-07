#!/usr/bin/env python3
"""诊断：相机看到什么 + 黄色掩码在哪。"""

from project_paths import open_video_capture

from project_paths import default_camera
import cv2
import numpy as np

cap = open_video_capture(default_camera())
for _ in range(8):
    ok, f = cap.read()
cap.release()
if not ok:
    print('READ FAIL')
    raise SystemExit(1)
cv2.imwrite('/tmp/diag_frame.jpg', f)
hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
m = cv2.inRange(hsv, np.array((20, 90, 90)), np.array((34, 255, 255)))
m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
n = cv2.countNonZero(m)
print(f'frame {f.shape} 黄色掩码像素 {n}')
# 掩码中最大的连通域
cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
big = max(cnts, key=cv2.contourArea) if cnts else None
if big is not None:
    x, y, w, h = cv2.boundingRect(big)
    print(f'最大轮廓: bbox=({x},{y}) {w}x{h}px area={cv2.contourArea(big)}')
cv2.imwrite('/tmp/diag_mask.jpg', m)
# 全图亮度统计（判断光照是否变了）
print(f'亮度 mean={f.mean():.1f}  HSV-H均值={hsv[:,:,0].mean():.1f}')
