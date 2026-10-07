#!/usr/bin/env python3
"""多帧采样，取方块边长的中位数（抑制单帧抖动）。

用棋盘单应在像素处的局部尺度换算，排除臂上黄件/工具（只取棋盘上、尺寸合理者）。
"""

from project_paths import default_camera, project_path
import json
import os
import statistics as st
import time

import cv2
import numpy as np

CFG = os.path.expanduser(project_path('config'))
fr = json.load(open(os.path.join(CFG, 'board_frame.json')))
H = np.array(fr['H'])          # H: 像素 -> 棋盘 mm（board_frame 里就是这么用的）


def g_of(p):
    v = H @ np.array([p[0], p[1], 1.0])
    return v[:2] / v[2] / 10.0          # 棋盘 cm


def size_mm(px, span):
    c = np.array(px, float)
    sx = np.linalg.norm(g_of(c + [5, 0]) - g_of(c - [5, 0])) / 10.0
    sy = np.linalg.norm(g_of(c + [0, 5]) - g_of(c - [0, 5])) / 10.0
    return span * (sx + sy) / 2 * 10.0


cap = cv2.VideoCapture(default_camera())
samples = []
for _ in range(10):
    ok, img = cap.read()
    if not ok:
        continue
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array((20, 90, 90)), np.array((34, 255, 255)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for c in cnts:
        A = cv2.contourArea(c)
        if A < 350:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if not 0.6 <= w / float(max(h, 1)) <= 1.7:
            continue
        cx, cy = x + w / 2.0, y + h / 2.0
        g = g_of((cx, cy))
        if not (-2 <= g[0] <= 22 and -2 <= g[1] <= 16):   # 必须在棋盘附近
            continue
        sz = size_mm((cx, cy), max(w, h))
        if not 20 <= sz <= 50:
            continue
        if best is None or sz < best[2]:
            best = (cx, cy, sz)
    if best:
        samples.append(best[2])
    time.sleep(0.12)
cap.release()
if not samples:
    print('NONE')
    raise SystemExit(1)
med = st.median(samples)
print(f'{med:.1f} # n={len(samples)} 样本={[round(s,1) for s in samples]}')
