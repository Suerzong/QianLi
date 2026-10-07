#!/usr/bin/env python3
"""抗遮挡的棋盘外框检测 + 方块定位。

思路：棋盘区域"局部方差"高（黑白格交替），纸边/桌面方差低。即便中间被
方块压住，外框仍是完整四边形。由外框 4 角建立 像素↔棋盘坐标 单应，
再把方块中心映射成棋盘坐标（cm）。
"""

from project_paths import open_video_capture

from project_paths import default_camera
import json

import cv2
import numpy as np

CELL = 33.0                      # 格边长 mm
COLS_SQ, ROWS_SQ = 8, 6          # 方格数（= 7x5 内角点）
INNER_X, INNER_Y = COLS_SQ - 1, ROWS_SQ - 1


def pattern_extent_mm():
    """棋盘图案外框相对"第一个内角点"的坐标 (mm)。"""
    h = CELL / 2
    return np.array([[-h, -h],
                     [INNER_X * CELL + h, -h],
                     [INNER_X * CELL + h, INNER_Y * CELL + h],
                     [-h, INNER_Y * CELL + h]], float)


def detect_board(gray):
    """返回图案外框 4 角（像素，顺时针）。"""
    f = gray.astype(np.float32)
    k = 9
    mean = cv2.blur(f, (k, k))
    sq = cv2.blur(f * f, (k, k))
    var = np.clip(sq - mean * mean, 0, None)
    v = cv2.normalize(var, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    _, th = cv2.threshold(v, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
    cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, th
    c = max(cnts, key=cv2.contourArea)
    peri = cv2.arcLength(c, True)
    for eps in (0.02, 0.03, 0.05, 0.08):
        ap = cv2.approxPolyDP(c, eps * peri, True)
        if len(ap) == 4:
            pts = ap.reshape(4, 2).astype(float)
            s = pts.sum(axis=1)
            d = np.diff(pts, axis=1).ravel()
            return np.array([pts[np.argmin(s)], pts[np.argmin(d)],
                             pts[np.argmax(s)], pts[np.argmax(d)]]), th
    return None, th


cap = open_video_capture(default_camera())
for _ in range(10):
    ok, img = cap.read()
    if ok:
        break
cap.release()
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
quad, mask = detect_board(gray)
print('棋盘外框(像素):')
if quad is None:
    print('  未检测到四边形')
else:
    for p in quad:
        print(f'  ({p[0]:.0f}, {p[1]:.0f})')

# 找方块：蓝色 / 灰色 都试
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
blobs = {}
for name, lo, hi in (('blue', (95, 90, 60), (135, 255, 255)),
                     ('gray', (0, 0, 90), (179, 90, 190)),
                     ('green', (35, 80, 60), (85, 255, 255)),
                     ('yellow', (20, 90, 90), (34, 255, 255)),
                     ('red1', (0, 100, 70), (8, 255, 255)),
                     ('red2', (170, 100, 70), (179, 255, 255))):
    m = cv2.inRange(hsv, np.array(lo), np.array(hi))
    if name.startswith('red'):
        blobs.setdefault('red', []).append(m)
        continue
    blobs[name] = [m]
print('\n方块候选:')
found = {}
for name, ms in blobs.items():
    m = ms[0] if len(ms) == 1 else cv2.bitwise_or(ms[0], ms[1])
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
        a = cv2.contourArea(c)
        if a < 250:
            continue
        x, y, w, h = cv2.boundingRect(c)
        print(f'  {name:>6}: 面积={a:6.0f} 中心=({x+w//2},{y+h//2}) '
              f'{w}x{h}px')
        found.setdefault(name, []).append(((x + w / 2), (y + h / 2), a, w, h))

if quad is not None and found:
    src = quad.astype(np.float32)
    dst = pattern_extent_mm().astype(np.float32)
    H = cv2.getPerspectiveTransform(src, dst)
    print('\n像素 -> 棋盘坐标 (mm)，以外框左上为 (-16.5,-16.5)：')
    for name, items in found.items():
        for (cx, cy, a, w, h) in items:
            vec = H @ np.array([cx, cy, 1.0])
            vec /= vec[2]
            print(f'  {name:>6} 像素({cx:.0f},{cy:.0f}) -> '
                  f'棋盘({vec[0]:+7.1f}, {vec[1]:+7.1f}) mm  '
                  f'= ({vec[0]/10:+.2f}, {vec[1]/10:+.2f}) cm  '
                  f'[内角点系 {vec[0]/CELL:+.2f}, {vec[1]/CELL:+.2f} 格]')
