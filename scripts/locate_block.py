#!/usr/bin/env python3
"""方块定位（带棋盘单应缓存）。

关键：方块压在棋盘上会挡住棋盘 -> 无法当帧检测。所以把单应缓存到
/tmp/vision_calib.json；相机不动时缓存长期有效，方块即使在棋盘上也能定位。

输出：每个颜色方块 -> 棋盘坐标(cm) -> base_link(m) -> 距底座水平半径(mm)
"""

from project_paths import calibration_path, default_camera
import json
import math
import os
import sys

import cv2
import numpy as np

CELL = 33.0
COLS, ROWS = 7, 5
FLAGS = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
CACHE = calibration_path('vision_calib.json')


def grab(n=15):
    cap = cv2.VideoCapture(default_camera())
    best = None
    for _ in range(n):
        ok, img = cap.read()
        if ok:
            best = img
    cap.release()
    return best


def detect_H(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, (COLS, ROWS), FLAGS)
    if not found:
        return None, None
    corners = cv2.cornerSubPix(
        gray, corners, (7, 7), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
    obj = np.zeros((COLS * ROWS, 2), np.float32)
    obj[:, 0] = np.tile(np.arange(COLS), ROWS) * CELL
    obj[:, 1] = np.repeat(np.arange(ROWS), COLS) * CELL
    H, _ = cv2.findHomography(corners.reshape(-1, 2).astype(np.float32), obj)
    c = corners.reshape(-1, 2)
    d = float(np.median(np.linalg.norm(c[1:] - c[:-1], axis=1)))
    return H, d


try:
    calib = json.load(open(CACHE))
except Exception:
    calib = {}
H = np.array(calib['H']) if calib.get('H') else None
affine = np.array(calib['affine']) if calib.get('affine') else None
if affine is None:
    MARKS = [((0.0, 0.0), (0.2915, 0.0296)), ((9.9, 0.0), (0.3050, -0.0688)),
             ((0.0, 6.6), (0.2260, 0.0168)), ((9.9, 6.6), (0.2323, -0.0710)),
             ((3.3, 3.3), (0.2582, -0.0086))]
    G = np.array([m[0] for m in MARKS], float) / 100.0
    B = np.array([m[1] for m in MARKS], float)
    affine, *_ = np.linalg.lstsq(
        np.hstack([G, np.ones((len(G), 1))]), B, rcond=None)

img = grab()
H2, px = detect_H(img)
if H2 is not None:
    H = H2
    calib.update(H=H.tolist(), affine=affine.tolist(), cell_mm=CELL,
                 cols=COLS, rows=ROWS)
    json.dump(calib, open(CACHE, 'w'), indent=2)
    print(f'棋盘识别 ✅ 单应已更新（角点间距 {px:.1f}px → {CELL/px:.3f} mm/px）')
else:
    print(f'棋盘未识别（被方块挡住？）→ 使用缓存单应 '
          f'{"✅ 有" if H is not None else "❌ 无！先移开方块重建缓存"}')
if H is None:
    raise SystemExit(1)


def to_base(gx_cm, gy_cm):
    return affine[:2].T @ np.array([gx_cm / 100.0, gy_cm / 100.0]) + affine[2]


hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
SPEC = {'blue': [((95, 90, 60), (135, 255, 255))],
        'green': [((35, 80, 60), (85, 255, 255))],
        'yellow': [((20, 90, 90), (34, 255, 255))],
        'red': [((0, 100, 70), (8, 255, 255)), ((170, 100, 70), (179, 255, 255))],
        'purple': [((136, 60, 60), (168, 255, 255))]}
print(f'\n{"颜色":>7}{"像素":>13}{"边长":>7}{"棋盘cm":>17}{"base(m)":>21}'
      f'{"半径mm":>8}  判定')
best = []
for name, rngs in SPEC.items():
    mask = None
    for lo_, hi_ in rngs:
        m = cv2.inRange(hsv, np.array(lo_), np.array(hi_))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in cnts:
        a = cv2.contourArea(cnt)
        if a < 350:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        ar = w / float(h) if h else 0
        if not 0.6 <= ar <= 1.7:          # 方块投影应接近方形
            continue
        cx, cy = x + w / 2, y + h / 2
        v = H @ np.array([cx, cy, 1.0])
        v /= v[2]
        gx, gy = v[0] / 10.0, v[1] / 10.0
        bx, by = to_base(gx, gy)
        rad = math.hypot(bx, by) * 1000
        tag = '可达' if rad < 380 else ('勉强' if rad < 450 else '超出范围')
        print(f'{name:>7}{f"({cx:.0f},{cy:.0f})":>13}{max(w,h):>6.0f}px'
              f'{f"({gx:+.2f},{gy:+.2f})":>17}{f"({bx:.3f},{by:.3f})":>21}'
              f'{rad:>8.0f}  {tag}')
        best.append((name, bx, by, rad, a))
print(f'\n共 {len(best)} 个方块（已过滤非方形/过小候选）')
