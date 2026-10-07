#!/usr/bin/env python3
"""把相机画面文字化（亮度网格）+ 多尺寸棋盘检测 + 灰色物块候选。

用于在无法看图的情况下判断场景：棋盘在不在画面里、物块大致在哪。
"""

from project_paths import open_video_capture
import cv2
import numpy as np

FLAGS = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)

RAMP = ' .:-=+*#%@'


def textify(gray, cols=48, rows=20):
    small = cv2.resize(gray, (cols, rows), interpolation=cv2.INTER_AREA)
    lo, hi = np.percentile(small, 2), np.percentile(small, 98)
    norm = np.clip((small - lo) / max(hi - lo, 1e-6), 0, 1)
    out = []
    for r in range(rows):
        out.append(''.join(RAMP[int(v * (len(RAMP) - 1))] for v in norm[r]))
    return '\n'.join(out)


for idx in (0, 1):
    cap = open_video_capture(idx)
    if not cap.isOpened():
        continue
    ok, img = False, None
    for _ in range(10):
        ok, img = cap.read()
        if ok:
            break
    cap.release()
    if not ok:
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    print('=' * 60)
    print(f'cam{idx}  {img.shape[1]}x{img.shape[0]}')
    print(textify(gray))
    print('--- 棋盘检测 ---')
    for size in [(7, 5), (8, 6), (6, 4), (9, 6), (5, 4), (7, 6), (6, 5)]:
        found, _ = cv2.findChessboardCorners(gray, size, FLAGS)
        if found:
            print(f'  {size} 识别到 ✅')
    print('--- 灰色物块候选（HSV 灰带 + 面积）---')
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 0, 90), (179, 90, 190))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                            np.ones((5, 5), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                               cv2.CHAIN_APPROX_SIMPLE)
    cands = []
    for c in cnts:
        a = cv2.contourArea(c)
        if 100 < a < 20000:
            x, y, w, h = cv2.boundingRect(c)
            cands.append((a, x, y, w, h))
    cands.sort(reverse=True)
    for a, x, y, w, h in cands[:6]:
        print(f'  面积={a:7.0f} bbox=({x:3d},{y:3d},{w:3d},{h:3d}) '
              f'中心=({x+w//2},{y+h//2}) 边长≈{max(w,h)}px')
    if not cands:
        print('  无候选')
