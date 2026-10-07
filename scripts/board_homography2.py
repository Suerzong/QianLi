#!/usr/bin/env python3
"""抗遮挡棋盘单应（穷举相位）。

findChessboardCorners 要求全部内角点可见，方块压上去就失败。
改用黑方格(48 个, 只被挡 1~2 个)的外轮廓四点建单应；
外轮廓对应的棋盘坐标有相位歧义 -> 穷举 16 种组合，取点阵误差最小的。
"""

from project_paths import open_video_capture

from project_paths import calibration_path, default_camera
import itertools
import json
import sys

import cv2
import numpy as np

CELL = 33.0
h = CELL / 2.0
COLS, ROWS = 7, 5
NX, NY = COLS - 1, ROWS - 1          # 6 x 4 格


def squares(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cents, pts = [], []
    for thr in (60, 75, 90, 105):
        mask = (gray < thr).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        c2, p2 = [], []
        for c in cnts:
            a = cv2.contourArea(c)
            if a < 150 or a > 5000:
                continue
            (_, _), (w, hh), _ = cv2.minAreaRect(c)
            if w < 5 or hh < 5:
                continue
            if not (0.5 <= min(w, hh) / max(w, hh) <= 2.0):
                continue
            if a / (w * hh) < 0.55:
                continue
            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            c2.append((M['m10'] / M['m00'], M['m01'] / M['m00']))
            p2.append(c.reshape(-1, 2))
        if len(c2) > len(cents):
            cents, pts = c2, p2
    return cents, pts


def hull_quad(pts):
    allp = np.vstack(pts).astype(np.float32)
    hull = cv2.convexHull(allp)
    peri = cv2.arcLength(hull, True)
    for eps in (0.01, 0.02, 0.03, 0.05, 0.08):
        ap = cv2.approxPolyDP(hull, eps * peri, True)
        if len(ap) == 4:
            q = ap.reshape(4, 2).astype(np.float32)
            s = q.sum(1)
            d = np.diff(q, axis=1).ravel()
            return np.array([q[np.argmin(s)], q[np.argmin(d)],
                             q[np.argmax(s)], q[np.argmax(d)]], np.float32)
    return None


def score(H, cents):
    d = []
    for (cx, cy) in cents:
        v = H @ np.array([cx, cy, 1.0])
        v = v[:2] / v[2]
        ex = (v[0] + h) / CELL
        ey = (v[1] + h) / CELL
        d.append(np.hypot((ex - round(ex)) * CELL, (ey - round(ey)) * CELL))
    d = np.array(d)
    return float(np.median(d)), int((d < 6).sum())


def main():
    img = cv2.imread('/tmp/board_src.jpg')
    if img is None:
        cap = open_video_capture(default_camera())
        for _ in range(20):
            ok, f = cap.read()
            if ok:
                img = f
        cap.release()
        cv2.imwrite('/tmp/board_src.jpg', img)
    cents, pts = squares(img)
    print(f'黑方格候选: {len(cents)} 个（8x6 棋盘应有 24 个黑格）')
    if len(cents) < 12:
        print('候选太少，无法建单应')
        return 1
    quad = hull_quad(pts)
    if quad is None:
        print('外轮廓不是四边形')
        return 1
    print('外轮廓四点:', [(round(float(p[0])), round(float(p[1]))) for p in quad])

    # 外轮廓四角在棋盘坐标里的可能取值（黑格角点相对内角点系的坐标）
    cand_lo = [-h, CELL + h]
    cand_hi_x = [NX * CELL - h, NX * CELL + h]
    cand_hi_y = [NY * CELL - h, NY * CELL + h]
    best = None
    for x0, x1, y0, y1 in itertools.product(cand_lo, cand_hi_x,
                                            cand_lo, cand_hi_y):
        dst = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], np.float32)
        H = cv2.getPerspectiveTransform(quad, dst)
        med, inl = score(H, cents)
        if best is None or (inl, -med) > (best[0], -best[1]):
            best = (inl, med, H, (x0, x1, y0, y1))
    inl, med, H, box = best
    print(f'最佳相位: x {box[0]:+.1f}..{box[1]:+.1f}  y {box[2]:+.1f}..{box[3]:+.1f}')
    print(f'  点阵中位误差 {med:.2f}mm  内点 {inl}/{len(cents)}')
    if med < 6 and inl >= len(cents) - 2:
        json.dump({'H': H.tolist(), 'cell_mm': CELL,
                   'quad': quad.tolist(), 'box': list(box),
                   'lattice_med_mm': med, 'inliers': inl,
                   'n_squares': len(cents)},
                  open(calibration_path('board_homography.json'), 'w'), indent=2)
        print('✅ 已写 /tmp/board_homography.json')
        return 0
    print('❌ 点阵拟合不够好')
    return 1


if __name__ == '__main__':
    sys.exit(main())
