#!/usr/bin/env python3
"""抗遮挡棋盘单应：用黑方格外轮廓四点建 像素<->棋盘 单应。

方块压在棋盘上时 findChessboardCorners 会失败（它要求全部内角点可见）。
但黑色方格有 48 个，只被挡掉 1~2 个，其**外轮廓**仍完整 -> 4 角可用。

外轮廓四角对应的棋盘坐标（格边长 CELL，原点=第一个内角点）：
  P1 (-h,-h)  P2 (6*CELL+h, -h)  P3 (7*CELL+h, 5*CELL+h)  P4 (CELL+h, 5*CELL+h)
  h = CELL/2
并做校验：把所有黑方格质心映过去，应落在格心点阵上。
"""

from project_paths import open_video_capture

from project_paths import calibration_path, default_camera
import json
import sys

import cv2
import numpy as np

CELL = 33.0
h = CELL / 2.0
COLS, ROWS = 7, 5


def square_lattice_score(H, cents):
    """黑方格质心映到棋盘坐标后，离"格心点阵"的平均距离。"""
    if len(cents) == 0:
        return 1e9, 0
    d = []
    for (cx, cy) in cents:
        v = H @ np.array([cx, cy, 1.0])
        v = v[:2] / v[2]
        gx, gy = v
        # 格心在 (k*CELL - h, m*CELL - h)
        ex = (gx + h) / CELL
        ey = (gy + h) / CELL
        d.append(np.hypot((ex - round(ex)) * CELL, (ey - round(ey)) * CELL))
    d = np.array(d)
    return float(np.median(d)), int((d < 8).sum())


def main():
    cap = open_video_capture(default_camera())
    img = None
    for _ in range(20):
        ok, f = cap.read()
        if ok:
            img = f
    cap.release()
    if img is None:
        print('取帧失败')
        return 1
    cv2.imwrite('/tmp/board_src.jpg', img)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 黑方格：低亮度阈值
    for thr in (60, 80, 100, 120):
        mask = (gray < thr).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        cents, pts_all = [], []
        for c in cnts:
            a = cv2.contourArea(c)
            if a < 200 or a > 4000:
                continue
            (_, _), (w, hh), _ = cv2.minAreaRect(c)
            if w == 0 or hh == 0:
                continue
            if not (0.55 <= min(w, hh) / max(w, hh) <= 1.8):
                continue
            if a / (w * hh) < 0.6:
                continue
            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            cents.append((M['m10'] / M['m00'], M['m01'] / M['m00']))
            pts_all.append(c.reshape(-1, 2))
        if len(cents) < 12:
            continue
        allpts = np.vstack(pts_all).astype(np.float32)
        hull = cv2.convexHull(allpts)
        peri = cv2.arcLength(hull, True)
        quad = None
        for eps in (0.01, 0.02, 0.03, 0.05):
            ap = cv2.approxPolyDP(hull, eps * peri, True)
            if len(ap) == 4:
                quad = ap.reshape(4, 2).astype(np.float32)
                break
        if quad is None:
            continue
        # 排序：左上、右上、右下、左下
        s = quad.sum(1)
        d = np.diff(quad, axis=1).ravel()
        ordered = np.array([quad[np.argmin(s)], quad[np.argmin(d)],
                            quad[np.argmax(s)], quad[np.argmax(d)]], np.float32)
        dst = np.array([[-h, -h], [6 * CELL + h, -h],
                        [7 * CELL + h, 5 * CELL + h],
                        [CELL + h, 5 * CELL + h]], np.float32)
        H = cv2.getPerspectiveTransform(ordered, dst)
        med, inl = square_lattice_score(H, cents)
        print(f'thr={thr}: 黑方格 {len(cents)} 个  点阵中位误差 {med:.2f}mm  '
              f'内点 {inl}/{len(cents)}')
        if inl >= len(cents) - 2 and med < 8:
            print('  ✅ 单应可信')
            for p in ordered:
                print(f'    外轮廓角点 ({p[0]:.0f},{p[1]:.0f})')
            json.dump({'H': H.tolist(), 'cell_mm': CELL,
                       'src_quad': ordered.tolist(),
                       'lattice_med_mm': med, 'inliers': inl,
                       'n_squares': len(cents)},
                      open(calibration_path('board_homography.json'), 'w'), indent=2)
            print('  已写 /tmp/board_homography.json')
            return 0
    print('❌ 未找到可信单应')
    return 1


if __name__ == '__main__':
    sys.exit(main())
