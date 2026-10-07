#!/usr/bin/env python3
"""抓一帧原始图 + 棋盘角点检测结果，用来目视评估畸变"""

from __future__ import annotations

from project_paths import default_camera

import sys
import time

import cv2
import numpy as np

COLS, ROWS = 7, 5          # 内角点
CELL_CM = 3.3


def main():
    cap = cv2.VideoCapture(default_camera())
    if not cap.isOpened():
        print('❌ 打不开 /dev/video0')
        return 1
    print(f'默认分辨率 {cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x'
          f'{cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f}')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    time.sleep(1.0)
    for _ in range(10):
        cap.read()
    ok, img = cap.read()
    if not ok:
        print('❌ 读帧失败')
        return 1
    print(f'实际分辨率 {img.shape[1]}x{img.shape[0]}')
    cv2.imwrite('/tmp/raw_frame.png', img)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
             | cv2.CALIB_CB_FAST_CHECK)
    found, corners = cv2.findChessboardCorners(gray, (COLS, ROWS), flags)
    print(f'棋盘检测: {"找到" if found else "没找到"}')
    vis = img.copy()
    if found:
        c = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        cv2.drawChessboardCorners(vis, (COLS, ROWS), c, found)
        pts = c.reshape(-1, 2)
        print(f'  角点像素范围 x[{pts[:,0].min():.0f},{pts[:,0].max():.0f}] '
              f'y[{pts[:,1].min():.0f},{pts[:,1].max():.0f}]')
        # 棋盘行/列的直线度：理想情况下每行角点的 y 应该共线
        grid = pts.reshape(ROWS, COLS, 2)
        for r in range(ROWS):
            y = grid[r, :, 1]
            print(f'  第 {r} 行 y: {np.round(y,1)}  极差 {y.ptp():.2f} px')
        print(f'  第 0 行首末角点间距 = '
              f'{np.linalg.norm(grid[0,0]-grid[0,-1]):.1f} px '
              f'(物理 {6*CELL_CM:.1f} cm)')
    cv2.imwrite('/tmp/raw_frame_marked.png', vis)
    print('已写 /tmp/raw_frame.png 与 /tmp/raw_frame_marked.png')
    cap.release()
    return 0


if __name__ == '__main__':
    sys.exit(main())
