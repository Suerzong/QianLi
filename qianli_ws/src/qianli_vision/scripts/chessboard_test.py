#!/usr/bin/env python3
"""测试棋盘格角点识别（比 Hough 格线更准，原点明确）

棋盘格标定原理：
  cv2.findChessboardCorners 找出所有内角点（亚像素精度）
  内角点 (i,j) → 物理坐标 (j*CELL_CM, i*CELL_CM)
  → findHomography → 高精度像素↔物理映射
  原点 = 第一个内角点（棋盘左上角第一个"十字"交点）

输出：/tmp/chessboard.png 标注图（角点编号）
"""

from project_paths import open_video_capture

from project_paths import default_camera

import itertools
import sys

import cv2
import numpy as np

CELL_CM = 3.3
ROI = (205, 143, 281, 216)

cap = open_video_capture(default_camera())
if not cap.isOpened():
    print('无法打开相机')
    sys.exit(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
frames = []
for _ in range(7):
    ok, f = cap.read()
    if ok:
        frames.append(f)
cap.release()
if not frames:
    print('读帧失败')
    sys.exit(1)
frame = np.median(np.array(frames), axis=0).astype(np.uint8)

x, y, w, h = ROI
roi = frame[y:y + h, x:x + w].copy()
gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

# 常见棋盘内角点数（列, 行）—— 8x6 方格的棋盘内角是 7x5
patterns = [(7, 5), (6, 4), (8, 6), (5, 3), (9, 7), (6, 5), (7, 6), (9, 6)]

found = None
for cols, rows in patterns:
    ok, corners = cv2.findChessboardCorners(
        gray, (cols, rows),
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_FAST_CHECK)
    if ok:
        print(f'✅ 识别成功: 内角点 {cols}x{rows}（= {cols+1}x{rows+1} 方格）')
        found = (cols, rows, corners)
        break
    else:
        print(f'❌ {cols}x{rows} 未识别')

if found is None:
    print('\n未识别到棋盘格。可能原因：ROI 不对 / 光照不均 / 棋盘不在 ROI 内')
    cv2.imwrite('/tmp/chessboard.png', frame)
    sys.exit(0)

cols, rows, corners = found
# 亚像素精化
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)

# 建立 像素 → 物理 映射（原点 = 第一个内角点）
pts_flat = corners.reshape(-1, 2)
src, dst = [], []
for i in range(rows):
    for j in range(cols):
        idx = i * cols + j
        px, py = pts_flat[idx]
        src.append((px, py))
        dst.append((j * CELL_CM, i * CELL_CM))
src = np.array(src, np.float32)
dst = np.array(dst, np.float32)
H, mask_h = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
print(f'\nHomography 计算: {"成功" if H is not None else "失败"}')
if H is not None:
    # 反投影误差
    proj = cv2.perspectiveTransform(src.reshape(-1, 1, 2), H).reshape(-1, 2)
    err = np.linalg.norm(proj - dst, axis=1)
    print(f'角点重投影误差: 平均 {err.mean():.3f} cm, 最大 {err.max():.3f} cm')
    # 棋盘物理尺寸
    print(f'棋盘覆盖范围: {cols*CELL_CM:.1f} x {rows*CELL_CM:.1f} cm')

# 标注图
display = frame.copy()
cv2.rectangle(display, (x, y), (x + w, y + h), (255, 0, 0), 2)
pts = corners.reshape(-1, 2)
for idx, (px, py) in enumerate(pts):
    gx, gy = int(x + px), int(y + py)
    cv2.circle(display, (gx, gy), 3, (0, 255, 0), -1)
    if idx == 0:
        cv2.circle(display, (gx, gy), 10, (0, 0, 255), 2)
        cv2.putText(display, 'ORIGIN(0,0)', (gx + 12, gy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
cv2.putText(display, f'corners {cols}x{rows}  red=origin',
            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
cv2.imwrite('/tmp/chessboard.png', display)
print('\n已保存 /tmp/chessboard.png')
