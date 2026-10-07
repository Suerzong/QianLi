#!/usr/bin/env python3
"""拍摄背景参考图（空棋盘，无物块）—— 供背景差分检测使用

用法：
  1. 把物块从棋盘上拿走（拿在手里，别放在棋盘区域）
  2. 让机械臂停在画面外/画面底部（不遮挡棋盘）
  3. 运行本脚本 → 保存 /tmp/board_bg.png

原理：
  多帧取中值（抑制噪声）→ 存灰度图。之后 object_localizer 用
  |当前帧 - 背景| > 阈值 找物块：棋盘花纹完全抵消，
  只留下"新出现的东西"（物块/影子），对光照模式不敏感。
"""

from project_paths import calibration_path, default_camera

import sys

import cv2
import numpy as np

OUT = calibration_path('board_bg.png')
OUT_COLOR = calibration_path('board_bg_color.png')
N = 11

cap = cv2.VideoCapture(default_camera())
if not cap.isOpened():
    print('无法打开相机')
    sys.exit(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print(f'>>> 拍摄背景（{N} 帧取中值），请确保棋盘上没有物块…')
frames = []
for i in range(N):
    ok, f = cap.read()
    if ok:
        frames.append(f)
cap.release()
if len(frames) < 3:
    print('读帧失败')
    sys.exit(1)

med = np.median(np.array(frames), axis=0).astype(np.uint8)
gray = cv2.cvtColor(med, cv2.COLOR_BGR2GRAY)
cv2.imwrite(OUT, gray)
cv2.imwrite(OUT_COLOR, med)

# 简单质检：棋盘格是否可见（保证拍的是棋盘，不是被挡住）
ok_cb, _ = cv2.findChessboardCorners(
    gray, (7, 5), cv2.CALIB_CB_ADAPTIVE_THRESH
    | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)

print(f'✅ 背景已保存: {OUT}（{len(frames)} 帧中值）')
print(f'   棋盘格可见: {"是" if ok_cb else "否（可能被遮挡，建议重拍）"}')
print(f'   亮度范围: {gray.min()} ~ {gray.max()}, 均值 {gray.mean():.0f}')
