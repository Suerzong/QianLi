#!/usr/bin/env python3
"""① 保存视觉标定（单应 + 仿射换算）到 /tmp/vision_calib.json
② 求"固定爪尖端"在 gripper_frame 中的常向量（供上方抓取 IK 用）
"""
import json
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.expanduser(
    '~/QianLi/qianli_ws/src/qianli_vision/scripts'))
from gripper_model import GripperModel, JOINTS

# ---- ① 视觉标定 ----
MARKS = [((0.0, 0.0), (0.2915, 0.0296)),
         ((9.9, 0.0), (0.3050, -0.0688)),
         ((0.0, 6.6), (0.2260, 0.0168)),
         ((9.9, 6.6), (0.2323, -0.0710)),
         ((3.3, 3.3), (0.2582, -0.0086))]
G = np.array([m[0] for m in MARKS], float) / 100.0
B = np.array([m[1] for m in MARKS], float)
X = np.hstack([G, np.ones((len(G), 1))])
coef, *_ = np.linalg.lstsq(X, B, rcond=None)

CELL = 33.0
COLS, ROWS = 7, 5
FLAGS = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
cap = cv2.VideoCapture(0)
for _ in range(12):
    ok, img = cap.read()
    if ok:
        break
cap.release()
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
found, corners = cv2.findChessboardCorners(gray, (COLS, ROWS), FLAGS)
print(f'棋盘 7x5: {"✅ 已更新单应" if found else "❌ 未识别（沿用旧缓存）"}')
Hcache = None
if found:
    corners = cv2.cornerSubPix(
        gray, corners, (7, 7), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
    obj = np.zeros((COLS * ROWS, 2), np.float32)
    obj[:, 0] = np.tile(np.arange(COLS), ROWS) * CELL
    obj[:, 1] = np.repeat(np.arange(ROWS), COLS) * CELL
    Hcache, _ = cv2.findHomography(corners.reshape(-1, 2).astype(np.float32), obj)
    res = np.linalg.norm((Hcache @ np.hstack([corners.reshape(-1, 2),
                                              np.ones((COLS * ROWS, 1))]).T).T[:, :2]
                         / (Hcache @ np.hstack([corners.reshape(-1, 2),
                                                np.ones((COLS * ROWS, 1))]).T).T[:, 2:3]
                         - obj, axis=1)
    print(f'  单应重投影误差: 中位 {np.median(res):.2f}mm 最大 {res.max():.2f}mm')
calib = {'H': Hcache.tolist() if Hcache is not None else None,
         'affine': coef.tolist(), 'cell_mm': CELL, 'cols': COLS, 'rows': ROWS}
json.dump(calib, open('/tmp/vision_calib.json', 'w'), indent=2)
print('已写 /tmp/vision_calib.json')

# ---- ② 固定爪尖端在工具系的常向量 ----
model = GripperModel(stride=8)
pts_link = model.parts['gripper_link']
T0 = model.solve({k: 0.0 for k in JOINTS})
F = T0['gripper_frame_link']
GL = T0['gripper_link']
# gripper_link 网格点 -> gripper_frame
w = (GL[:3, :3] @ pts_link.T).T + GL[:3, 3]
p_frame = (F[:3, :3].T @ (w - F[:3, 3]).T).T
k = int(np.argmin(p_frame[:, 2]))
tip = p_frame[k]
print('\n固定爪(gripper_link)网格在工具系中的范围:')
print(f'  x {p_frame[:,0].min()*1000:+.1f}..{p_frame[:,0].max()*1000:+.1f} mm')
print(f'  y {p_frame[:,1].min()*1000:+.1f}..{p_frame[:,1].max()*1000:+.1f} mm')
print(f'  z {p_frame[:,2].min()*1000:+.1f}..{p_frame[:,2].max()*1000:+.1f} mm')
print(f'最深点(爪尖候选) = {np.round(tip*1000,1).tolist()} mm  '
      f'距TCP {np.linalg.norm(tip)*1000:.1f} mm')
# 前端区域（-z 最深的一批点）的平均
order = np.argsort(p_frame[:, 2])
front = p_frame[order[:max(5, len(order)//50)]]
print(f'最深 2% 点均值 = {np.round(front.mean(axis=0)*1000,1).tolist()} mm')
json.dump({'tip_local_mm': (tip * 1000).tolist(),
           'front_mean_mm': (front.mean(axis=0) * 1000).tolist()},
          open('/tmp/jaw_tip.json', 'w'), indent=2)
print('已写 /tmp/jaw_tip.json')
