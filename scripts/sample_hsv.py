#!/usr/bin/env python3
"""取样方块实际 HSV + 连续5次 (0,0) 重合稳定性。"""

from project_paths import open_video_capture

from project_paths import default_camera, project_path
import os

import cv2
import json
import numpy as np

CFG = os.path.expanduser(project_path('config'))
REF = os.path.join(CFG, 'at00_mask.png')
fr = json.load(open(os.path.join(CFG, 'board_frame.json')))
H = np.array(fr['H'])
Hi = np.linalg.inv(H)
v = Hi @ np.array([0.0, 0.0, 1.0])
PX = v[:2] / v[2]
print('(0,0)像素:', PX.round(1))

cap = open_video_capture(default_camera())
for _ in range(25):
    cap.read()
res = []
for trial in range(5):
    ok, f = cap.read()
    hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
    # 方块位置采样（棋盘 (57.7,65.9)mm -> 像素）
    v2 = H @ np.array([57.7, 65.9, 1.0])
    pc = (v2[:2] / v2[2]).astype(int)
    roi = hsv[max(0, pc[1]-8):pc[1]+8, max(0, pc[0]-8):pc[0]+8]
    h_med = np.median(roi[:, :, 0])
    s_med = np.median(roi[:, :, 1])
    v_med = np.median(roi[:, :, 2])
    # (0,0) 重合
    x0, y0 = int(PX[0]), int(PX[1])
    win = f[max(0, y0-70):y0+70, max(0, x0-70):x0+70]
    m = cv2.inRange(cv2.cvtColor(win, cv2.COLOR_BGR2HSV),
                    np.array((15, 60, 60)), np.array((40, 255, 255)))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    cur = cv2.countNonZero(m)
    ref = cv2.imread(REF, 0)
    if ref is not None and ref.shape == m.shape:
        ov = float(cv2.countNonZero(cv2.bitwise_and(m, ref))) / \
            max(cv2.countNonZero(ref), 1)
    else:
        ov = -1
    res.append((h_med, s_med, v_med, cur, ov))
    print(f'试{trial}: 方块HSV=({h_med:.0f},{s_med:.0f},{v_med:.0f}) '
          f'(0,0)掩码={cur}px 重合={ov:.2f}')
cap.release()
