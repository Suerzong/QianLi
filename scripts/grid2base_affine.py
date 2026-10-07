#!/usr/bin/env python3
"""用 5 组实测点拟合 棋盘坐标 -> base_link，并给出方块位置。

对比刚性(4DOF) 与 仿射(6DOF)：仿射能吸收非刚性系统误差（尺度/剪切）。
"""

from project_paths import open_video_capture

from project_paths import default_camera
import math

import cv2
import numpy as np

# 上一轮实测（棋盘坐标 cm -> base_link m）
MARKS = [((0.0, 0.0), (0.2915, 0.0296)),
         ((9.9, 0.0), (0.3050, -0.0688)),
         ((0.0, 6.6), (0.2260, 0.0168)),
         ((9.9, 6.6), (0.2323, -0.0710)),
         ((3.3, 3.3), (0.2582, -0.0086))]

G = np.array([m[0] for m in MARKS], float) / 100.0
B = np.array([m[1] for m in MARKS], float)


def rms(res):
    return float(np.sqrt(np.mean(np.sum(res ** 2, axis=1)))) * 1000


# 刚性
gc, bc = G.mean(0), B.mean(0)
Gd, Bd = G - gc, B - bc
num = float(np.sum(Gd[:, 0] * Bd[:, 1] - Gd[:, 1] * Bd[:, 0]))
den = float(np.sum(Gd[:, 0] * Bd[:, 0] + Gd[:, 1] * Bd[:, 1]))
th = math.atan2(num, den)
c, s = math.cos(th), math.sin(th)
R = np.array([[c, -s], [s, c]])
t = bc - R @ gc
res_r = (R @ G.T).T + t - B
print(f'刚性 4DOF: theta={math.degrees(th):+.2f}deg origin=({t[0]:.4f},{t[1]:.4f}) '
      f'RMS={rms(res_r):.2f}mm')

# 仿射
X = np.hstack([G, np.ones((len(G), 1))])
coef, *_ = np.linalg.lstsq(X, B, rcond=None)
res_a = X @ coef - B
print(f'仿射 6DOF: RMS={rms(res_a):.2f}mm')
print('  逐点残差(mm): 刚性 / 仿射')
for m, r1, r2 in zip(MARKS, res_r, res_a):
    print(f'    {str(m[0]):>12}  ({r1[0]*1000:+7.1f},{r1[1]*1000:+7.1f})  '
          f'({r2[0]*1000:+7.1f},{r2[1]*1000:+7.1f})')

A = coef[:2].T
print(f'  仿射 x列={np.round(A[:,0],4).tolist()} |x|={np.linalg.norm(A[:,0]):.4f}')
print(f'  仿射 y列={np.round(A[:,1],4).tolist()} |y|={np.linalg.norm(A[:,1]):.4f}')
ang = math.degrees(math.acos(np.clip(A[:, 0] @ A[:, 1] /
      (np.linalg.norm(A[:, 0]) * np.linalg.norm(A[:, 1])), -1, 1)))
print(f'  两列夹角 {ang:.2f}deg (理想 90)  |列长| 理想 1.0')


def to_base_affine(gx_cm, gy_cm):
    v = coef[:2].T @ np.array([gx_cm / 100.0, gy_cm / 100.0]) + coef[2]
    return v


# ---- 视觉：棋盘 + 方块 ----
CELL = 33.0
COLS, ROWS = 7, 5
FLAGS = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
cap = open_video_capture(default_camera())
for _ in range(12):
    ok, img = cap.read()
    if ok:
        break
cap.release()
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
found, corners = cv2.findChessboardCorners(gray, (COLS, ROWS), FLAGS)
print(f'\n棋盘 7x5: {"✅" if found else "❌"}')
if not found:
    raise SystemExit(0)
corners = cv2.cornerSubPix(gray, corners, (7, 7), (-1, -1),
                           (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
c2 = corners.reshape(-1, 2).astype(np.float32)
obj = np.zeros((COLS * ROWS, 2), np.float32)
obj[:, 0] = np.tile(np.arange(COLS), ROWS) * CELL
obj[:, 1] = np.repeat(np.arange(ROWS), COLS) * CELL
H, _ = cv2.findHomography(c2, obj)
d = np.linalg.norm(c2[1:] - c2[:-1], axis=1)
print(f'  尺度: 中位角点间距 {np.median(d):.1f}px → {CELL/np.median(d):.3f} mm/px')

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
SPEC = {'blue': [((95, 90, 60), (135, 255, 255))],
        'green': [((35, 80, 60), (85, 255, 255))],
        'yellow': [((20, 90, 90), (34, 255, 255))],
        'red': [((0, 100, 70), (8, 255, 255)), ((170, 100, 70), (179, 255, 255))],
        'purple': [((136, 60, 60), (168, 255, 255))]}
print('\n方块定位（40mm 方块在 1.05mm/px 下应约 38px）:')
print(f'{"颜色":>7}{"像素":>14}{"边长px":>8}{"棋盘cm":>18}{"base(仿射)":>22}  判定')
mmpp = CELL / float(np.median(d))
for name, rngs in SPEC.items():
    mask = None
    for lo_, hi_ in rngs:
        m = cv2.inRange(hsv, np.array(lo_), np.array(hi_))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in cnts:
        a = cv2.contourArea(cnt)
        if a < 250:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        cx, cy = x + w / 2, y + h / 2
        v = H @ np.array([cx, cy, 1.0])
        v /= v[2]
        gx, gy = v[0] / 10.0, v[1] / 10.0
        bx, by = to_base_affine(gx, gy)
        est = max(w, h) * mmpp
        ok_size = 25 <= est <= 70
        print(f'{name:>7}{f"({cx:.0f},{cy:.0f})":>14}{max(w,h):>8.0f}'
              f'{f"({gx:+.2f},{gy:+.2f})":>18}{f"({bx:.3f},{by:.3f})":>22}'
              f'  {"尺寸OK" if ok_size else f"尺寸异常{est:.0f}mm"}')
