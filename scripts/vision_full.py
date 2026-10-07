#!/usr/bin/env python3
"""棋盘标定 + 方块定位（相机给每个位置一个坐标）。

1) findChessboardCorners(7x5) 找内角点 -> 单应 像素<->棋盘坐标(mm)
2) 颜色分割找方块 -> 映射成棋盘坐标
3) 用外参换算到 base_link（旧外参，仅作初值）
"""

from project_paths import default_camera
import math

import cv2
import numpy as np

CELL = 33.0
COLS, ROWS = 7, 5                     # 内角点
FLAGS = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
CELL_CM = CELL / 10.0

# 旧外参（RMS 60mm，仅作初值/定位参考）
EXT_ORIGIN = (0.2246, 0.0218)
EXT_THETA_DEG = -83.499


def to_base(gx_cm, gy_cm):
    th = math.radians(EXT_THETA_DEG)
    c, s = math.cos(th), math.sin(th)
    gx, gy = gx_cm / 100.0, gy_cm / 100.0
    return (EXT_ORIGIN[0] + c * gx - s * gy,
            EXT_ORIGIN[1] + s * gx + c * gy)


cap = cv2.VideoCapture(default_camera())
for _ in range(12):
    ok, img = cap.read()
    if ok:
        break
cap.release()
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

found, corners = cv2.findChessboardCorners(gray, (COLS, ROWS), FLAGS)
print(f'棋盘 7x5 内角点: {"识别到 ✅" if found else "未识别 ❌"}')
if not found:
    print('→ 请确认蓝方块已从棋盘上移开、棋盘四边与内角点完整可见')
    raise SystemExit(0)

corners = cv2.cornerSubPix(
    gray, corners, (7, 7), (-1, -1),
    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
c2 = corners.reshape(-1, 2)
print(f'  角点像素跨度: x {c2[:,0].min():.0f}..{c2[:,0].max():.0f}  '
      f'y {c2[:,1].min():.0f}..{c2[:,1].max():.0f}')
# 尺度自检：相邻格距（像素）应大致均匀
d = np.linalg.norm(c2[1:] - c2[:-1], axis=1)
print(f'  相邻角点像素间距: 中位 {np.median(d):.1f}px  '
      f'(33mm → {CELL/np.median(d):.3f} mm/px)')

# 单应：像素 -> 棋盘 mm（原点=第一个内角点）
obj = np.zeros((COLS * ROWS, 2), np.float32)
obj[:, 0] = np.tile(np.arange(COLS), ROWS) * CELL
obj[:, 1] = np.repeat(np.arange(ROWS), COLS) * CELL
H, _ = cv2.findHomography(c2.astype(np.float32), obj)


def px_to_grid(px, py):
    v = H @ np.array([px, py, 1.0])
    v /= v[2]
    return v[0], v[1]


# 方块检测
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
SPEC = {
    'blue':   [((95, 90, 60), (135, 255, 255))],
    'green':  [((35, 80, 60), (85, 255, 255))],
    'yellow': [((20, 90, 90), (34, 255, 255))],
    'red':    [((0, 100, 70), (8, 255, 255)), ((170, 100, 70), (179, 255, 255))],
    'purple': [((136, 60, 60), (168, 255, 255))],
}
print('\n方块在棋盘坐标下的位置:')
print(f'{"颜色":>7}{"像素中心":>14}{"像素边长":>10}{"棋盘(cm)":>18}'
      f'{"base(m)":>20}  判定')
report = []
for name, ranges in SPEC.items():
    mask = None
    for lo, hi in ranges:
        m = cv2.inRange(hsv, np.array(lo), np.array(hi))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in cnts:
        a = cv2.contourArea(cnt)
        if a < 250:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        cx, cy = x + w / 2, y + h / 2
        gx, gy = px_to_grid(cx, cy)
        gx_cm, gy_cm = gx / 10, gy / 10
        bx, by = to_base(gx_cm, gy_cm)
        span = max(w, h)
        on_board = (-0.5 <= gx / CELL <= COLS - 0.5
                    and -0.5 <= gy / CELL <= ROWS - 0.5)
        # 方块≈40mm；棋盘格 33mm -> 约 1.2 格 → 像素尺度自检
        mm_per_px = CELL / np.median(d)
        est_mm = span * mm_per_px
        tag = '在棋盘上' if on_board else '棋盘外/桌面'
        if not (25 <= est_mm <= 70):
            tag += f' (尺寸可疑 {est_mm:.0f}mm)'
        print(f'{name:>7}{f"({cx:.0f},{cy:.0f})":>14}{span:>9.0f}px'
              f'{f"({gx_cm:+.2f},{gy_cm:+.2f})":>18}'
              f'{f"({bx:.3f},{by:.3f})":>20}  {tag}')
        report.append(dict(color=name, px=[cx, cy], span_px=span,
                           grid_cm=[gx_cm, gy_cm], base_m=[bx, by],
                           on_board=bool(on_board), est_size_mm=est_mm))
print(f'\n共 {len(report)} 个方块候选')
