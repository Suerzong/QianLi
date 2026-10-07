#!/usr/bin/env python3
"""裁出棋盘区域并放大 2 倍，便于目视确认图案四角。"""

from project_paths import default_camera
import cv2
import numpy as np

img = cv2.imread('/tmp/board_src.jpg')
if img is None:
    cap = cv2.VideoCapture(default_camera())
    for _ in range(20):
        ok, f = cap.read()
        if ok:
            img = f
    cap.release()
    cv2.imwrite('/tmp/board_src.jpg', img)

crop = img[170:420, 220:540]
big = cv2.resize(crop, None, fx=2.2, fy=2.2, interpolation=cv2.INTER_CUBIC)
cv2.imwrite('/tmp/board_zoom.jpg', big)
print('crop 起点 (220,170)  尺寸', crop.shape, '-> zoom', big.shape)
print('zoom 内坐标 (zx,zy) 对应原图 (220+zx/2.2, 170+zy/2.2)')

# 叠加网格线便于读数
g = big.copy()
for x in range(0, big.shape[1], 110):
    cv2.line(g, (x, 0), (x, big.shape[0]), (0, 0, 255), 1)
    cv2.putText(g, str(int(220 + x / 2.2)), (x + 3, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
for y in range(0, big.shape[0], 110):
    cv2.line(g, (0, y), (big.shape[1], y), (255, 0, 0), 1)
    cv2.putText(g, str(int(170 + y / 2.2)), (3, y + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 1)
cv2.imwrite('/tmp/board_zoom_grid.jpg', g)
print('已写 /tmp/board_zoom_grid.jpg（红=原图x，蓝=原图y）')
