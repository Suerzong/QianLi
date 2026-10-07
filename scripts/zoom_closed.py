#!/usr/bin/env python3
"""放大 fsm_closed.jpg 的夹爪区域，看清方块与夹爪的相对位置。"""
import cv2

img = cv2.imread('/tmp/fsm_closed.jpg')
print('原图', img.shape)
crop = img[260:460, 320:560]
big = cv2.resize(crop, None, fx=3.0, fy=3.0, interpolation=cv2.INTER_CUBIC)
cv2.imwrite('/tmp/closed_zoom.jpg', big)
print('已写 /tmp/closed_zoom.jpg  crop 起点 (320,260) 放大 3x')
g = big.copy()
for x in range(0, big.shape[1], 150):
    cv2.line(g, (x, 0), (x, big.shape[0]), (0, 0, 255), 1)
    cv2.putText(g, str(int(320 + x / 3)), (x + 3, 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
for y in range(0, big.shape[0], 150):
    cv2.line(g, (0, y), (big.shape[1], y), (255, 0, 0), 1)
    cv2.putText(g, str(int(260 + y / 3)), (3, y + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 1)
cv2.imwrite('/tmp/closed_zoom_grid.jpg', g)
print('已写 /tmp/closed_zoom_grid.jpg（红=原图x 蓝=原图y）')
