#!/usr/bin/env python3
"""放大画面底部（折叠位夹爪区域），检查方块是否还夹着。"""
import cv2

img = cv2.imread('/tmp/scene.jpg')
h, w = img.shape[:2]
crop = img[int(h * 0.72):h, int(w * 0.30):int(w * 0.85)]
big = cv2.resize(crop, None, fx=3.0, fy=3.0, interpolation=cv2.INTER_CUBIC)
cv2.imwrite('/tmp/grip_zoom.jpg', big)
print('已写 /tmp/grip_zoom.jpg  crop y从', int(h * 0.72), 'x从', int(w * 0.30))
