#!/usr/bin/env python3
"""抓一帧存图（供人工/视觉判读当前场景）。"""
import cv2

cap = cv2.VideoCapture(0)
img = None
for _ in range(20):
    ok, f = cap.read()
    if ok:
        img = f
cap.release()
if img is not None:
    cv2.imwrite('/tmp/scene.jpg', img)
    print('已存 /tmp/scene.jpg', img.shape)
else:
    print('取帧失败')
