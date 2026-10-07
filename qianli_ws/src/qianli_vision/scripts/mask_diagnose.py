#!/usr/bin/env python3
"""掩码诊断视图：实时显示 mask 中所有灰色区域 + 面积 + 物块框

用途：看清"掩码有问题"到底是哪种情况：
  - mask 混入杂质（手/阴影/网格纸）
  - 物块被拆碎
  - 参数不合适

尺寸过滤：只保留边长在 [MIN_SIZE, MAX_SIZE] 像素的物块（默认 30~50），
         过滤 15x72 那种窄条杂质。滑块实时可调。

按键：q 退出 | s 保存 /tmp/mask_diag.png
"""

from project_paths import open_video_capture

from project_paths import default_camera

import cv2
import numpy as np

MIN_AREA = 100
MIN_SIZE = 30    # 物块最小边长（像素）
MAX_SIZE = 50    # 物块最大边长（像素）

cap = open_video_capture(default_camera())
if not cap.isOpened():
    print('无法打开相机')
    exit(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

cv2.namedWindow('Size Filter')
cv2.createTrackbar('MIN_SIZE', 'Size Filter', MIN_SIZE, 100, lambda v: None)
cv2.createTrackbar('MAX_SIZE', 'Size Filter', MAX_SIZE, 200, lambda v: None)

print('mask 诊断启动（尺寸过滤 30~50px）。q 退出，s 保存。')

while True:
    ok, frame = cap.read()
    if not ok:
        continue

    min_size = cv2.getTrackbarPos('MIN_SIZE', 'Size Filter')
    max_size = cv2.getTrackbarPos('MAX_SIZE', 'Size Filter')

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 0, 60), (179, 127, 167))
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    cnts = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_AREA:
            continue
        bx, by, bw, bh = cv2.boundingRect(c)
        side = max(bw, bh)   # 用最长边判断尺寸
        if min_size <= side <= max_size:
            cnts.append((area, bx, by, bw, bh))
    cnts.sort(reverse=True)

    display = frame.copy()
    for i, (area, bx, by, bw, bh) in enumerate(cnts[:6]):
        color = (0, 255, 0) if i == 0 else (0, 200, 255)
        cv2.rectangle(display, (bx, by), (bx + bw, by + bh), color, 2)
        cv2.putText(display, f'#{i} a={int(area)} ({bx},{by}) {bw}x{bh}',
                    (bx, by - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    cv2.putText(display, f'kept: {len(cnts)} | size range '
                f'{min_size}-{max_size}px | gray: '
                f'{cv2.countNonZero(mask)}px',
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

    cv2.imshow('mask diagnose', display)
    cv2.imshow('mask', mask)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('s'):
        cv2.imwrite('/tmp/mask_diag.png', display)
        print('已保存 /tmp/mask_diag.png')

cap.release()
cv2.destroyAllWindows()
