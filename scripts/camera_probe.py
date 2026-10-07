#!/usr/bin/env python3
"""探测各摄像头：能否读帧、能否识别 7x5 棋盘内角点。"""
import cv2
import numpy as np

FLAGS = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
         | cv2.CALIB_CB_FAST_CHECK)

for idx in range(4):
    cap = cv2.VideoCapture(idx)
    if not cap.isOpened():
        print(f'cam{idx}: 打不开')
        cap.release()
        continue
    ok, img = False, None
    for _ in range(8):
        ok, img = cap.read()
        if ok:
            break
    cap.release()
    if not ok or img is None:
        print(f'cam{idx}: 打开但读不到帧')
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    path = f'/tmp/cam{idx}.jpg'
    cv2.imwrite(path, img)
    found, corners = cv2.findChessboardCorners(gray, (7, 5), FLAGS)
    print(f'cam{idx}: {img.shape[1]}x{img.shape[0]} 亮度均值={gray.mean():.1f} '
          f'对比度(std)={gray.std():.1f} 棋盘7x5={"识别到" if found else "未识别"} '
          f'-> {path}')
    if found:
        c = corners.reshape(-1, 2)
        span_x = c[:, 0].max() - c[:, 0].min()
        span_y = c[:, 1].max() - c[:, 1].min()
        print(f'         棋盘像素跨度: {span_x:.0f} x {span_y:.0f} '
              f'(格宽≈{span_x/6:.1f}px = 33mm → 尺度 {33/(span_x/6):.2f} mm/px)')
