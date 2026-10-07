#!/usr/bin/env python3
"""更强棋盘检测：findChessboardCornersSB + CLAHE + 多尺寸/多尺度。"""

from project_paths import open_video_capture
import cv2
import numpy as np

SIZES = [(7, 5), (6, 4), (9, 6), (8, 6), (5, 4), (6, 5), (7, 6), (10, 7)]

for idx in (0, 1):
    cap = open_video_capture(idx)
    if not cap.isOpened():
        continue
    ok, img = False, None
    for _ in range(10):
        ok, img = cap.read()
        if ok:
            break
    cap.release()
    if not ok:
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(3.0, (8, 8)).apply(gray)
    print('=' * 50)
    print(f'cam{idx}: 原图 std={gray.std():.1f}')
    hit = False
    for name, im in (('raw', gray), ('clahe', clahe),
                     ('half', cv2.resize(gray, None, fx=.5, fy=.5))):
        for size in SIZES:
            try:
                ok2, corners = cv2.findChessboardCornersSB(im, size)
            except Exception:
                continue
            if ok2:
                c = corners.reshape(-1, 2)
                sx = c[:, 0].max() - c[:, 0].min()
                sy = c[:, 1].max() - c[:, 1].min()
                scale = 2.0 if name == 'half' else 1.0
                print(f'  ✅ {name} {size}: 跨度 {sx*scale:.0f}x{sy*scale:.0f}px')
                hit = True
    if not hit:
        # 找"最像棋盘"的区域：局部对比度最强的块
        lap = cv2.Laplacian(gray, cv2.CV_32F)
        bs = 32
        h, w = gray.shape
        grid = []
        for y in range(0, h - bs + 1, bs):
            for x in range(0, w - bs + 1, bs):
                grid.append((float(np.abs(lap[y:y+bs, x:x+bs]).mean()), x, y))
        grid.sort(reverse=True)
        print('  未识别。局部对比度最高的 5 块 (强度, x, y):')
        for v, x, y in grid[:5]:
            print(f'    {v:6.1f}  ({x:3d},{y:3d})  亮度均值={gray[y:y+bs, x:x+bs].mean():.0f}')
