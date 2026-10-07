#!/usr/bin/env python3
"""相机自检：试 video0/video1 两个索引。"""

from project_paths import open_video_capture
import time

import cv2

for idx in (0, 1):
    cap = open_video_capture(idx)
    if not cap.isOpened():
        print(f'index {idx}: OPEN FAIL')
        cap.release()
        continue
    ok, fr = cap.read()
    print(f'index {idx}: read ok={ok} '
          f'{time.time()-time.monotonic():.2f}s shape={None if fr is None else fr.shape}')
    cap.release()
print('DONE')
