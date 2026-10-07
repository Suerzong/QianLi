#!/usr/bin/env python3
"""导出 target_viewer 的一帧标注图，便于核对叠加是否正确。"""

from project_paths import default_camera, project_path
import json
import os
import sys

import cv2

sys.path.insert(0, os.path.expanduser(
    project_path('qianli_ws/src/qianli_vision/scripts')))
import target_viewer as tv

vis = tv.Vision(20.0, 6.0, 'yellow')
cap = cv2.VideoCapture(default_camera())
img = None
for _ in range(15):
    ok, f = cap.read()
    if ok:
        img = f
cap.release()
img, info = vis.frame(img)
cv2.imwrite('/tmp/overlay.jpg', img)
print('已写 /tmp/overlay.jpg')
print('目标:', json.dumps(info.get('target', {}).get('base_m'), ensure_ascii=False))
if info.get('target'):
    g = info['target']['grid_mm']
    print(f"  目标棋盘坐标 ({g[0]/10:.2f}, {g[1]/10:.2f}) cm")
print('机械臂爪尖:', json.dumps(info.get('arm', {}).get('tip_m', {}),
                              ensure_ascii=False))
print('目标-实际 水平差:', info.get('target_vs_arm_mm'), 'mm')
print('\n检测到的方块:')
for b in info['blobs']:
    print(f"  {b['color']:>6} 像素({b['px'][0]:.0f},{b['px'][1]:.0f}) "
          f"{max(b['wh']):.0f}px  棋盘({b['grid_mm'][0]/10:+.1f},"
          f"{b['grid_mm'][1]/10:+.1f})cm  base({b['base_m'][0]:.3f},"
          f"{b['base_m'][1]:.3f}) 半径{b['radius_mm']:.0f}mm")
