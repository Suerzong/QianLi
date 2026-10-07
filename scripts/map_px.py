#!/usr/bin/env python3
"""把黄色大轮廓的像素位置映射到棋盘坐标，判断是否在棋盘上。"""
import json

import os

import numpy as np

fr = json.load(open(os.path.expanduser(
    '~/QianLi/qianli_ws/config/board_frame.json')))
print('棋盘 cols,rows:', fr.get('cols'), fr.get('rows'),
      'cell_mm:', fr.get('cell_mm'), 'pixel_extent:', fr.get('pixel_extent'))
H = np.array(fr['H'])
px = np.array([508.0, 407.0, 1.0])
v = H @ px
b = v[:2] / v[2]
print(f'像素(508,407) -> 棋盘mm: ({b[0]:.1f}, {b[1]:.1f})')
print('board_frame.json 字段:', list(fr.keys()))
if 'board_corners_mm' in fr:
    print('棋盘边界:', fr['board_corners_mm'])
# 反查：棋盘 (0,0) 和边界像素位置
v0 = np.linalg.inv(H) @ np.array([0.0, 0.0, 1.0])
p00 = v0[:2] / v0[2]
print(f'棋盘(0,0) -> 像素: ({p00[0]:.0f}, {p00[1]:.0f})')
# 棋盘大概范围（假设 6x5 格）
cell = float(fr.get('cell_mm', 31.25))
for corner in [(0, 0), (6 * cell, 0), (6 * cell, 5 * cell), (0, 5 * cell)]:
    vc = np.linalg.inv(H) @ np.array([corner[0], corner[1], 1.0])
    pc = vc[:2] / vc[2]
    print(f'棋盘mm{corner} -> 像素 ({pc[0]:.0f},{pc[1]:.0f})')
