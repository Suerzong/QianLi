#!/usr/bin/env python3
"""显示旧标定 10 点的 grid→base 映射，帮用户确认棋盘原点方位。"""

from project_paths import calibration_path
import json

marks = json.load(open(calibration_path('extrinsic_marks.json')))
print('旧标定 10 点顺序 (grid cm -> base m):')
for p in marks:
    c = p['contact_m']
    print(f'  grid {p["grid_cm"]!s:>12} -> base ({c[0]:.3f}, {c[1]:.3f})')
print()
print('首点 (0,0) 的 base 位置 = 棋盘原点在 base_link 下的方位')
