#!/usr/bin/env python3
"""把 /tmp/extrinsic_new.txt 的实测外参写进长期棋盘坐标系。"""

from project_paths import calibration_path, project_path
import json
import math
import os
import re

EXT = calibration_path('extrinsic_new.txt')
FRAME = os.path.expanduser(project_path('config/board_frame.json'))


def parse(path):
    kv = {}
    for line in open(path, encoding='utf-8'):
        s = line.strip()
        if not s or s.startswith('#') or '=' not in s:
            continue
        k, v = s.split('=', 1)
        kv[k.strip()] = v.strip()
    return kv


kv = parse(EXT)
A = json.loads(kv['affine_A'])
b = json.loads(kv['affine_b'])
# 仿射 RMS 写在注释行里，用正则抓
txt = open(EXT, encoding='utf-8').read()
m = re.search(r'仿射RMS\s*([0-9.]+)', txt)
aff_rms = float(m.group(1)) if m else float('nan')
frame = json.load(open(FRAME)) if os.path.exists(FRAME) else {}
old_rms = frame.get('affine_rms_mm')
frame['affine'] = [A[0], A[1], b]
frame['affine_source'] = EXT
frame['affine_rms_mm'] = aff_rms
frame['rigid_rms_mm'] = float(kv['rms_mm'])
frame['grid_origin_rigid'] = [float(kv['grid_origin_x']),
                              float(kv['grid_origin_y'])]
frame['grid_theta_deg_rigid'] = float(kv['grid_theta_deg'])
frame['note'] = (f'affine 为实测 grid->base 仿射（5 点，RMS {aff_rms:.2f}mm）；'
                 f'刚性拟合 RMS {kv["rms_mm"]}mm 说明棋盘格非正方形，故用仿射')
json.dump(frame, open(FRAME, 'w'), indent=2, ensure_ascii=False)
print(f'已更新长期坐标系 {FRAME}')
print(f'  affine_A = {A}')
print(f'  affine_b = {b}')
print(f'  仿射 RMS {aff_rms:.2f} mm（旧 {old_rms}）')
print(f'  刚性 RMS {kv["rms_mm"]} mm')
print(f'  |x列| = {math.hypot(A[0][0], A[1][0]):.4f}  '
      f'|y列| = {math.hypot(A[0][1], A[1][1]):.4f}')
print(f'  → 实测格宽 x ≈ {math.hypot(A[0][0], A[1][0])*33:.1f}mm  '
      f'y ≈ {math.hypot(A[0][1], A[1][1])*33:.1f}mm')
