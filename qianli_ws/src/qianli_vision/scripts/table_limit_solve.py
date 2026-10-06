#!/usr/bin/env python3
"""从已记录的最低点 JSON 求解桌面平面并写出限位文件（离线，可回放）

前端 `table_limit_gui.py` 的记录只存在内存里；万一界面出问题，
把点存成 JSON 用本脚本回放，不用重新手拖一遍。

marks JSON 格式（与前端 /tmp/table_marks.json 一致）::

    [{"p": [x, y, z], "tilt": 2.3, "link": "gripper_link", "at": "09:20:00"}, ...]

用法::

    ~/mj/bin/python table_limit_solve.py --marks /tmp/table_marks.json \
        --margin-mm 8 --out /tmp/table_limit.txt
"""

from __future__ import annotations

import argparse
import json
import math
import sys

import numpy as np


def fit_plane(points):
    centroid = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - centroid)
    n = vt[2]
    if n[2] < 0:
        n = -n
    return n, float(n @ centroid)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--marks', required=True)
    ap.add_argument('--margin-mm', type=float, default=8.0)
    ap.add_argument('--out', default='/tmp/table_limit.txt')
    args = ap.parse_args()

    with open(args.marks) as fh:
        marks = json.load(fh)
    if len(marks) < 3:
        print(f'❌ 至少需要 3 个点（现在 {len(marks)}）')
        return 1

    pts = np.array([m['p'] for m in marks], dtype=float)
    n, d = fit_plane(pts)
    res = (pts @ n - d) * 1000.0
    rms = float(np.sqrt(np.mean(res ** 2)))
    tilt = math.degrees(math.acos(min(1.0, abs(float(n[2])))))
    zs = pts @ n
    safe = float(zs.max()) + args.margin_mm / 1000.0

    print('=' * 66)
    print('  桌面标定结果（记录夹爪最低点，姿态无关）')
    print('=' * 66)
    print(f'  点数            {len(marks)}')
    print(f'  XY 跨度         [{np.ptp(pts[:,0])*1000:.1f}, '
          f'{np.ptp(pts[:,1])*1000:.1f}] mm')
    print(f'  桌面高度        {float(zs.mean())*1000:+.2f} mm')
    print(f'  平面倾角        {tilt:.3f}°  （偏离水平）')
    print(f'  拟合残差        RMS {rms:.3f} mm   最大 '
          f'{float(np.max(np.abs(res))):.3f} mm')
    print(f'  单点残差(mm)    {np.round(res, 2).tolist()}')
    print(f'  安全下限        {safe*1000:+.2f} mm '
          f'(= 平面最高处 + {args.margin_mm:.1f}mm 边距)')
    print()
    print(f'  质检            ' + ('✅ 可信' if rms < 1.5 else
                                   ('⚠️ 尚可' if rms < 3 else '❌ 残差过大')))

    lines = [
        '# 桌面标定（记录夹爪最低点，姿态无关）',
        f'# 采样 {len(marks)} 点  XY跨度 '
        f'[{np.ptp(pts[:,0])*1000:.1f}, {np.ptp(pts[:,1])*1000:.1f}] mm',
        f'# 平面倾角 {tilt:.4f}°  拟合残差 RMS {rms:.3f} mm',
        '# 依据：碰到桌面的是夹爪几何最低点，不是 TCP',
        '# （TCP 相对最低点的高度随姿态变化，实测 5mm ~ 100mm）',
        f'table_z={d:.6f}',
        f'table_z_mean={float(zs.mean()):.6f}',
        f'table_normal_x={n[0]:.6f}',
        f'table_normal_y={n[1]:.6f}',
        f'table_normal_z={n[2]:.6f}',
        f'safe_z_min={safe:.6f}',
        f'margin_mm={args.margin_mm}',
    ]
    with open(args.out, 'w') as fh:
        fh.write('\n'.join(lines) + '\n')
    print(f'\n📄 已写入 {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
