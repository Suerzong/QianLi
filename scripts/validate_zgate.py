#!/usr/bin/env python3
"""用旧 marks 验证修复：板面 z 判据会拒掉哪些点？幸存点重拟合 RMS 多少？

旧 marks 的 contact_m 是"整机最低点"；在夹爪角 40.5° 时整机最低点恰好
落在 gripper_link（固定爪）上，所以 contact_m ≈ 固定爪顶端，可以用它
模拟新判据的效果。
"""

from project_paths import calibration_path
import json
import math

import numpy as np

BOARD_Z = -0.06909 + 0.0005   # 桌面 + 棋盘纸
Z_TOL = 2.0 / 1000.0
CELL = 3.3 / 100.0


def solve_planar(grid_xy, base_xy):
    G = np.asarray(grid_xy, float)
    B = np.asarray(base_xy, float)
    gc, bc = G.mean(axis=0), B.mean(axis=0)
    Gd, Bd = G - gc, B - bc
    num = float(np.sum(Gd[:, 0] * Bd[:, 1] - Gd[:, 1] * Bd[:, 0]))
    den = float(np.sum(Gd[:, 0] * Bd[:, 0] + Gd[:, 1] * Bd[:, 1]))
    theta = math.atan2(num, den)
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, -s], [s, c]])
    t = bc - R @ gc
    pred = (R @ G.T).T + t
    return theta, t, np.linalg.norm(pred - B, axis=1)


def main():
    marks = json.load(open(calibration_path('extrinsic_marks.json')))
    print(f'板面 z = {BOARD_Z*1000:+.2f} mm, 判据容差 ±{Z_TOL*1000:.1f} mm\n')
    print(f'{"#":>3}{"grid(cm)":>12}{"contact z(mm)":>14}{"z-板面(mm)":>12}  判定')
    keep, reject = [], []
    for i, m in enumerate(marks, 1):
        cz = m['contact_m'][2]
        d = (cz - BOARD_Z) * 1000
        if abs(d) < Z_TOL * 1000:
            keep.append(m)
            verdict = '✅ 保留'
        else:
            reject.append(m)
            verdict = '❌ 拒掉'
        print(f'{i:>3}{str(m["grid_cm"]):>12}{cz*1000:>14.2f}{d:>12.2f}  {verdict}')

    print(f'\n保留 {len(keep)} / {len(marks)} 点')
    if len(keep) >= 3:
        grid = [np.array(m['grid_cm']) / 100.0 for m in keep]
        base = [np.array(m['contact_m'][:2]) for m in keep]
        th, t, res = solve_planar(grid, base)
        print(f'幸存点重拟合: θ={math.degrees(th):+.2f}°  '
              f't=({t[0]:+.4f},{t[1]:+.4f})  RMS={np.sqrt(np.mean(res**2))*1000:.2f} mm')
        print(f'单点残差: {np.round(res*1000,2).tolist()}')
    print()
    if reject:
        print('被拒点的特征:')
        for m in reject:
            print(f'  grid {m["grid_cm"]}  z 差 {(m["contact_m"][2]-BOARD_Z)*1000:+.2f} mm  '
                  f'gripper角 {m.get("gripper_rad")}')


if __name__ == '__main__':
    main()
