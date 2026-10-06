#!/usr/bin/env python3
"""爪口开合行程：SO-101 能夹多大的物块

为什么不能用"两爪网格最近距离"
------------------------------
第一版就是这么写的，结果量出来"最大爪口 0.3mm" —— 因为两片爪的**侧面
和铰链**永远靠得很近，全局最近点根本不在爪口上（实测最近点落在
|Y|≈18-20mm、Z≈-13~-32mm 的铰链侧壁）。

正确做法（与 tcp_from_model.py 一致）：
  1. 全部换算到 gripper_frame_link 坐标系；
  2. 只在**爪尖前端**区域比较（否则最近点必然是铰链）；
  3. 在抓取深度所在的 Z 薄层里，分别取两爪的 X 跨度；
  4. 爪口 = 固定爪内侧 X0 − 活动爪内侧 X。

已实测（tcp_from_model.py 三个开度一致）：固定爪内侧面稳定在 frame X≈0，
活动爪随开度往 −X 走。
"""

from __future__ import annotations

import argparse
import math
import sys

import numpy as np

from gripper_model import GripperModel, JOINTS, JAW_LINK, FLANGE_LINK

FRAME_LINK = 'gripper_frame_link'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--stride', type=int, default=3)
    ap.add_argument('--front-mm', type=float, default=25.0,
                    help='只比较爪尖往内这个深度的前端区域')
    ap.add_argument('--band-mm', type=float, default=6.0,
                    help='抓取深度处的 Z 薄层厚度')
    ap.add_argument('--tcp-z-mm', type=float, default=2.373,
                    help='抓取深度（= tcp_link 在 frame 下的 z），来自 TCP 标定')
    args = ap.parse_args()

    m = GripperModel(stride=args.stride)
    fails = m.selftest()
    if fails:
        print('❌ FK 自检未通过：', fails)
        return 1

    fixed_l = m.link_points(FLANGE_LINK)
    jaw_l = m.link_points(JAW_LINK)
    print(f'网格点：固定爪 {len(fixed_l)}，活动爪 {len(jaw_l)}')

    base = {k: 0.0 for k in JOINTS}
    band = args.band_mm / 1000.0
    z0 = args.tcp_z_mm / 1000.0

    def to_frame(T_frame, T_link, pts):
        """link 系点 → gripper_frame_link 系"""
        w = (T_link[:3, :3] @ pts.T).T + T_link[:3, 3]
        return (T_frame[:3, :3].T @ (w - T_frame[:3, 3]).T).T

    print()
    print(f'{"夹爪(°)":>8}{"固定爪内侧X0":>14}{"活动爪内侧X":>14}'
          f'{"爪口(mm)":>11}  说明')
    print('-' * 74)
    rows = []
    for g in np.arange(-0.132, 1.76, 0.08):
        base['gripper'] = float(g)
        T = m.solve(base)
        Tf, Tl, Tj = T[FRAME_LINK], T[FLANGE_LINK], T[JAW_LINK]
        ff = to_frame(Tf, Tl, fixed_l)
        jf = to_frame(Tf, Tj, jaw_l)

        # 前端区域：从爪尖往内 front_mm
        tip_z = max(ff[:, 2].max(), jf[:, 2].max())
        z_cut = tip_z - args.front_mm / 1000.0
        sf = ff[(ff[:, 2] >= z_cut) & (np.abs(ff[:, 2] - z0) < band)]
        sj = jf[(jf[:, 2] >= z_cut) & (np.abs(jf[:, 2] - z0) < band)]
        if len(sf) < 5 or len(sj) < 5:
            print(f'{math.degrees(g):>8.1f}   前端/薄层点不足（{len(sf)},'
                  f'{len(sj)}），检查 --front-mm / --band-mm')
            continue

        # 固定爪占 +X 侧（内侧面 = min X）；活动爪占 −X 侧（内侧面 = max X）
        x0 = float(sf[:, 0].min())
        xj = float(sj[:, 0].max())
        opening = x0 - xj
        rows.append((float(g), opening))
        note = ''
        if opening >= 0.040:
            note = '✅ 装得下 40mm'
        print(f'{math.degrees(g):>8.1f}{x0*1000:>14.2f}{xj*1000:>14.2f}'
              f'{opening*1000:>11.2f}  {note}')

    if not rows:
        print('❌ 没算出行程')
        return 1
    ws = [r[1] for r in rows]
    gmax = rows[int(np.argmax(ws))]
    print()
    print('=' * 74)
    print(f'  最大爪口 = {max(ws)*1000:.1f} mm  (夹爪角 {math.degrees(gmax[0]):.1f}°)')
    print(f'  最小爪口 = {min(ws)*1000:.1f} mm')
    print(f'  40mm EVA 块：{"✅ 装得下" if max(ws) >= 0.040 else "❌ 装不下"}')
    for g, w in rows:
        if w <= 0.040:
            print(f'  夹住 40mm 时的夹爪角 ≈ {math.degrees(g):.1f}° '
                  f'({g:.3f} rad)')
            break
    print('=' * 74)
    return 0


if __name__ == '__main__':
    sys.exit(main())
