#!/usr/bin/env python3
"""触标时该用爪子哪个点？—— TCP 与爪口几何的对应关系

问题
----
`tcp_link` 是相对 `gripper_frame_link` 的**固定偏移** (-20.0, 0, -5.364) mm，
它**不随夹爪开合而动**。但它当初是照"W=40mm 开口时的开合中心"定的。
所以必须回答：
  1. 夹爪开到多少度，实际爪口才正好 40mm？
  2. 那个开度下，"开合中心"是不是真的落在 tcp_link 上？
  3. tcp_link 相对**爪尖**高多少？—— 因为用户是拿爪尖去碰桌面的，
     这决定了他"碰到板面"时 TCP 的 z 读数应该是多少。

不把这三条算清楚，触标就会引入系统性偏移，而且**残差看不出来**
（因为偏移是系统性的，5 个点会一致地偏，拟合残差依然很小）。
"""

from __future__ import annotations

import argparse
import math
import sys

import numpy as np

from gripper_model import GripperModel, JOINTS, JAW_LINK, FLANGE_LINK

FRAME = 'gripper_frame_link'
TCP = 'tcp_link'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--stride', type=int, default=3)
    ap.add_argument('--front-mm', type=float, default=25.0)
    ap.add_argument('--band-mm', type=float, default=6.0)
    ap.add_argument('--target-mm', type=float, default=40.0,
                    help='目标爪口宽度（EVA 块 40mm）')
    args = ap.parse_args()

    m = GripperModel(stride=args.stride)
    fails = m.selftest()
    if fails:
        print('❌ FK 自检未通过：', fails)
        return 1

    fixed_l = m.link_points(FLANGE_LINK)
    jaw_l = m.link_points(JAW_LINK)
    base = {k: 0.0 for k in JOINTS}
    band = args.band_mm / 1000.0

    # tcp_link 在 gripper_frame_link 下的固定偏移（从 URDF 读，不写死）
    T0 = m.solve(base)
    M = np.linalg.inv(T0[FRAME]) @ T0[TCP]
    tcp_off = M[:3, 3]
    print(f'tcp_link 在 {FRAME} 下的偏移 = '
          f'({tcp_off[0]*1000:+.3f}, {tcp_off[1]*1000:+.3f}, '
          f'{tcp_off[2]*1000:+.3f}) mm')
    print()

    def to_frame(Tf, Tl, pts):
        w = (Tl[:3, :3] @ pts.T).T + Tl[:3, 3]
        return (Tf[:3, :3].T @ (w - Tf[:3, 3]).T).T

    # 用 TCP 的 frame z 作为"抓取深度"这一层来量爪口
    z0 = float(tcp_off[2])

    print(f'{"夹爪(°)":>8}{"爪口(mm)":>10}{"开合中心X":>12}'
          f'{"爪尖最低z":>11}{"爪尖-TCP":>11}  tcp对准?')
    print('-' * 68)
    rows = []
    for g in np.arange(0.0, 1.30, 0.02):
        base['gripper'] = float(g)
        T = m.solve(base)
        Tf, Tl, Tj = T[FRAME], T[FLANGE_LINK], T[JAW_LINK]
        ff = to_frame(Tf, Tl, fixed_l)
        jf = to_frame(Tf, Tj, jaw_l)
        tip_z = max(ff[:, 2].max(), jf[:, 2].max())
        z_cut = tip_z - args.front_mm / 1000.0
        sf = ff[(ff[:, 2] >= z_cut) & (np.abs(ff[:, 2] - z0) < band)]
        sj = jf[(jf[:, 2] >= z_cut) & (np.abs(jf[:, 2] - z0) < band)]
        if len(sf) < 5 or len(sj) < 5:
            continue
        x0 = float(sf[:, 0].min())      # 固定爪内侧面
        xj = float(sj[:, 0].max())      # 活动爪内侧面
        opening = x0 - xj
        center = (x0 + xj) / 2.0
        tip_min = float(min(ff[:, 2].min(), jf[:, 2].min()))
        rows.append((float(g), opening, center, tip_min))

    # 找到最接近目标开度的那一行
    if not rows:
        print('❌ 没算出行程')
        return 1
    best = min(rows, key=lambda r: abs(r[1] - args.target_mm / 1000.0))
    for r in rows[::3]:
        mark = ' ←' if r is best else ''
        print(f'{math.degrees(r[0]):>8.1f}{r[1]*1000:>10.2f}'
              f'{r[2]*1000:>12.2f}{r[3]*1000:>11.2f}'
              f'{(r[3]-z0)*1000:>11.2f}  '
              f'{"YES" if abs(r[2]-tcp_off[0])<0.0005 else "no":>6}{mark}')

    print()
    print('=' * 68)
    g_best, op_best, c_best, tip_best = best
    print(f'  ★ 爪口 = {op_best*1000:.2f} mm 时，夹爪角 = '
          f'{math.degrees(g_best):.2f}°  ({g_best:.4f} rad)')
    print(f'    该开度下开合中心 X = {c_best*1000:+.2f} mm')
    print(f'    tcp_link 的 X     = {tcp_off[0]*1000:+.2f} mm')
    dx = (c_best - tcp_off[0]) * 1000
    print(f'    → 中心与 TCP 的偏差 {dx:+.2f} mm  '
          f'{"✅ 重合" if abs(dx) < 0.5 else "⚠️ 不重合"}')
    print()
    print(f'  ★ 爪尖最低点 frame z = {tip_best*1000:+.2f} mm')
    print(f'    tcp_link frame z   = {z0*1000:+.2f} mm')
    dz = (z0 - tip_best) * 1000
    print(f'    → TCP 比爪尖**高** {dz:+.2f} mm')
    print()
    print('  这决定了触标时"碰到板面"的那一刻，TCP 的 z 读数应该是：')
    print(f'      桌面 z + 棋盘厚度 + {dz:+.2f} mm')
    print('=' * 68)
    return 0


if __name__ == '__main__':
    sys.exit(main())
