#!/usr/bin/env python3
"""固定爪内侧面 vs 实际接触点：水平差多少

为什么必须量这个
----------------
触标工具算"接触点"的方式是：**TCP 位置减去一个竖直偏移**
（`tcp_above_lowest_m`）。这隐含假设：**接触点正好在 TCP 的正下方**。

如果改用"固定爪内侧面"当 TCP，而实际碰板的是爪尖，那么两者在
**水平方向**上的偏差不会被这个竖直偏移扣掉 —— 会直接变成外参的
系统性误差，而且残差看不出来（5 个点一致地偏）。

所以换 TCP 之前必须先确认：对准"固定爪内侧面"时，真正碰板的点
偏了多少。
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
    ap.add_argument('--gripper', type=float, default=0.340,
                    help='夹爪角（默认 0.340 = 40mm 开度）')
    ap.add_argument('--stride', type=int, default=3)
    args = ap.parse_args()

    m = GripperModel(stride=args.stride)
    if m.selftest():
        print('❌ FK 自检未通过')
        return 1

    fixed_l = m.link_points(FLANGE_LINK)
    jaw_l = m.link_points(JAW_LINK)

    print('把工具轴摆成竖直朝下，量各点在水平面上的投影位置。')
    print('（用臂关节全 0、只动 wrist_flex 让工具朝下不方便，直接用'
          'gripper_frame_link 坐标系：工具轴 = frame +Z）\n')

    for g in (args.gripper, 0.0, 0.75, 1.745):
        j = {k: 0.0 for k in JOINTS}
        j['gripper'] = g
        T = m.solve(j)
        Tf = T[FRAME_LINK]

        def to_frame(T_link, pts):
            w = (T_link[:3, :3] @ pts.T).T + T_link[:3, 3]
            return (Tf[:3, :3].T @ (w - Tf[:3, 3]).T).T

        ff = to_frame(T[FLANGE_LINK], fixed_l)
        jf = to_frame(T[JAW_LINK], jaw_l)

        # 固定爪内侧面：X 接近 0 的那些点。取它们的 Y/Z 范围。
        face = ff[np.abs(ff[:, 0]) < 0.0008]
        if len(face) < 10:
            face = ff[np.abs(ff[:, 0]) < 0.002]
        # 实际接触点 = 整个夹爪里 frame Z 最大的点（工具轴 +Z 朝向物块，
        # 工具竖直朝下时 frame +Z 就是世界 −Z，所以 Z 最大 = 世界最低）
        allp = np.vstack([ff, jf])
        k_low = int(np.argmax(allp[:, 2]))
        low = allp[k_low]

        n_open = m.jaw_opening(g)
        print(f'夹爪 {math.degrees(g):+6.2f}°   开度 '
              f'{n_open*1000 if n_open else float("nan"):6.2f}mm')
        print(f'  固定爪内侧面点数 {len(face)}')
        print(f'    内侧面 X 范围 [{face[:,0].min()*1000:+.2f}, '
              f'{face[:,0].max()*1000:+.2f}] mm')
        print(f'    内侧面 Z 范围 [{face[:,2].min()*1000:+.2f}, '
              f'{face[:,2].max()*1000:+.2f}] mm')
        print(f'  实际最低点 (frame) = ({low[0]*1000:+.2f}, '
              f'{low[1]*1000:+.2f}, {low[2]*1000:+.2f}) mm')
        # 水平偏差 = 在 frame 的 (X, Y) 平面上的距离
        # 工具竖直时，frame 的 X/Y 就是世界的水平面
        dx = low[0] - 0.0      # 内侧面在 X≈0
        dy = low[1] - 0.0
        print(f'  → 最低点相对"内侧面中心(0,0)"的水平偏差 = '
              f'({dx*1000:+.2f}, {dy*1000:+.2f}) mm  '
              f'|{math.hypot(dx,dy)*1000:.2f}| mm')
        print(f'  竖直偏移 (Z) = {low[2]*1000:+.2f} mm  '
              f'（这个量工具已经会扣）')
        print()

    print('=' * 70)
    print('判读：')
    print('  水平偏差就是换用"固定爪内侧面"当 TCP 后会引入的系统性误差。')
    print('  竖直偏差工具会扣掉，水平偏差**不会** —— 它会直接进外参。')
    print('  如果水平偏差只有 1~2mm，可以接受；如果接近 10mm 就不能这么换。')
    print('=' * 70)
    return 0


if __name__ == '__main__':
    sys.exit(main())
