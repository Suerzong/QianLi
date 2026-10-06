#!/usr/bin/env python3
"""扫描夹爪姿态，看"TCP 高出夹爪最低点"这个量能变化多少

这直接回答"为什么我每次让爪子碰桌子，读到的 TCP Z 差那么多"：
碰到桌子的恒是**夹爪最低点**，而 TCP 相对它的高度**随姿态变化**。

用法::
    ~/mj/bin/python tcp_offset_sweep.py
"""

from __future__ import annotations

import math
import sys

import numpy as np

from gripper_model import GripperModel, JOINTS


def main():
    model = GripperModel(stride=6)
    fails = model.selftest()
    if fails:
        print('❌ FK 自检未通过：', fails)
        return 1

    base = {n: 0.0 for n in JOINTS}
    base['shoulder_lift'] = 1.2
    base['elbow_flex'] = -0.8

    print(f'{"wrist_flex":>11}{"工具轴偏离竖直":>16}{"TCP Z":>12}'
          f'{"最低点 Z":>12}{"TCP-最低点":>13}  最低点部件')
    print('-' * 84)
    results = []
    for wf_deg in range(-90, 91, 15):
        joints = dict(base)
        joints['wrist_flex'] = math.radians(wf_deg)
        T = model.solve(joints)
        tcp = T['tcp_link'][:3, 3]
        tool = T['tcp_link'][:3, 2]
        tilt = math.degrees(math.acos(min(1.0, abs(float(tool[2])))))
        low, link = model.lowest_point(joints)
        d = (tcp[2] - low[2]) * 1000
        results.append(d)
        print(f'{wf_deg:>10}°{tilt:>15.1f}°{tcp[2]*1000:>12.1f}'
              f'{low[2]*1000:>12.1f}{d:>13.2f}  {link}')
    print()
    print(f'  "TCP - 最低点" 变化范围：{min(results):+.1f} ~ {max(results):+.1f} mm '
          f'（跨度 {max(results)-min(results):.1f} mm）')
    print()
    print('  这就是为什么"读 TCP 的 Z 当桌面高度"不成立：')
    print('  碰到桌子的永远是夹爪最低点，而 TCP 相对它的高度随姿态变。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
