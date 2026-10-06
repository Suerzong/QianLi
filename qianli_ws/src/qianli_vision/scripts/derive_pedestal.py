#!/usr/bin/env python3
"""从网格反推垫台高度

链条：
    桌面高度（实测）      TABLE_Z
    机械臂底座最低点      BASE_BOTTOM   ← 本脚本从 base_link 的网格算出
    垫台高度 = BASE_BOTTOM - TABLE_Z

`base_link` 上有三块几何：base_motor_holder_so101_v1、base_so101_v2、
waveshare_mounting_plate_so101_v2（还有一颗 sts3215 电机）。它们整体的
最小 Z 就是"机械臂坐在垫台上的那个面"。
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

from gripper_model import GripperModel

TABLE_Z_MEASURED = -0.06909


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--table-z', type=float, default=TABLE_Z_MEASURED)
    args = ap.parse_args()

    m = GripperModel(stride=1)

    print('base_link 各 visual 网格的 Z 范围（base_link 坐标系）：')
    lowest = None
    for vis in m.links['base_link'].findall('visual'):
        mesh = vis.find('geometry/mesh')
        if mesh is None:
            continue
        import os
        name = os.path.basename(mesh.get('filename'))
        o = vis.find('origin')
        t = np.array([float(v) for v in (o.get('xyz') or '0 0 0').split()])
        r = np.array([float(v) for v in (o.get('rpy') or '0 0 0').split()])
        pts = []
        from gripper_model import load_stl, rpy_matrix
        from pathlib import Path
        p = Path(m.assets) / name
        if not p.exists():
            continue
        pts = (rpy_matrix(*r) @ load_stl(p).T).T + t
        zmin, zmax = pts[:, 2].min(), pts[:, 2].max()
        print(f'  {name:<42} z {zmin*1000:+9.2f} ~ {zmax*1000:+9.2f} mm')
        if lowest is None or zmin < lowest:
            lowest = zmin

    # 支撑面 = **base_link 自身**的最低点，不能取全机最低点：
    # 零位姿态下夹爪是垂下来的（-104mm），那是悬空的末端，不是坐在垫台上的面。
    print()
    print('注意：要取的是 base_link 自身的最低点（机械臂的安装面），')
    print('      不是全机最低点 —— 零位姿态下夹爪垂到 -104mm，那是悬空的。')
    base_pts = m.link_points('base_link')
    mount_z = float(base_pts[:, 2].min())
    print(f'  base_link 最低点（安装面） = {mount_z*1000:+.2f} mm')

    print()
    print('=' * 64)
    print(f'  机械臂安装面（base_link 最低点）  = {mount_z*1000:+.2f} mm')
    print(f'  实测桌面高度                      = {args.table_z*1000:+.2f} mm')
    ped = mount_z - args.table_z
    print(f'  → 反推垫台高度                    = '
          f'{ped*1000:.2f} mm  ({ped:.5f} m)')
    print()
    print('  对照：孪生里旧值写的是 50 mm（由"最下端离桌面约 5cm"估算）')
    print(f'        实测反推 {ped*1000:.1f} mm，差 {(ped-0.05)*1000:+.1f} mm')
    print('=' * 64)
    return 0


if __name__ == '__main__':
    sys.exit(main())
