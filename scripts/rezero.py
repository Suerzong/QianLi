#!/usr/bin/env python3
"""修正零点：让实测机械行程与 URDF 行程对齐。

原理：joint_angle = (raw - zero) * dir * 2π/4096。
要让 URDF 的 [lo,hi] 正好落在实测机械行程 [mlo,mhi] 上：
    zero = mlo - lo*4096/2π   （由下限推）
    zero = mhi - hi*4096/2π   （由上限推）
两者取平均（实测行程与 URDF 行程略有差异时取最小二乘中值）。
对称 URDF 行程的关节（±x）等价于 zero = 实测中点。
"""
import argparse
import json
import os
import re
import shutil
import sys
import time

K = 4096.0 / (2 * 3.141592653589793)     # counts per rad
RANGES = '/tmp/joint_ranges_merged.json'
YAMLS = [
    os.path.expanduser('~/legacy/arm/arm-final/ros2_ws/src/so101_bringup'
                       '/config/driver_params.yaml'),
    os.path.expanduser('~/legacy/arm/arm-final/ros2_ws/install/so101_bringup'
                       '/share/so101_bringup/config/driver_params.yaml'),
]
OUT = os.path.expanduser('~/QianLi/qianli_ws/config/zero_frozen.json')
NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
         'wrist_roll', 'gripper']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--apply', action='store_true')
    ap.add_argument('--skip', default='gripper',
                    help='不改零点的关节（逗号分隔）')
    a = ap.parse_args()
    skip = {s.strip() for s in a.skip.split(',') if s.strip()}

    rep = json.load(open(RANGES))
    yaml_path = next(p for p in YAMLS if os.path.exists(p))
    txt = open(yaml_path).read()
    m_zero = re.search(r'^(\s*)zero_raw:\s*\[([^\]]*)\]', txt, re.M)
    old = [int(v) for v in m_zero.group(2).split(',')]

    new, rows = [], []
    print(f'{"关节":<15}{"实测机械行程":>16}{"URDF行程(计数)":>17}'
          f'{"旧零点":>8}{"新零点":>8}{"偏移":>7}  URDF窗(新)')
    print('-' * 92)
    for i, n in enumerate(NAMES):
        j = rep['joints'][n]
        mlo = j['measured_min_unwrapped']
        mhi = j['measured_max_unwrapped']
        lo, hi = j['urdf_lower_rad'], j['urdf_upper_rad']
        z_fit = ((mlo - lo * K) + (mhi - hi * K)) / 2.0
        z = old[i] if n in skip else int(round(z_fit))
        new.append(z)
        w_lo, w_hi = z + lo * K, z + hi * K
        fits = 0 <= w_lo and w_hi <= 4095
        inside = (w_lo >= mlo - 1) and (w_hi <= mhi + 1)
        tag = ('OK' if (fits and inside) else
               ('超12位' if not fits else '超出实测行程'))
        print(f'{n:<15}{f"{mlo}..{mhi}":>16}'
              f'{f"{lo*K:.0f}..{hi*K:.0f}":>17}'
              f'{old[i]:>8}{z:>8}{z-old[i]:>+7}  '
              f'{f"{w_lo:.0f}..{w_hi:.0f}":>16} {tag}')
        rows.append({'joint': n, 'measured': [mlo, mhi],
                     'urdf_counts': [round(lo * K), round(hi * K)],
                     'old_zero': old[i], 'new_zero': z,
                     'zero_shift': z - old[i],
                     'urdf_window_new': [round(w_lo), round(w_hi)],
                     'fits_12bit': bool(fits), 'window_inside_measured': bool(inside),
                     'skipped': n in skip})

    if not a.apply:
        print('\nPLAN ONLY（--apply 才写入）')
        return 0

    stamp = time.strftime('%Y-%m-%d %H:%M:%S')
    for p in YAMLS:
        if not os.path.exists(p):
            continue
        t = open(p).read()
        t = re.sub(r'^(\s*)zero_raw:\s*\[[^\]]*\]',
                   lambda mm: f'{mm.group(1)}zero_raw: [{", ".join(map(str, new))}]',
                   t, count=1, flags=re.M)
        shutil.copy2(p, p + f'.zerobak_{time.strftime("%Y%m%d_%H%M%S")}')
        open(p, 'w').write(t)
        print(f'已写入 zero_raw → {p}')
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump({'frozen_at': stamp, 'source': RANGES, 'old_zero': old,
               'new_zero': new, 'per_joint': rows,
               'note': '零点已按"实测机械行程 ↔ URDF 行程"对齐；'
                       '改零点后 FK 角度变化，棋盘外参必须重做'},
              open(OUT, 'w'), indent=2, ensure_ascii=False)
    print(f'✅ 零点冻结记录 → {OUT}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
