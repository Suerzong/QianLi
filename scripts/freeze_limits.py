#!/usr/bin/env python3
"""冻结实测机械限位 → driver_params.yaml（src + install 两份）+ 冻结记录。

- raw_min/raw_max = 实测机械行程 ± 余量（余量在行程内侧，避免顶死点）
- 没扫到的关节（行程 < 阈值）保留原限位并标红
- 不改 zero_raw（改零点会改变 FK 零位，使已标定的棋盘外参失效；
  零点偏差单独报告，作为后续建议）
"""
import argparse
import json
import os
import re
import shutil
import sys
import time

RANGES = '/tmp/joint_ranges_merged.json' if os.path.exists(
    '/tmp/joint_ranges_merged.json') else '/tmp/joint_ranges.json'
YAMLS = [
    os.path.expanduser('~/legacy/arm/arm-final/ros2_ws/src/so101_bringup'
                       '/config/driver_params.yaml'),
    os.path.expanduser('~/legacy/arm/arm-final/ros2_ws/install/so101_bringup'
                       '/share/so101_bringup/config/driver_params.yaml'),
]
OUT = os.path.expanduser('~/QianLi/qianli_ws/config/limits_frozen.json')
NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
         'wrist_roll', 'gripper']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--margin', type=int, default=25,
                    help='行程内侧余量（计数，1 计数≈0.088°）')
    ap.add_argument('--min-span', type=int, default=200,
                    help='小于此行程视为"没扫到"')
    ap.add_argument('--apply', action='store_true')
    a = ap.parse_args()

    rep = json.load(open(RANGES))
    yaml_path = None
    for p in YAMLS:
        if os.path.exists(p):
            yaml_path = p
            break
    if yaml_path is None:
        print('找不到 driver_params.yaml')
        return 1
    txt = open(yaml_path).read()
    m_min = re.search(r'^(\s*)raw_min:\s*\[([^\]]*)\]', txt, re.M)
    m_max = re.search(r'^(\s*)raw_max:\s*\[([^\]]*)\]', txt, re.M)
    m_zero = re.search(r'^(\s*)zero_raw:\s*\[([^\]]*)\]', txt, re.M)
    if not (m_min and m_max and m_zero):
        print('yaml 里找不到 raw_min/raw_max/zero_raw')
        return 1
    cur_min = [int(v) for v in m_min.group(2).split(',')]
    cur_max = [int(v) for v in m_max.group(2).split(',')]
    zeros = [int(v) for v in m_zero.group(2).split(',')]

    new_min, new_max, notes = [], [], []
    print(f'{"关节":<14}{"实测行程":>16}{"现限位":>16}{"新限位":>16}  说明')
    print('-' * 80)
    for i, n in enumerate(NAMES):
        j = rep['joints'][n]
        lo = j['measured_min_unwrapped']
        hi = j['measured_max_unwrapped']
        span = 0 if lo is None else hi - lo
        if lo is None or span < a.min_span:
            new_min.append(cur_min[i])
            new_max.append(cur_max[i])
            notes.append(f'{n}: 未扫到(行程{span})，保留原限位')
            tag = f'⚠ 未扫到(行程{span})，保留原值'
            print(f'{n:<14}{str(lo)+".."+str(hi):>16}'
                  f'{f"{cur_min[i]}..{cur_max[i]}":>16}'
                  f'{f"{cur_min[i]}..{cur_max[i]}":>16}  {tag}')
            continue
        nl = max(0, lo + a.margin)
        nh = min(4095, hi - a.margin)
        new_min.append(nl)
        new_max.append(nh)
        # 零点偏差（仅对 URDF 对称行程的关节有意义）
        urdf_lo, urdf_hi = j['urdf_lower_rad'], j['urdf_upper_rad']
        sym = abs(urdf_lo + urdf_hi) < 1e-6
        mid = (lo + hi) / 2.0
        dz = zeros[i] - mid
        note = ''
        if sym:
            note = f'零点偏 {dz:+.0f} 计数 ({dz*360/4096:+.1f}°)'
            if abs(dz) > 40:
                note += ' ❌'
        print(f'{n:<14}{f"{lo}..{hi}":>16}'
              f'{f"{cur_min[i]}..{cur_max[i]}":>16}'
              f'{f"{nl}..{nh}":>16}  {note}')
        if note:
            notes.append(f'{n}: {note}')
    print()
    print('对比（可用量程变化，计数）:')
    for i, n in enumerate(NAMES):
        d_lo = new_min[i] - cur_min[i]
        d_hi = new_max[i] - cur_max[i]
        if d_lo or d_hi:
            print(f'  {n:<14} 下限 {d_lo:+5d}  上限 {d_hi:+5d}  '
                  f'总行程 {(new_max[i]-new_min[i]) - (cur_max[i]-cur_min[i]):+5d}')

    if not a.apply:
        print('\nPLAN ONLY（加 --apply 才写入）')
        return 0

    stamp = time.strftime('%Y-%m-%d %H:%M:%S')
    for p in YAMLS:
        if not os.path.exists(p):
            continue
        t = open(p).read()
        t = re.sub(r'^(\s*)raw_min:\s*\[[^\]]*\]',
                   lambda mm: f'{mm.group(1)}raw_min: [{", ".join(map(str, new_min))}]',
                   t, count=1, flags=re.M)
        t = re.sub(r'^(\s*)raw_max:\s*\[[^\]]*\]',
                   lambda mm: f'{mm.group(1)}raw_max: [{", ".join(map(str, new_max))}]',
                   t, count=1, flags=re.M)
        shutil.copy2(p, p + f'.bak_{time.strftime("%Y%m%d_%H%M%S")}')
        open(p, 'w').write(t)
        print(f'已写入 {p}（备份 .bak_*）')

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump({'frozen_at': stamp,
               'source': RANGES,
               'sampled_at': rep['read_at'],
               'samples': rep['samples'],
               'margin_counts': a.margin,
               'names': NAMES,
               'raw_min': new_min, 'raw_max': new_max,
               'zero_raw_unchanged': zeros,
               'measured': {n: [rep['joints'][n]['measured_min_unwrapped'],
                                rep['joints'][n]['measured_max_unwrapped']]
                            for n in NAMES},
               'notes': notes,
               'warning': 'zero_raw 未改动；实测中点与零点的偏差见 notes，'
                          '修正零点会使已标定的棋盘外参失效，需重做外参'},
              open(OUT, 'w'), indent=2, ensure_ascii=False)
    print(f'✅ 冻结记录 → {OUT}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
