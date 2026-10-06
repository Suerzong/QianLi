#!/usr/bin/env python3
"""把"手动扫行程"实测结果换算成正确的软限位 / 零点方案（纯离线，不碰硬件）

输入：``joint_range_calibrate.py`` 产出的 ``/tmp/joint_ranges.json``
输出：报告 + 可直接使用的 ``driver_params.yaml`` 片段

限位来源的判定规则（这里是本工具的全部价值）
--------------------------------------------
1. **机械死点** = 实测极值，是硬件事实。
2. **URDF 限位** = 官方模型的设计意图（防自碰撞 / 防线缆过扭），是语义事实。
3. 正确的软限位 = ``min(设计意图, 机械死点 − 安全边距)``：
   * URDF 比机械死点更保守 → 听 URDF（设计者故意留的余量，不是 bug）
   * URDF 比机械死点更宽 → 必须收窄到机械死点，否则软件会命令舵机顶死
4. 若换算出的窗口超出舵机 12 位量程 ``[0, 4095]``，说明**零点位置不对**，
   需要改舵机 Homing_Offset 把行程挪到量程中间（lerobot 的 set_half_turn_homings
   就是干这个的），而不是把 raw_max 截断到 4095 —— 那正是"吞掉行程"的根源。

用法::

    ~/mj/bin/python derive_joint_limits.py --ranges /tmp/joint_ranges.json
"""

from __future__ import annotations

import argparse
import json
import math
import sys

JOINT_NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
               'wrist_roll', 'gripper']

CURRENT_ZERO = [2078, 1980, 3076, 2035, 3053, 2030]
CURRENT_MIN = [826, 842, 1974, 954, 1264, 1916]
CURRENT_MAX = [3330, 3118, 4095, 3116, 4095, 3168]

# 官方 so101_new_calib.urdf 的关节限位（rad）
URDF = {
    'shoulder_pan':  (-1.91986, 1.91986),
    'shoulder_lift': (-1.74533, 1.74533),
    'elbow_flex':    (-1.69, 1.69),
    'wrist_flex':    (-1.65806, 1.65806),
    'wrist_roll':    (-2.74385, 2.84121),
    'gripper':       (-0.174533, 1.74533),
}

COUNT = 4096.0
DEG_PER_COUNT = 360.0 / COUNT
RAD_PER_COUNT = math.tau / COUNT


def c2deg(c):
    return c * DEG_PER_COUNT


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ranges', default='/tmp/joint_ranges.json')
    ap.add_argument('--margin-deg', type=float, default=2.0,
                    help='机械死点侧的安全边距（度），默认 2°')
    ap.add_argument('--json', default='/tmp/joint_limits_derived.json')
    ap.add_argument('--yaml', default='/tmp/driver_params.proposed.yaml')
    args = ap.parse_args()

    with open(args.ranges) as fh:
        sweep = json.load(fh)

    margin = args.margin_deg / DEG_PER_COUNT
    plan = {}
    print('=' * 100)
    print(f'  软限位推导（机械死点侧安全边距 {args.margin_deg:.1f}° = {margin:.0f} 计数）')
    print('=' * 100)
    print(f'{"关节":<14}{"机械死点":>18}{"URDF 换算":>18}'
          f'{"正确软限位(取窄)":>20}{"现软限位":>18}  结论')
    print('-' * 100)

    for i, name in enumerate(JOINT_NAMES):
        entry = sweep['joints'][name]
        zero = CURRENT_ZERO[i]
        mech_lo = entry['measured_min_unwrapped']
        mech_hi = entry['measured_max_unwrapped']
        urdf_lo_deg = math.degrees(URDF[name][0])
        urdf_hi_deg = math.degrees(URDF[name][1])
        urdf_lo_raw = zero + URDF[name][0] / RAD_PER_COUNT
        urdf_hi_raw = zero + URDF[name][1] / RAD_PER_COUNT

        # 正确软限位：两侧各取"更窄"的那个
        good_lo = max(urdf_lo_raw, mech_lo + margin)
        good_hi = min(urdf_hi_raw, mech_hi - margin)

        # 需要重新定零吗？（窗口要装进 12 位）
        need_shift = None
        if good_lo < 0 or good_hi > 4095:
            # 把机械行程居中到 2048；位移取整，好和实际写进 EEPROM 的值一致
            mid = (mech_lo + mech_hi) / 2.0
            shift = float(round(2048.0 - mid))
            need_shift = shift
            good_lo += shift
            good_hi += shift

        lo_bound = ('URDF' if urdf_lo_raw > mech_lo + margin else '机械')
        hi_bound = ('URDF' if urdf_hi_raw < mech_hi - margin else '机械')

        notes = []
        if abs(good_lo - CURRENT_MIN[i]) > 12 or abs(good_hi - CURRENT_MAX[i]) > 12:
            notes.append('需改')
        if need_shift is not None:
            notes.append(f'需重定零 {need_shift:+.0f}')
        if CURRENT_MIN[i] < mech_lo or CURRENT_MAX[i] > mech_hi:
            notes.append('现限位越过死点!')
        if not notes:
            notes.append('OK')

        print(f'{name:<14}'
              f'{mech_lo:>8.0f}~{mech_hi:<9.0f}'
              f'{urdf_lo_raw:>8.0f}~{urdf_hi_raw:<9.0f}'
              f'{good_lo:>9.0f}~{good_hi:<10.0f}'
              f'{CURRENT_MIN[i]:>8d}~{CURRENT_MAX[i]:<9d}'
              f'  {lo_bound}下/{hi_bound}上  ' + ','.join(notes))

        plan[name] = {
            'mech_lo_raw': mech_lo,
            'mech_hi_raw': mech_hi,
            'mech_lo_deg': c2deg(mech_lo - zero),
            'mech_hi_deg': c2deg(mech_hi - zero),
            'urdf_lo_raw': round(urdf_lo_raw, 1),
            'urdf_hi_raw': round(urdf_hi_raw, 1),
            'good_raw_min': round(good_lo),
            'good_raw_max': round(good_hi),
            'good_lo_deg': c2deg(good_lo - zero - (need_shift or 0)),
            'good_hi_deg': c2deg(good_hi - zero - (need_shift or 0)),
            'lower_bound_by': lo_bound,
            'upper_bound_by': hi_bound,
            'homing_shift_counts': None if need_shift is None else round(need_shift),
            'current_raw_min': CURRENT_MIN[i],
            'current_raw_max': CURRENT_MAX[i],
            'current_zero_raw': zero,
            'new_zero_raw': round(zero + need_shift) if need_shift else zero,
            'span_deg': c2deg(mech_hi - mech_lo),
        }

    print()
    print('=' * 100)
    print('  结论')
    print('=' * 100)
    rezero = {n: p for n, p in plan.items() if p['homing_shift_counts'] is not None}
    if rezero:
        print('需要改 Homing_Offset 重新定零的关节：')
        for name, p in rezero.items():
            print(f'  · {name}: 平移 {p["homing_shift_counts"]:+d} 计数 '
                  f'({c2deg(p["homing_shift_counts"]):+.1f}°)，'
                  f'zero_raw {p["current_zero_raw"]} → {p["new_zero_raw"]}')
    else:
        print('无需重新定零')

    over = [n for i, n in enumerate(JOINT_NAMES)
            if CURRENT_MIN[i] < plan[n]['mech_lo_raw']
            or CURRENT_MAX[i] > plan[n]['mech_hi_raw']]
    if over:
        print('现有软限位越过了机械死点（会顶死舵机）：'
              + ', '.join(over))

    narrow = [n for i, n in enumerate(JOINT_NAMES)
              if (plan[n]['good_raw_max'] - plan[n]['good_raw_min'])
              > (CURRENT_MAX[i] - CURRENT_MIN[i]) + 12]
    if narrow:
        print('修正后会明显变宽的关节：')
        for n in narrow:
            i = JOINT_NAMES.index(n)
            old = CURRENT_MAX[i] - CURRENT_MIN[i]
            new = plan[n]['good_raw_max'] - plan[n]['good_raw_min']
            print(f'  · {n}: {c2deg(old):.1f}° → {c2deg(new):.1f}°  '
                  f'(+{c2deg(new - old):.1f}°)')

    # ---------------- 产出新配置 ----------------
    zero = []
    rmin = []
    rmax = []
    for name in JOINT_NAMES:
        p = plan[name]
        zero.append(p['new_zero_raw'])
        rmin.append(p['good_raw_min'])
        rmax.append(p['good_raw_max'])

    lines = [
        '    # ---- 机械限位：由 2026-10-06 手动扫行程实测得出 ----',
        '    # 依据：每个关节用手推到两个机械死点，记录 Present_Position 极值。',
        '    # 旧值来自 lerobot 标定文件 my_so101_arm.json，那份文件是"照着 URDF',
        '    # 摆位"录的（shoulder_pan 826~3330 与 URDF 换算值完全一致，',
        '    # wrist_roll 直接是 0~4095 默认值），并非机械行程。',
        f'    # 软限位 = min(URDF 设计意图, 机械死点 - {args.margin_deg:.1f}°)。',
        f'    zero_raw: {zero}',
        '    direction: [1, 1, 1, 1, 1, 1]',
        f'    raw_min: {rmin}',
        f'    raw_max: {rmax}',
    ]
    with open(args.yaml, 'w') as fh:
        fh.write('\n'.join(lines) + '\n')
    print()
    print('建议的 driver_params.yaml 片段：')
    print('\n'.join(lines))

    with open(args.json, 'w') as fh:
        json.dump({'margin_deg': args.margin_deg, 'plan': plan,
                   'proposed': {'zero_raw': zero, 'raw_min': rmin,
                                'raw_max': rmax}},
                  fh, indent=2, ensure_ascii=False)
    print()
    print(f'📄 {args.json}')
    print(f'📄 {args.yaml}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
