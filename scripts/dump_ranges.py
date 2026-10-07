#!/usr/bin/env python3
"""汇总 joint_ranges.json 的关键字段。"""

from project_paths import calibration_path
import json
import math

R = json.load(open(calibration_path('joint_ranges.json')))
CFG_MIN = [826, 842, 1974, 954, 1264, 1944]
CFG_MAX = [3276, 3118, 4061, 3116, 3885, 3168]
CFG_ZERO = [2078, 1980, 3076, 2035, 3053, 2030]
NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
         'wrist_roll', 'gripper']

print(f'采样 {R["samples"]} 次  读失败 {R["read_failures"]}  时间 {R["read_at"]}')
print()
print(f'{"关节":<14}{"实测lo":>8}{"实测hi":>8}{"行程":>7}{"行程°":>8}'
      f'{"中点":>7}{"现零点":>7}{"现限位":>15}{"当前零点URDF窗":>18}  12位')
print('-' * 96)
for i, n in enumerate(NAMES):
    j = R['joints'][n]
    lo = j['measured_min_unwrapped']
    hi = j['measured_max_unwrapped']
    if lo is None:
        print(f'{n:<14} 无数据')
        continue
    z = CFG_ZERO[i]
    need_lo = j['urdf_lower_rad'] * 4096 / math.tau
    need_hi = j['urdf_upper_rad'] * 4096 / math.tau
    w_lo, w_hi = z + need_lo, z + need_hi
    fits = 'OK' if (w_lo >= 0 and w_hi <= 4095) else f'超界({w_hi-4095:+.0f})'
    print(f'{n:<14}{lo:>8d}{hi:>8d}{hi-lo:>7d}{j["span_deg"]:>8.1f}'
          f'{(lo+hi)/2:>7.0f}{z:>7d}'
          f'{f"{CFG_MIN[i]}..{CFG_MAX[i]}":>15}'
          f'{f"{w_lo:.0f}..{w_hi:.0f}":>18}  {fits}')
    print(f'{"":>14}  错误位={j["error_flags_seen"]} 电压={j["voltage"]}V '
          f'温度={j["temperature_c"]}C 末载荷={j["final_load"]}')
