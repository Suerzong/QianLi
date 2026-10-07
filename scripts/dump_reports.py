#!/usr/bin/env python3
"""汇总 setdown / regrasp 报告。"""
import json

for name in ('/tmp/quick_setdown.json', '/tmp/quick_regrasp.json'):
    print('=' * 60)
    print(name)
    try:
        d = json.load(open(name))
    except Exception as exc:
        print('  读取失败:', exc)
        continue
    for k in ('mode', 'completed', 'error', 'start_lowest_mm',
              'final_lowest_mm', 'start_load_pct', 'end_load_pct',
              'released', 'start_gripper_rad', 'held_gripper_rad',
              'contact_load_pct', 'measured_lift_mm'):
        if k in d:
            v = d[k]
            if isinstance(v, float):
                v = round(v, 3)
            print(f'  {k}: {v}')
    if 'start_tcp_m' in d:
        print('  start_tcp_m:', [round(v, 4) for v in d['start_tcp_m']])
    if 'end_tcp_m' in d:
        print('  end_tcp_m:', [round(v, 4) for v in d['end_tcp_m']])
    steps = d.get('descent_steps', [])
    print(f'  下探步数: {len(steps)}')
    for i, s in enumerate(steps):
        print(f'    步{i}: lowest={s["lowest_z_mm"]:.2f}mm tcp={s["tcp_z_mm"]:.2f}mm')
