#!/usr/bin/env python3
"""合并：用 /tmp/extrinsic_redo4.json 的新 (9.9,6.6) 点替换原始 5 点中的旧点，
输出合并后的 marks，供 --solve-only 重新求解。
"""

from project_paths import calibration_path
import json

orig = json.load(open(calibration_path('extrinsic_marks.json')))
redo = json.load(open(calibration_path('extrinsic_redo4.json')))

new_point = redo[0]
assert tuple(new_point['grid_cm']) == (9.9, 6.6)

merged = []
replaced = False
for m in orig:
    if tuple(m['grid_cm']) == (9.9, 6.6):
        merged.append(new_point)
        replaced = True
    else:
        merged.append(m)
if not replaced:
    merged.append(new_point)
    print('⚠️ 原始 marks 里没有 (9.9,6.6)，已追加')

json.dump(merged, open(calibration_path('extrinsic_marks_merged.json'), 'w'), indent=2)
print(f'合并完成: {len(merged)} 个点')
for m in merged:
    c = m['contact_m']
    print(f'  grid {m["grid_cm"]!s:>12} -> ({c[0]:.4f}, {c[1]:.4f}, {c[2]:.4f})'
          + ('  ← 新' if m is new_point else ''))
