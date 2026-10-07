#!/usr/bin/env python3
"""合并两次扫行程结果：主报告取全关节，wrist_roll 用补扫报告。"""
import json
import shutil

main = json.load(open('/tmp/joint_ranges.json'))
wr = json.load(open('/tmp/joint_ranges_wr.json'))

j = wr['joints']['wrist_roll']
lo = j['measured_min_unwrapped']
hi = j['measured_max_unwrapped']
print(f'补扫 wrist_roll: {lo}..{hi} (行程 {hi-lo} 计数 = {j["span_deg"]:.1f}°)')
if lo is None or hi - lo < 200:
    print('补扫无效，保持原值')
    raise SystemExit(1)

main['joints']['wrist_roll'] = j
main.setdefault('merged_from', []).append({
    'wrist_roll_source': '/tmp/joint_ranges_wr.json',
    'sampled_at': wr['read_at'], 'samples': wr['samples']})
main['merged_at'] = wr['read_at']
json.dump(main, open('/tmp/joint_ranges_merged.json', 'w'), indent=2,
          ensure_ascii=False)
print('已写 /tmp/joint_ranges_merged.json')
for n, e in main['joints'].items():
    lo, hi = e['measured_min_unwrapped'], e['measured_max_unwrapped']
    print(f'  {n:<15} {lo}..{hi}  行程 {hi-lo if lo is not None else 0}')
