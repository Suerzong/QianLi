#!/usr/bin/env python3
"""标定会话状态汇总：当前进度 + 最近判定 + 下一个待打点。

用户只拖动，其余指令从这里来。
"""

from project_paths import calibration_path
import json
import os
import re

LOG = '/tmp/ext_calib.log'
MARKS = calibration_path('extrinsic_marks.json')
DEFAULT_POINTS = [(0.0, 0.0), (9.9, 0.0), (0.0, 6.6), (9.9, 6.6), (3.3, 3.3)]

marks = []
if os.path.exists(MARKS):
    try:
        with open(MARKS) as fh:
            marks = json.load(fh)
        # 去掉旧标定的点（如果 mtime 早于脚本启动）
    except Exception:
        marks = []

print('=' * 56)
print('  标定会话状态')
print('=' * 56)

if not os.path.exists(LOG):
    print('❌ 日志不存在——脚本没跑起来？')
    raise SystemExit(1)

raw = open(LOG, encoding='utf-8', errors='replace').read()
lines = [ln for ln in raw.splitlines()
         if 'Warning' not in ln and 'warn(' not in ln]

# 当前提示点
current = None
for ln in reversed(lines):
    m = re.search(r'第 (\d+)/(\d+) 点.*?grid \(([-\d.]+), ([-\d.]+)\)', ln)
    if m:
        current = (int(m.group(1)), int(m.group(2)),
                   float(m.group(3)), float(m.group(4)))
        break

# 最近判定（✅/⛔ 行）
verdicts = [ln for ln in lines if ('✅' in ln or '⛔' in ln)]
recent = verdicts[-4:] if verdicts else []

done = [p for p in marks] if marks else []
print(f'已记录点: {len(done)}')
for d in done:
    print(f'   grid {d["grid_cm"]}  顶端 ({d["contact_m"][0]:.4f}, '
          f'{d["contact_m"][1]:.4f}, {d["contact_m"][2]:.4f})')

if current:
    idx, total, gx, gy = current
    remaining = [p for p in DEFAULT_POINTS[idx-1:] if p not in
                 [tuple(m['grid_cm']) for m in done]]
    print(f'\n▶ 当前该打: 第 {idx}/{total} 点  grid ({gx:.1f}, {gy:.1f}) cm')
    if recent:
        print('  最近判定:')
        for r in recent:
            print('   ' + r.strip()[:100])
    if remaining:
        nxt = remaining[0]
        if (gx, gy) != nxt:
            print(f'  （此点已记录？下一个待打: grid ({nxt[0]:.1f}, {nxt[1]:.1f})）')
else:
    print('▶ 未找到活动提示（脚本可能已结束或卡住）')
print('=' * 56)
