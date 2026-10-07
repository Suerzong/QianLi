#!/bin/bash

QI_PROJECT_ROOT="${QI_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
export QI_PROJECT_ROOT
source "$QI_PROJECT_ROOT/scripts/setup/source_env.sh" || exit 1
# 一条龙：归位 -> 立刻定位 -> 立刻抓取
cd "$QI_PROJECT_ROOT/scripts" || exit 1

echo "=== 1) 归位 ==="
timeout 150 python3 "$QI_PROJECT_ROOT/scripts/move_retry.py" --q 0.0276 -1.8009 1.5156 -1.7978 0.5476 0.4541 2>&1 \
  | grep -vE 'UserWarning|warnings.warn' | tail -2
sleep 1

echo
echo "=== 2) 立刻定位 ==="
python3 "$QI_PROJECT_ROOT/scripts/board_frame.py" --locate --color yellow 2>&1 \
  | grep -vE 'Warning|warn|VIDIOC|obsensor' | tail -8
SEL=$(python3 "$QI_PROJECT_ROOT/scripts/pick_block.py" --color yellow 2>/dev/null)
echo "选中: $SEL"
XY=$(echo "$SEL" | awk '{print $1, $2}')
set -- $XY
if [ -z "$1" ]; then echo "❌ 没选到方块，退出"; exit 1; fi
echo "目标 base = ($1, $2)"

echo
echo "=== 3) 抓取（间隙 8mm）==="
timeout 290 python3 -u "$QI_PROJECT_ROOT/scripts/grasp_fsm.py" --x "$1" --y "$2" \
  --half-mm 20 --clearance-mm 8 --tip-z-mm -46 --hover-mm 50 --lift-mm 40 \
  > /tmp/fsm_auto2.log 2>&1
grep -vE 'UserWarning|warnings.warn' /tmp/fsm_auto2.log | tail -24
