#!/bin/bash

QI_PROJECT_ROOT="${QI_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
export QI_PROJECT_ROOT
source "$QI_PROJECT_ROOT/scripts/setup/source_env.sh" || exit 1
# 自动抓取：定位 -> 选目标 -> 跑状态机
cd "$QI_PROJECT_ROOT/scripts" || exit 1
echo "=== 定位 ==="
python3 "$QI_PROJECT_ROOT/scripts/board_frame.py" --locate --color yellow 2>&1 \
  | grep -vE 'Warning|warn|VIDIOC|obsensor' | tail -6
echo
echo "=== 选目标 ==="
SEL=$(python3 "$QI_PROJECT_ROOT/scripts/pick_block.py" --color yellow 2>&1)
echo "$SEL"
XY=$(echo "$SEL" | awk '{print $1, $2}')
set -- $XY
if [ -z "$1" ]; then echo "没选到目标，退出"; exit 1; fi
echo "目标 base = ($1, $2)"
echo
echo "=== 状态机 ==="
timeout 290 python3 -u "$QI_PROJECT_ROOT/scripts/grasp_fsm.py" --x "$1" --y "$2" \
  --half-mm 20 --clearance-mm 6 --tip-z-mm -46 --hover-mm 50 --lift-mm 40 \
  > /tmp/fsm_auto.log 2>&1
grep -vE 'UserWarning|warnings.warn' /tmp/fsm_auto.log | tail -26
