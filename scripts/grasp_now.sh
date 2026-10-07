#!/bin/bash

QI_PROJECT_ROOT="${QI_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
export QI_PROJECT_ROOT
source "$QI_PROJECT_ROOT/scripts/setup/source_env.sh" || exit 1
# 停值守 -> 定位 -> 抓取一轮
bash /tmp/stop_all.sh >/dev/null 2>&1
cd "$QI_PROJECT_ROOT/scripts" || exit 1
echo "=== 定位 ==="
python3 "$QI_PROJECT_ROOT/scripts/board_frame.py" --locate --color yellow 2>&1 \
  | grep -vE 'Warning|warn|VIDIOC|obsensor' | tail -6
SEL=$(python3 "$QI_PROJECT_ROOT/scripts/pick_block.py" --color yellow 2>/dev/null)
echo "选中: $SEL"
X=$(echo "$SEL" | awk '{print $1}')
case "$X" in
  ''|NONE|*[!0-9.-]*) echo "❌ 棋盘上没找到方块，不执行"; exit 1 ;;
esac
echo
echo "=== 抓取（y 偏置 -20mm）==="
bash auto_cycle.sh 2>&1 | grep -aE '选中|目标 base|几何|偏置|最终|悬停|抓取位|READY|路径|落到底|合爪|继续压|抬升|保持|放置|放到|回中间|腕部|归位|held|contact|total_offset|load_after|夹住|滑脱|error' | tail -20
exit 0
