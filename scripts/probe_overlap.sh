#!/bin/bash

QI_PROJECT_ROOT="${QI_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
export QI_PROJECT_ROOT
source "$QI_PROJECT_ROOT/scripts/setup/source_env.sh" || exit 1
# 探测 (0,0) 掩码重合度与当前方块位置
cd "$QI_PROJECT_ROOT/scripts" || exit 1
for t in 0.10 0.30 0.50 0.70 0.90; do
  printf '阈值 %s -> ' "$t"
  python3 "$QI_PROJECT_ROOT/scripts/check_at_00.py" "$t" 2>/dev/null
done
echo "--- 当前方块定位 ---"
python3 "$QI_PROJECT_ROOT/scripts/board_frame.py" --locate --color yellow 2>&1 \
  | grep -vE 'Warning|warn|VIDIOC|obsensor' | tail -4
exit 0
