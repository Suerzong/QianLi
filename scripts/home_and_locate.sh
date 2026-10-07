#!/bin/bash

QI_PROJECT_ROOT="${QI_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
export QI_PROJECT_ROOT
source "$QI_PROJECT_ROOT/scripts/setup/source_env.sh" || exit 1
# 折回臂 -> 重新定位方块
cd "$QI_PROJECT_ROOT/scripts" || exit 1
echo "=== 折回 home ==="
timeout 120 python3 "$QI_PROJECT_ROOT/scripts/move_retry.py" --q 0.0276 -1.8009 1.5156 -1.7978 0.5476 0.4541 2>&1 \
  | grep -vE 'UserWarning|warnings.warn' | tail -2
echo
echo "=== 重新定位（所有颜色）==="
python3 "$QI_PROJECT_ROOT/scripts/board_frame.py" --locate 2>&1 | grep -vE 'Warning|warn|VIDIOC|obsensor' | tail -12
