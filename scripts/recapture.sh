#!/bin/bash

QI_PROJECT_ROOT="${QI_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
export QI_PROJECT_ROOT
source "$QI_PROJECT_ROOT/scripts/setup/source_env.sh" || exit 1
# 停值守 -> 重新检测棋盘(重建单应) -> 显示长期坐标系
bash /tmp/stop_all.sh
cd "$QI_PROJECT_ROOT/scripts" || exit 1
echo "=== 重新检测棋盘 ==="
python3 "$QI_PROJECT_ROOT/scripts/board_frame.py" --capture 2>&1 \
  | grep -vE 'Warning|warn|VIDIOC|obsensor' | tail -10
echo
echo "=== 长期坐标系 ==="
python3 "$QI_PROJECT_ROOT/scripts/board_frame.py" --show 2>&1 \
  | grep -vE 'Warning|warn|VIDIOC|obsensor' | tail -8
exit 0
