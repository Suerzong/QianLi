#!/bin/bash

QI_PROJECT_ROOT="${QI_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
export QI_PROJECT_ROOT
source "$QI_PROJECT_ROOT/scripts/setup/source_env.sh" || exit 1
# 部署窗口化 (0,0) 检测并重启值守
bash /tmp/stop_all.sh
cd "$QI_PROJECT_ROOT/scripts" || exit 1
cp /tmp/check_at_00.py . || exit 1
rm -f "$QI_PROJECT_ROOT/config/at00_mask.png"
echo "重建参考掩码（当前 (0,0) 处应当有方块）："
python3 "$QI_PROJECT_ROOT/scripts/check_at_00.py" --reset
setsid bash watch.sh 2 0 > /tmp/watch_boot.log 2>&1 < /dev/null &
sleep 8
echo "--- 值守日志 ---"
tail -4 /tmp/watch.log
exit 0
