#!/bin/bash

QI_PROJECT_ROOT="${QI_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
export QI_PROJECT_ROOT
source "$QI_PROJECT_ROOT/scripts/setup/source_env.sh" || exit 1
# 重开拖拽记录，用于量 y 方向（竖着走）
pids=$(pgrep -f 'drag_record' || true)
[ -n "$pids" ] && kill $pids 2>/dev/null
sleep 1
cd "$QI_PROJECT_ROOT/scripts" || exit 1
rm -f /tmp/drag_stop
setsid python3 -u "$QI_PROJECT_ROOT/scripts/drag_record.py" --record --duration 90 --start-delay 6 \
  --json /tmp/drag_line2.json > /tmp/drag2.log 2>&1 < /dev/null &
sleep 9
grep -vE 'UserWarning|warnings.warn' /tmp/drag2.log | tail -4
