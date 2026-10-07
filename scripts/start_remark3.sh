#!/bin/bash

QI_PROJECT_ROOT="${QI_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
export QI_PROJECT_ROOT
source "$QI_PROJECT_ROOT/scripts/setup/source_env.sh" || exit 1
# 续打：保留已记录点，容差放宽到 3.5mm
cd "$QI_PROJECT_ROOT/scripts" || exit 1
pids=$(pgrep -f '[r]emark_extrinsic.py' || true)
[ -n "$pids" ] && kill $pids 2>/dev/null
sleep 1
rm -f /tmp/grid_mark
setsid python3 -u "$QI_PROJECT_ROOT/scripts/remark_extrinsic.py" \
  --table-z -0.06485 --z-tol-mm 3.5 --resume \
  --json "$QI_CALIB_DIR/extrinsic_marks_new.json" \
  --out "$QI_CALIB_DIR/extrinsic_new.txt" \
  > /tmp/remark5.log 2>&1 < /dev/null &
sleep 7
echo "--- 启动状态 ---"
grep -E '续打|第 .*/5|板面' /tmp/remark5.log | tail -6
pgrep -f '[r]emark_extrinsic.py' >/dev/null && echo "运行中" || echo "未运行"
