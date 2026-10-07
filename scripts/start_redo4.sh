#!/bin/bash

QI_PROJECT_ROOT="${QI_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
export QI_PROJECT_ROOT
source "$QI_PROJECT_ROOT/scripts/setup/source_env.sh" || exit 1
# 重打第 4 点 (9.9,6.6)：容差放宽到 3.5mm，配合"压住不松手"触发
set -e
cd "$QI_PROJECT_ROOT/scripts" || exit 1
pkill -f 'extrinsic_calib_multi.py' 2>/dev/null || true
sleep 1
rm -f /tmp/grid_mark /tmp/redo4.log
nohup python3 -u "$QI_PROJECT_ROOT/qianli_ws/src/qianli_vision/scripts/extrinsic_calib_multi.py" --cell-cm 3.3 \
  --points "9.9,6.6" --json "$QI_CALIB_DIR/extrinsic_redo4.json" --z-tol-mm 3.5 \
  > /tmp/redo4.log 2>&1 &
echo "PID=$!"
sleep 8
grep -vE 'Warning|warn' /tmp/redo4.log | head -12 || true
