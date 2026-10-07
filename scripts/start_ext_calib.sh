#!/bin/bash

QI_PROJECT_ROOT="${QI_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
export QI_PROJECT_ROOT
source "$QI_PROJECT_ROOT/scripts/setup/source_env.sh" || exit 1
# 无缓冲后台启动外参标定脚本
set -e
cd "$QI_PROJECT_ROOT/scripts" || exit 1
# 杀掉旧进程
pkill -f 'extrinsic_calib_multi.py' 2>/dev/null || true
sleep 1
rm -f /tmp/ext_calib.log /tmp/grid_mark
nohup python3 -u "$QI_PROJECT_ROOT/qianli_ws/src/qianli_vision/scripts/extrinsic_calib_multi.py" --cell-cm 3.3 > /tmp/ext_calib.log 2>&1 &
echo "PID=$!"
sleep 8
echo '--- 日志 ---'
grep -vE 'Warning|warn' /tmp/ext_calib.log | head -40 || true
