#!/bin/bash

QI_PROJECT_ROOT="${QI_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
export QI_PROJECT_ROOT
source "$QI_PROJECT_ROOT/scripts/setup/source_env.sh" || exit 1
# 触发当前标定点：touch /tmp/grid_mark -> 等脚本记录 -> 显示判定结果
# 用法: bash trigger_mark.sh
rm -f /tmp/grid_mark
touch /tmp/grid_mark
sleep 3
echo '--- 最近判定 ---'
grep -vE 'Warning|warn' /tmp/ext_calib.log | tail -12
echo '--- 已记录点数 ---'
python3 -c "import json; m=json.load(open(__import__('os').path.join(__import__('os').environ['QI_CALIB_DIR'], 'extrinsic_marks.json'))); print(len(m), '个点:', [p['grid_cm'] for p in m])" 2>/dev/null || echo '（尚无记录文件）'
