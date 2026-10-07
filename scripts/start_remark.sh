#!/bin/bash

QI_PROJECT_ROOT="${QI_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
export QI_PROJECT_ROOT
source "$QI_PROJECT_ROOT/scripts/setup/source_env.sh" || exit 1
# 启动打点脚本（读回滚后的零点）
cd "$QI_PROJECT_ROOT/scripts" || exit 1
rm -f /tmp/grid_mark
setsid python3 -u "$QI_PROJECT_ROOT/scripts/remark_extrinsic.py" \
  --json "$QI_CALIB_DIR/extrinsic_marks_new.json" \
  --out "$QI_CALIB_DIR/extrinsic_new.txt" \
  > /tmp/remark3.log 2>&1 < /dev/null &
sleep 7
echo "--- 启动状态 ---"
grep -E '零点|限位|第 1/5|板面' /tmp/remark3.log | tail -6
echo "--- 进程 ---"
pgrep -f '[r]emark_extrinsic.py' >/dev/null && echo "运行中" || echo "未运行"
