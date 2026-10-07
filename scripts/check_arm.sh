#!/bin/bash

QI_PROJECT_ROOT="${QI_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
export QI_PROJECT_ROOT
source "$QI_PROJECT_ROOT/scripts/setup/source_env.sh" || exit 1
cd "$QI_PROJECT_ROOT/scripts" || exit 1
for i in 1 2 3; do
  OUT=$(python3 "$QI_PROJECT_ROOT/scripts/servo_status.py" 2>&1 | grep -a 'TCP(gripper')
  if [ -n "$OUT" ]; then
    echo "第${i}次成功: $OUT"
    python3 "$QI_PROJECT_ROOT/scripts/servo_status.py" 2>&1 | grep -aE '夹爪载荷|开合|最低点' | head -3
    exit 0
  fi
  echo "第${i}次: 串口超时"
  sleep 2
done
echo "串口连续失败"
exit 1
