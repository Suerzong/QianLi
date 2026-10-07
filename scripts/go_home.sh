#!/bin/bash

QI_PROJECT_ROOT="${QI_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
export QI_PROJECT_ROOT
source "$QI_PROJECT_ROOT/scripts/setup/source_env.sh" || exit 1
# 使能力矩 -> 回到折叠 baseline -> 让开棋盘
cd "$QI_PROJECT_ROOT/scripts" || exit 1
echo "=== 使能力矩 ==="
python3 "$QI_PROJECT_ROOT/scripts/servo_torque.py" --on 2>&1 | grep -vE 'UserWarning|warnings.warn' | tail -3
echo
echo "=== 回到折叠位（home_pose 的 raw_exec）==="
python3 "$QI_PROJECT_ROOT/scripts/move_joints.py" --q 0.908 -1.800 0.883 -0.661 1.647 -0.155 --execute 2>&1 \
  | grep -vE 'UserWarning|warnings.warn' | tail -6
