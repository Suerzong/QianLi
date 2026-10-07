#!/bin/bash
# 使能力矩 -> 回到折叠 baseline -> 让开棋盘
cd /home/ros/QianLi/qianli_ws/src/qianli_vision/scripts || exit 1
echo "=== 使能力矩 ==="
python3 servo_torque.py --on 2>&1 | grep -vE 'UserWarning|warnings.warn' | tail -3
echo
echo "=== 回到折叠位（home_pose 的 raw_exec）==="
python3 move_joints.py --q 0.908 -1.800 0.883 -0.661 1.647 -0.155 --execute 2>&1 \
  | grep -vE 'UserWarning|warnings.warn' | tail -6
