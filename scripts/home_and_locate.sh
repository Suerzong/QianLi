#!/bin/bash
# 折回臂 -> 重新定位方块
cd /home/ros/QianLi/qianli_ws/src/qianli_vision/scripts || exit 1
echo "=== 折回 home ==="
timeout 120 python3 move_retry.py --q 0.0276 -1.8009 1.5156 -1.7978 0.5476 0.4541 2>&1 \
  | grep -vE 'UserWarning|warnings.warn' | tail -2
echo
echo "=== 重新定位（所有颜色）==="
python3 board_frame.py --locate 2>&1 | grep -vE 'Warning|warn|VIDIOC|obsensor' | tail -12
