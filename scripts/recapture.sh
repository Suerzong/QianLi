#!/bin/bash
# 停值守 -> 重新检测棋盘(重建单应) -> 显示长期坐标系
bash /tmp/stop_all.sh
cd /home/ros/QianLi/qianli_ws/src/qianli_vision/scripts || exit 1
echo "=== 重新检测棋盘 ==="
python3 board_frame.py --capture 2>&1 \
  | grep -vE 'Warning|warn|VIDIOC|obsensor' | tail -10
echo
echo "=== 长期坐标系 ==="
python3 board_frame.py --show 2>&1 \
  | grep -vE 'Warning|warn|VIDIOC|obsensor' | tail -8
exit 0
