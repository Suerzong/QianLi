#!/bin/bash
# 探测 (0,0) 掩码重合度与当前方块位置
cd /home/ros/QianLi/qianli_ws/src/qianli_vision/scripts || exit 1
for t in 0.10 0.30 0.50 0.70 0.90; do
  printf '阈值 %s -> ' "$t"
  python3 check_at_00.py "$t" 2>/dev/null
done
echo "--- 当前方块定位 ---"
python3 board_frame.py --locate --color yellow 2>&1 \
  | grep -vE 'Warning|warn|VIDIOC|obsensor' | tail -4
exit 0
