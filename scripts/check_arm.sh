#!/bin/bash
cd /home/ros/QianLi/qianli_ws/src/qianli_vision/scripts || exit 1
for i in 1 2 3; do
  OUT=$(python3 servo_status.py 2>&1 | grep -a 'TCP(gripper')
  if [ -n "$OUT" ]; then
    echo "第${i}次成功: $OUT"
    python3 servo_status.py 2>&1 | grep -aE '夹爪载荷|开合|最低点' | head -3
    exit 0
  fi
  echo "第${i}次: 串口超时"
  sleep 2
done
echo "串口连续失败"
exit 1
