#!/bin/bash
# 停止前端（释放串口/相机）并跑一次状态机
pids=$(pgrep -f 'target_viewer' || true)
if [ -n "$pids" ]; then kill $pids 2>/dev/null; echo "已停前端: $pids"; else echo "前端未运行"; fi
sleep 1
cd /home/ros/QianLi/qianli_ws/src/qianli_vision/scripts || exit 1
timeout 290 python3 -u grasp_fsm.py \
  --x 0.2280 --y -0.1315 --half-mm 20 --clearance-mm 6 \
  --tip-z-mm -56 --hover-mm 50 --lift-mm 40 > /tmp/fsm7.log 2>&1
echo "--- 结果 ---"
grep -vE 'UserWarning|warnings.warn' /tmp/fsm7.log | tail -26
