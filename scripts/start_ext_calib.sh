#!/bin/bash
# 无缓冲后台启动外参标定脚本
set -e
source /opt/ros/jazzy/setup.bash
source ~/legacy/arm/arm-final/ros2_ws/install/setup.bash 2>/dev/null || true
cd ~/QianLi/qianli_ws/src/qianli_vision/scripts
# 杀掉旧进程
pkill -f 'extrinsic_calib_multi.py' 2>/dev/null || true
sleep 1
rm -f /tmp/ext_calib.log /tmp/grid_mark
nohup python3 -u extrinsic_calib_multi.py --cell-cm 3.3 > /tmp/ext_calib.log 2>&1 &
echo "PID=$!"
sleep 8
echo '--- 日志 ---'
grep -vE 'Warning|warn' /tmp/ext_calib.log | head -40 || true
