#!/bin/bash
# 重启目标点可视化前端
pids=$(pgrep -f 'target_viewer' || true)
[ -n "$pids" ] && kill $pids 2>/dev/null
sleep 1
cd /home/ros/QianLi/qianli_ws/src/qianli_vision/scripts || exit 1
setsid ~/mj/bin/python target_viewer.py --port 8099 > /tmp/viewer.log 2>&1 < /dev/null &
sleep 6
echo "--- 日志 ---"
grep -vE 'UserWarning|warnings.warn' /tmp/viewer.log | tail -6
echo "--- 监听 ---"
ss -tlnp 2>/dev/null | grep 8099
echo "--- 取一次数据 ---"
curl -s --max-time 5 http://127.0.0.1:8099/data | head -c 400
