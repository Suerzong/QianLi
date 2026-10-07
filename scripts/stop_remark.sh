#!/bin/bash
# 停止旧的打点脚本（按 PID，避免 pkill -f 自匹配）
pids=$(pgrep -f '[r]emark_extrinsic.py' || true)
if [ -n "$pids" ]; then
  kill $pids 2>/dev/null
  echo "已停止 PID: $pids"
else
  echo "没有在跑的打点脚本"
fi
sleep 1
