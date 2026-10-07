#!/bin/bash
# 启动打点脚本（正确零点 + 修正后的板面高度 -64.85mm）
cd /home/ros/QianLi/qianli_ws/src/qianli_vision/scripts || exit 1
pids=$(pgrep -f 'python3 -u remark_extrinsic' || true)
[ -n "$pids" ] && kill $pids 2>/dev/null
sleep 1
rm -f /tmp/grid_mark
setsid python3 -u remark_extrinsic.py \
  --table-z -0.06485 \
  --json /tmp/extrinsic_marks_new.json \
  --out /tmp/extrinsic_new.txt \
  > /tmp/remark4.log 2>&1 < /dev/null &
sleep 7
echo "--- 启动状态 ---"
grep -E '零点|限位|第 1/5|板面' /tmp/remark4.log | tail -6
pgrep -f 'python3 -u remark_extrinsic' >/dev/null && echo "运行中" || echo "未运行"
