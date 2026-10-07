#!/bin/bash
# 续打：保留已记录点，容差放宽到 3.5mm
cd /home/ros/QianLi/qianli_ws/src/qianli_vision/scripts || exit 1
pids=$(pgrep -f 'python3 -u remark_extrinsic' || true)
[ -n "$pids" ] && kill $pids 2>/dev/null
sleep 1
rm -f /tmp/grid_mark
setsid python3 -u remark_extrinsic.py \
  --table-z -0.06485 --z-tol-mm 3.5 --resume \
  --json /tmp/extrinsic_marks_new.json \
  --out /tmp/extrinsic_new.txt \
  > /tmp/remark5.log 2>&1 < /dev/null &
sleep 7
echo "--- 启动状态 ---"
grep -E '续打|第 .*/5|板面' /tmp/remark5.log | tail -6
pgrep -f 'python3 -u remark_extrinsic' >/dev/null && echo "运行中" || echo "未运行"
