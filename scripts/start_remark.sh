#!/bin/bash
# 启动打点脚本（读回滚后的零点）
cd /home/ros/QianLi/qianli_ws/src/qianli_vision/scripts || exit 1
rm -f /tmp/grid_mark
setsid python3 -u remark_extrinsic.py \
  --json /tmp/extrinsic_marks_new.json \
  --out /tmp/extrinsic_new.txt \
  > /tmp/remark3.log 2>&1 < /dev/null &
sleep 7
echo "--- 启动状态 ---"
grep -E '零点|限位|第 1/5|板面' /tmp/remark3.log | tail -6
echo "--- 进程 ---"
pgrep -f 'python3 -u remark_extrinsic' >/dev/null && echo "运行中" || echo "未运行"
