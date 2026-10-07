#!/bin/bash
# 停掉值守与正在跑的抓取流程（释放相机/串口）
for pat in 'bash watch.sh' 'auto_cycle.sh' 'grasp_auto.py' 'move_retry.py'; do
  for p in $(pgrep -f "$pat" 2>/dev/null); do
    [ "$p" = "$$" ] && continue
    kill "$p" 2>/dev/null && echo "killed $pat pid=$p"
  done
done
sleep 2
echo "--- 剩余相关进程 ---"
pgrep -af 'watch.sh|auto_cycle|grasp_auto' 2>/dev/null | head -5
echo "--- 清理完成 ---"
exit 0
