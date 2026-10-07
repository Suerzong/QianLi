#!/bin/bash
# 值守模式：用**掩码重合**判断 (0,0) 上的方块有没有被拿走。
#   与参考掩码有重合       -> 待命
#   连续2次重合不足        -> 方块被拿走 -> 找棋盘上的方块 -> 抓取放回 (0,0)
#   每轮结束后等臂归位，再重建参考掩码（面积校验通过才录）
#
# 用法: watch.sh [轮询秒] [最大轮次(0=无限)] [最小重合比例]
cd /home/ros/QianLi/qianli_ws/src/qianli_vision/scripts || exit 1
POLL=${1:-4}
MAX=${2:-0}
MINOV=${3:-0.10}
LOG=/tmp/watch.log
: > $LOG
echo "[watch] 值守开始：掩码重合判定 (0,0)，轮询 ${POLL}s，最小重合 ${MINOV}" | tee -a $LOG
round=0
miss=0
while true; do
  round=$((round+1))
  AT=$(timeout 8 python3 check_at_00.py "$MINOV" 2>/dev/null)
  if [ "$AT" = "YES" ]; then
    miss=0
    echo "[$(date +%H:%M:%S)] (0,0) 掩码有重合 -> 没动，待命" >> $LOG
    sleep "$POLL"
    continue
  fi
  miss=$((miss+1))
  if [ "$miss" -lt 2 ]; then
    echo "[$(date +%H:%M:%S)] (0,0) 掩码不重合(第${miss}次)，再确认一次…" >> $LOG
    sleep "$POLL"
    continue
  fi
  echo "[$(date +%H:%M:%S)] 连续2次不重合 -> 方块被拿走，找棋盘上的方块…" >> $LOG
  SEL=$(python3 pick_block.py --color yellow 2>/dev/null)
  X=$(echo "$SEL" | awk '{print $1}')
  case "$X" in
    ''|NONE|*[!0-9.-]*) echo "[$(date +%H:%M:%S)] 棋盘上没找到方块，继续等" >> $LOG ;;
    *)
      echo "[$(date +%H:%M:%S)] 发现方块: $SEL" >> $LOG
      bash /tmp/auto_cycle.sh >> $LOG 2>&1
      # 等臂完全归位后重建参考掩码（避免把夹爪黄件录进参考）
      for k in 1 2 3 4 5 6; do
        sleep 4
        RS=$(timeout 8 python3 check_at_00.py --reset 2>/dev/null)
        case "$RS" in
          RESET_OK*) echo "[$(date +%H:%M:%S)] 参考掩码已重建 ($RS)" >> $LOG
                     miss=0
                     break ;;
          *)         echo "[$(date +%H:%M:%S)] 参考重建暂缓 ($RS)，稍后再试" >> $LOG ;;
        esac
      done
      ;;
  esac
  if [ "$MAX" -gt 0 ] && [ "$round" -ge "$MAX" ]; then
    echo "[watch] 达到最大轮次 $MAX，退出" | tee -a $LOG
    break
  fi
  sleep "$POLL"
done
