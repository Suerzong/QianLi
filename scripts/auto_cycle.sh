#!/bin/bash
# 全自动循环：定位 -> 多帧实测尺寸 -> 自动抓取 -> 中间位松手 -> 腕部摆正 -> 回折叠位
# 深度 -48mm（实测这个值夹持稳定；-38mm 只咬住方块顶部会滑）
# 偏置 = mesh 自动居中 + 额外 -5mm（用户要求：固定爪离面约 2.5mm，别太险）
cd /home/ros/QianLi/qianli_ws/src/qianli_vision/scripts || exit 1
echo "=== 1) 定位 ==="
python3 board_frame.py --locate --color yellow 2>&1 \
  | grep -vE 'Warning|warn|VIDIOC|obsensor' | tail -5
SEL=$(python3 pick_block.py --color yellow 2>/dev/null)
echo "选中: $SEL"
XY=$(echo "$SEL" | awk '{print $1, $2}')
set -- $XY
case "$1" in
  ''|NONE|*[!0-9.-]*) echo "❌ 没定位到方块"; exit 1 ;;
esac
echo
echo "=== 2) 多帧实测方块尺寸 ==="
MS=$(python3 measure_size_median.py 2>/dev/null)
echo "尺寸采样: $MS"
CUBE=$(echo "$MS" | awk '{print $1}')
case "$CUBE" in ''|*[!0-9.]*) CUBE=33 ;; esac
echo "采用边长 = ${CUBE}mm   目标 base = ($1, $2)"
echo
echo "=== 3) 自动抓取 ==="
timeout 280 python3 -u grasp_auto.py --x "$1" --y "$2" \
  --cube-mm "$CUBE" --gap-mm 0.5 --y-offset-mm -20 --grasp-depth-mm -48 \
  --target-load 20 --extra-squeeze-rad 0.05 --lift-mm 40 \
  > /tmp/auto9.log 2>&1
grep -vE 'UserWarning|warnings.warn' /tmp/auto9.log \
  | grep -E '几何|偏置|最终|抓取位|路径|落到底|拍照|合爪|继续压|抬升|保持|回中间|松手|腕部|归位|held|contact|total_offset|lift|load_after|滑脱|夹住|error' \
  | tail -22
