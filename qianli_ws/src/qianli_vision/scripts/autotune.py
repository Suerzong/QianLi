#!/usr/bin/env python3
"""自动调参：遍历 HSV 阈值，找出能稳定锁定物块的参数组合

目标物块特征（用户给定）：
  轮廓面积 330~350 px
  长宽差 <= 5（立方体投影接近正方形）
  最长边 20~50 px

输出：
  /tmp/autotune_raw.png    原始帧（带 ROI）
  /tmp/autotune_best.png   最佳参数下的标注图
  /tmp/autotune_mask.png   最佳参数下的掩码
  控制台：各候选参数组合的命中统计
"""

from project_paths import default_camera

import itertools
import sys

import cv2
import numpy as np

# ROI（默认用之前框选的位置；None = 全画面）
ROI = (205, 143, 281, 216)

AREA_MIN, AREA_MAX = 330, 350
DIFF_MAX = 5
SIZE_MIN, SIZE_MAX = 20, 50

cap = cv2.VideoCapture(default_camera())
if not cap.isOpened():
    print('无法打开相机')
    sys.exit(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 抓 7 帧取中值，抑制噪声
frames = []
for _ in range(7):
    ok, f = cap.read()
    if ok:
        frames.append(f)
cap.release()
if not frames:
    print('读帧失败')
    sys.exit(1)
frame = np.median(np.array(frames), axis=0).astype(np.uint8)
print(f'抓帧 {len(frames)} 张，取中值。画面 {frame.shape[1]}x{frame.shape[0]}')

x, y, w, h = ROI if ROI else (0, 0, frame.shape[1], frame.shape[0])
roi = frame[y:y + h, x:x + w].copy()
hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

vis = frame.copy()
cv2.rectangle(vis, (x, y), (x + w, y + h), (255, 0, 0), 2)
cv2.imwrite('/tmp/autotune_raw.png', vis)

k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))


def candidates(s_max, v_min, v_max, area_lo, area_hi):
    """返回满足面积+正方形+尺寸过滤的候选 (area, bx, by, bw, bh)。"""
    mask = cv2.inRange(hsv_roi, (0, 0, v_min), (179, s_max, v_max))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                               cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in cnts:
        a = cv2.contourArea(c)
        if not (area_lo <= a <= area_hi):
            continue
        bx, by, bw, bh = cv2.boundingRect(c)
        if DIFF_MAX > 0 and abs(bw - bh) > DIFF_MAX:
            continue
        if not (SIZE_MIN <= max(bw, bh) <= SIZE_MAX):
            continue
        out.append((a, bx, by, bw, bh))
    return out, mask


print('\n=== 阶段1：宽松面积(200~700)看有哪些区域 ===')
s_max_list = [100, 115, 127, 140, 155]
v_min_list = [120, 130, 140, 143, 150, 160]
v_max_list = [167, 175, 185, 200, 215]
best = None
for s_max, v_min, v_max in itertools.product(s_max_list, v_min_list,
                                             v_max_list):
    if v_min >= v_max:
        continue
    cands, _ = candidates(s_max, v_min, v_max, 200, 700)
    if len(cands) == 1:
        a, bx, by, bw, bh = cands[0]
        # 优先：面积接近 330~350，且正方形
        score = abs(a - 340) + abs(bw - bh) * 5
        print(f'  S_MAX={s_max} V={v_min}-{v_max}: 1个候选 '
              f'area={int(a)} {bw}x{bh} score={score:.0f}')
        if best is None or score < best[0]:
            best = (score, s_max, v_min, v_max, cands[0])

print('\n=== 阶段2：严格面积 330~350 验证 ===')
if best:
    score, s_max, v_min, v_max, _ = best
    cands, mask = candidates(s_max, v_min, v_max, AREA_MIN, AREA_MAX)
    print(f'最佳候选参数 S_MAX={s_max} V_MIN={v_min} V_MAX={v_max}')
    print(f'严格过滤后候选数: {len(cands)}')
    for a, bx, by, bw, bh in cands:
        print(f'  area={int(a)} bbox=({bx},{by}) {bw}x{bh} '
              f'长宽差={abs(bw - bh)}')

    # 保存标注图
    display = frame.copy()
    cv2.rectangle(display, (x, y), (x + w, y + h), (255, 0, 0), 2)
    for i, (a, bx, by, bw, bh) in enumerate(cands):
        color = (0, 255, 0) if i == 0 else (0, 200, 255)
        cv2.rectangle(display, (x + bx, y + by),
                      (x + bx + bw, y + by + bh), color, 2)
        cv2.putText(display, f'a={int(a)} {bw}x{bh}',
                    (x + bx, y + by - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    color, 1)
    cv2.putText(display, f'S={s_max} V={v_min}-{v_max} n={len(cands)}',
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.imwrite('/tmp/autotune_best.png', display)
    cv2.imwrite('/tmp/autotune_mask.png', mask)

    # 打印该参数下 ROI 内所有轮廓（不过滤），帮助判断
    mask2 = cv2.inRange(hsv_roi, (0, 0, v_min), (179, s_max, v_max))
    mask2 = cv2.morphologyEx(mask2, cv2.MORPH_OPEN, k)
    mask2 = cv2.morphologyEx(mask2, cv2.MORPH_CLOSE, k)
    cnts, _ = cv2.findContours(mask2, cv2.RETR_EXTERNAL,
                               cv2.CHAIN_APPROX_SIMPLE)
    allc = sorted(((cv2.contourArea(c),) + cv2.boundingRect(c)
                   for c in cnts), reverse=True)
    print(f'\n该参数下 ROI 内全部轮廓（前8，未过滤）:')
    for a, bx, by, bw, bh in allc[:8]:
        print(f'  area={int(a)} bbox=({bx},{by}) {bw}x{bh} 差={abs(bw-bh)}')
else:
    print('未找到恰好 1 个候选的参数组合')
print('\n已保存 /tmp/autotune_raw.png /tmp/autotune_best.png '
      '/tmp/autotune_mask.png')
