#!/usr/bin/env python3
"""终极调参版：ROI + HSV 滑块 + 尺寸过滤（全部可调）

流程：
  1. 启动时框选 ROI（排除灰色桌面）
  2. 滑块调 HSV：让掩码只罩住物块
  3. 程序尺寸过滤：只保留物块大小的区域
  4. 按 r 可重新框选 ROI

滑块面板（'Trackbars' 窗口）：
  S_MAX   0~179   饱和度上限（灰色判定：S 低 = 颜色淡）
  V_MIN   0~255   亮度下限（调大→排除暗部/阴影）
  V_MAX   0~255   亮度上限（调小→排除亮白）
  MIN_SIZE 0~100  物块最小边长（按最长边）
  MAX_SIZE 0~300  物块最大边长
  MIN_AREA 0~2000 最小面积（滤噪点）

窗口：
  'Final'  原图 + ROI + 物块绿框 + 掩码红色半透明叠加
  'mask'   灰色二值图（白=判为灰色）

按键：q 退出 | s 保存 /tmp/final_tune.png | r 重框 ROI
"""

from project_paths import default_camera

import cv2
import numpy as np

# 初始值（当前画面物块 24x25px，下限放 20）
INIT = {'S_MAX': 127, 'V_MIN': 143, 'V_MAX': 167,
        'MIN_SIZE': 20, 'MAX_SIZE': 50, 'MIN_AREA': 330, 'MAX_AREA': 350,
        'MAX_DIFF': 5}

cap = cv2.VideoCapture(default_camera())
if not cap.isOpened():
    print('无法打开相机')
    exit(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 第一步：框选 ROI
ok, frame0 = cap.read()
if not ok:
    print('读帧失败')
    exit(1)
print('>>> 请框选【方格纸】区域（排除灰色桌面），回车确认')
r = cv2.selectROI('set ROI', frame0, showCrosshair=True, fromCenter=False)
x, y, w, h = r
if w <= 0 or h <= 0:
    x, y, w, h = 0, 0, frame0.shape[1], frame0.shape[0]
cv2.destroyWindow('set ROI')
print(f'>>> ROI: x={x} y={y} w={w} h={h}（r 键可重框）')

cv2.namedWindow('Trackbars')
cv2.createTrackbar('S_MAX', 'Trackbars', INIT['S_MAX'], 179, lambda v: None)
cv2.createTrackbar('V_MIN', 'Trackbars', INIT['V_MIN'], 255, lambda v: None)
cv2.createTrackbar('V_MAX', 'Trackbars', INIT['V_MAX'], 255, lambda v: None)
cv2.createTrackbar('MIN_SIZE', 'Trackbars', INIT['MIN_SIZE'], 100,
                   lambda v: None)
cv2.createTrackbar('MAX_SIZE', 'Trackbars', INIT['MAX_SIZE'], 300,
                   lambda v: None)
cv2.createTrackbar('MIN_AREA', 'Trackbars', INIT['MIN_AREA'], 2000,
                   lambda v: None)
# 面积上限：物块轮廓面积区间（滤掉过大/过小的区域）
cv2.createTrackbar('MAX_AREA', 'Trackbars', INIT['MAX_AREA'], 2000,
                   lambda v: None)
# 长宽差上限：物块是立方体，投影应接近正方形 → 滤掉长条杂质
cv2.createTrackbar('MAX_DIFF', 'Trackbars', INIT['MAX_DIFF'], 50,
                   lambda v: None)

print('终极调参版启动。拖滑块调 HSV 让掩码干净，再看物块框。q 退出。')

while True:
    ok, frame = cap.read()
    if not ok:
        continue

    s_max = cv2.getTrackbarPos('S_MAX', 'Trackbars')
    v_min = cv2.getTrackbarPos('V_MIN', 'Trackbars')
    v_max = cv2.getTrackbarPos('V_MAX', 'Trackbars')
    min_size = cv2.getTrackbarPos('MIN_SIZE', 'Trackbars')
    max_size = cv2.getTrackbarPos('MAX_SIZE', 'Trackbars')
    min_area = cv2.getTrackbarPos('MIN_AREA', 'Trackbars')
    max_area = cv2.getTrackbarPos('MAX_AREA', 'Trackbars')
    max_diff = cv2.getTrackbarPos('MAX_DIFF', 'Trackbars')

    # 只处理 ROI 内的图像（排除灰色桌面）
    roi_img = frame[y:y + h, x:x + w]
    hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 0, v_min), (179, s_max, v_max))
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    cands = []
    for c in contours:
        area = cv2.contourArea(c)
        # 面积区间过滤：只保留 [min_area, max_area] 内的区域
        if not (min_area <= area <= max_area):
            continue
        bx, by, bw, bh = cv2.boundingRect(c)
        side = max(bw, bh)
        # 正方形过滤：长宽差超过 max_diff 的（长条杂质）直接丢弃
        if max_diff > 0 and abs(bw - bh) > max_diff:
            continue
        if min_size <= side <= max_size:
            cands.append((area, bx, by, bw, bh))
    cands.sort(reverse=True)

    # 显示：ROI 框 + 掩码半透明叠加（叠加到 ROI 区域）+ 物块框
    display = frame.copy()
    overlay = np.zeros_like(display)
    overlay[y:y + h, x:x + w] = cv2.merge([mask, np.zeros_like(mask),
                                           np.zeros_like(mask)])
    display = cv2.addWeighted(display, 0.7, overlay, 0.3, 0)
    cv2.rectangle(display, (x, y), (x + w, y + h), (255, 0, 0), 2)
    cv2.putText(display, 'ROI', (x + 5, y - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    for i, (area, bx, by, bw, bh) in enumerate(cands[:3]):
        color = (0, 255, 0) if i == 0 else (0, 200, 255)
        # ROI 内坐标 → 全图坐标
        gx, gy = x + bx + bw // 2, y + by + bh // 2
        cv2.rectangle(display, (x + bx, y + by), (x + bx + bw, y + by + bh),
                      color, 2)
        cv2.circle(display, (gx, gy), 4, (0, 0, 255), -1)
        cv2.putText(display, f'#{i} {bw}x{bh} a={int(area)}',
                    (x + bx, y + by - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    color, 1)

    cv2.putText(display,
                f'kept={len(cands)} S={s_max} V={v_min}-{v_max} '
                f'size={min_size}-{max_size} dif<={max_diff} '
                f'area={min_area}-{max_area}',
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    cv2.putText(display, 'r=ROI q=quit s=save',
                (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (200, 200, 200), 1)

    cv2.imshow('Final', display)
    cv2.imshow('mask', mask)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('s'):
        cv2.imwrite('/tmp/final_tune.png', display)
        print(f'已保存。参数: S_MAX={s_max} V_MIN={v_min} V_MAX={v_max} '
              f'MIN_SIZE={min_size} MAX_SIZE={max_size} '
              f'MIN_AREA={min_area} MAX_AREA={max_area} MAX_DIFF={max_diff}')
    elif key == ord('r'):
        ok2, frame2 = cap.read()
        if ok2:
            r = cv2.selectROI('set ROI', frame2, showCrosshair=True,
                              fromCenter=False)
            x, y, w, h = r
            if w > 0 and h > 0:
                print(f'>>> 新 ROI: x={x} y={y} w={w} h={h}')

cap.release()
cv2.destroyAllWindows()
