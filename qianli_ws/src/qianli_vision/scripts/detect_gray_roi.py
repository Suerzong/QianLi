#!/usr/bin/env python3
"""第2步v4：ROI + 灰色识别（排除灰色桌面干扰）

为什么需要 ROI：
  桌面也是灰色 → 灰色 mask 会把整张桌面都抠出来。
  用 ROI 把检测限定在"方格纸区域"，桌面直接不参与计算。

交互流程：
  1. 启动后出现 "set ROI" 窗口 → 用鼠标在方格纸周围拉框 → 回车确认
     （也可以用下方 ROI_* 常量写死坐标，跳过鼠标框选）
  2. 之后每帧只在 ROI 内做灰色检测（绿框 = 物块）
  3. 按 r 重新框选 ROI，按 s 保存结果，按 q 退出

核心代码（每帧做的事）：
  roi = frame[y:y+h, x:x+w]        # 裁剪出感兴趣区域
  mask = 灰色判定(roi)              # 只在 ROI 里找灰色
  ... 找到物块后坐标加回 (x, y) 偏移 → 全图坐标
"""

from project_paths import default_camera

import cv2
import numpy as np

# ================= 手调配置区 =================
# 方式一：写死 ROI 坐标（设为 None 则启动时用鼠标框选）
# 坐标 = 像素位置，画面是 640x480。把相机固定后，填你框选到的值。
ROI_X, ROI_Y, ROI_W, ROI_H = None, None, None, None
# 例：ROI_X, ROI_Y, ROI_W, ROI_H = 200, 120, 300, 260

# 灰色定义（HSV）—— 手调这三个值：
S_MAX = 45      # 饱和度上限：调大→更多"淡色"算灰色；调小→更严格
V_MIN = 60      # 亮度下限：调大→排除暗部/阴影
V_MAX = 200     # 亮度上限：调小→排除亮白

MIN_AREA = 400  # ROI 内最小面积（像素）


def gray_mask_of(roi):
    """对 ROI 返回灰色二值图。"""
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 0, V_MIN), (179, S_MAX, V_MAX))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def detect_in_roi(roi):
    """ROI 内找灰色物块，返回 (cx, cy, w, h, contour)，坐标相对 ROI。"""
    mask = gray_mask_of(roi)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        if best is None or area > best[4]:
            best = (x + w // 2, y + h // 2, w, h, area)
    return best, mask


def main():
    cap = cv2.VideoCapture(default_camera())
    if not cap.isOpened():
        print('无法打开相机')
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # 第一帧用于框选 ROI（或直接用手调配置区写死的坐标）
    ok, frame = cap.read()
    if not ok:
        print('读帧失败')
        return

    if ROI_X is not None:
        x, y, w, h = ROI_X, ROI_Y, ROI_W, ROI_H
        print(f'>>> 使用手调 ROI: x={x} y={y} w={w} h={h}')
    else:
        print('>>> 请用鼠标在【方格纸】周围拉一个框，然后按回车确认 ROI')
        roi = cv2.selectROI('set ROI', frame, showCrosshair=True,
                            fromCenter=False)
        x, y, w, h = roi
        if w <= 0 or h <= 0:
            print('ROI 无效，使用全图')
            x, y, w, h = 0, 0, frame.shape[1], frame.shape[0]
        print(f'>>> ROI 已设定: x={x} y={y} w={w} h={h} (按 r 可重新框选)')
        cv2.destroyWindow('set ROI')

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        # 1) 裁剪 ROI
        roi_img = frame[y:y + h, x:x + w]
        # 2) 只在 ROI 里检测灰色
        best, mask = detect_in_roi(roi_img)

        # 3) 画图（坐标加回偏移量 → 全图坐标）
        display = frame.copy()
        cv2.rectangle(display, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.putText(display, 'ROI', (x + 5, y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        if best:
            cx, cy, bw, bh, area = best
            # ROI 内坐标 → 全图坐标
            gx, gy = x + cx, y + cy
            cv2.rectangle(display, (x + cx - bw // 2, y + cy - bh // 2),
                          (x + cx + bw // 2, y + cy + bh // 2),
                          (0, 255, 0), 3)
            cv2.circle(display, (gx, gy), 6, (0, 0, 255), -1)
            cv2.putText(display,
                        f'object center=({gx},{gy}) size={bw}x{bh}px',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0), 2)
        else:
            cv2.putText(display, 'no object in ROI',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 255), 2)

        cv2.putText(display, 'r=re-ROI s=save q=quit',
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (200, 200, 200), 1)

        cv2.imshow('ROI + Gray Detect', display)
        # 单独窗口看 ROI 内的 mask（学习用）
        cv2.imshow('ROI mask', mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite('/tmp/roi_result.png', display)
            print('已保存 /tmp/roi_result.png')
        elif key == ord('r'):
            ok2, frame2 = cap.read()
            if ok2:
                roi = cv2.selectROI('set ROI', frame2, showCrosshair=True,
                                    fromCenter=False)
                x, y, w, h = roi
                if w > 0 and h > 0:
                    print(f'>>> 新 ROI: x={x} y={y} w={w} h={h}')

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
