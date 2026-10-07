#!/usr/bin/env python3
"""第2步v5：滑块实时调参版（ROI + 灰色识别）

窗口：
  [1] ROI+Detect   检测结果（蓝框=ROI，绿框=物块）
  [2] ROI mask     灰色二值图（白=被判为灰色）—— 调参时看它
  [3] Trackbars    滑块面板

滑块（实时生效）：
  S_MAX  0-179   饱和度上限（灰色判定关键）
  V_MIN  0-255   亮度下限（调大→排除暗部/阴影）
  V_MAX  0-255   亮度上限（调小→排除亮白）
  MIN_AREA 0-5000 最小面积（调大→忽略小杂点）

按键：q 退出 | s 保存结果 | r 重新框选 ROI
"""

from project_paths import default_camera

import cv2
import numpy as np

# 初始值（与手调版一致）
INIT = {'S_MAX': 45, 'V_MIN': 60, 'V_MAX': 200, 'MIN_AREA': 400}


def gray_mask_of(roi, s_max, v_min, v_max):
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 0, v_min), (179, s_max, v_max))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def main():
    cap = cv2.VideoCapture(default_camera())
    if not cap.isOpened():
        print('无法打开相机')
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    ok, frame = cap.read()
    if not ok:
        print('读帧失败')
        return

    # 第一步：框选 ROI
    print('>>> 请用鼠标在【方格纸】周围拉框，回车确认')
    roi = cv2.selectROI('set ROI', frame, showCrosshair=True,
                        fromCenter=False)
    x, y, w, h = roi
    if w <= 0 or h <= 0:
        x, y, w, h = 0, 0, frame.shape[1], frame.shape[0]
    print(f'>>> ROI: x={x} y={y} w={w} h={h}')
    cv2.destroyWindow('set ROI')

    # 滑块面板
    cv2.namedWindow('Trackbars')
    cv2.createTrackbar('S_MAX', 'Trackbars', INIT['S_MAX'], 179, lambda v: None)
    cv2.createTrackbar('V_MIN', 'Trackbars', INIT['V_MIN'], 255, lambda v: None)
    cv2.createTrackbar('V_MAX', 'Trackbars', INIT['V_MAX'], 255, lambda v: None)
    cv2.createTrackbar('MIN_AREA', 'Trackbars', INIT['MIN_AREA'], 5000,
                       lambda v: None)

    print('>>> 拖动滑块实时调参。q 退出，s 保存，r 重框选。')
    last_params = None  # 上次写入文件的参数（避免每帧重复写盘）

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        # 读滑块当前值（每帧实时生效）
        s_max = cv2.getTrackbarPos('S_MAX', 'Trackbars')
        v_min = cv2.getTrackbarPos('V_MIN', 'Trackbars')
        v_max = cv2.getTrackbarPos('V_MAX', 'Trackbars')
        min_area = cv2.getTrackbarPos('MIN_AREA', 'Trackbars')

        # 参数变化时自动落盘（方便外部读取 / 固化）
        params = (s_max, v_min, v_max, min_area, x, y, w, h)
        if params != last_params:
            last_params = params
            try:
                with open('/tmp/current_params.txt', 'w') as f:
                    f.write(f'S_MAX={s_max}\nV_MIN={v_min}\nV_MAX={v_max}\n'
                            f'MIN_AREA={min_area}\n'
                            f'ROI={x},{y},{w},{h}\n')
            except OSError:
                pass

        roi_img = frame[y:y + h, x:x + w]
        mask = gray_mask_of(roi_img, s_max, v_min, v_max)

        # ROI 内找灰色物块
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        best = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue
            bx, by, bw, bh = cv2.boundingRect(cnt)
            if best is None or area > best[4]:
                best = (bx + bw // 2, by + bh // 2, bw, bh, area)

        # 画结果
        display = frame.copy()
        cv2.rectangle(display, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.putText(display, 'ROI', (x + 5, y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        if best:
            cx, cy, bw, bh, area = best
            gx, gy = x + cx, y + cy
            cv2.rectangle(display, (x + cx - bw // 2, y + cy - bh // 2),
                          (x + cx + bw // 2, y + cy + bh // 2),
                          (0, 255, 0), 3)
            cv2.circle(display, (gx, gy), 6, (0, 0, 255), -1)
            cv2.putText(display,
                        f'center=({gx},{gy}) size={bw}x{bh}px area={area}',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0), 2)
        else:
            cv2.putText(display, 'no object',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 255), 2)
        cv2.putText(display,
                    f'S_MAX={s_max} V_MIN={v_min} V_MAX={v_max} '
                    f'MIN_AREA={min_area}',
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (200, 200, 200), 1)

        cv2.imshow('ROI+Detect', display)
        cv2.imshow('ROI mask', mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite('/tmp/roi_trackbar_result.png', display)
            print(f'已保存。当前参数: S_MAX={s_max} V_MIN={v_min} '
                  f'V_MAX={v_max} MIN_AREA={min_area}')
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
