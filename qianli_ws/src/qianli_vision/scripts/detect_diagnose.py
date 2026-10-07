#!/usr/bin/env python3
"""第2步v7：诊断模式 —— 显示 mask 里所有轮廓（不过滤）+ 面积落盘

目标：回答"mask 里明明有物块，为什么检测不到？"
方案：
  1. 不过滤面积（只要 >30 像素都画出来，标注面积）
  2. 形态学核改 3x3（5x5 会把小物块腐蚀掉）
  3. 每帧把轮廓面积列表写入 /tmp/contours_info.txt，供外部诊断

按键：q 退出 | s 保存 | r 重框选
"""

from project_paths import open_video_capture

from project_paths import default_camera

import cv2
import numpy as np

INIT = {'S_MAX': 127, 'V_MIN': 69, 'V_MAX': 167}

# 最小面积：物块在 mask 里实测 ~391px（23x23），噪声碎片 ~38px
# 取中间值 150：保住物块、滤掉噪声
MIN_AREA = 150


def main():
    cap = open_video_capture(default_camera())
    if not cap.isOpened():
        print('无法打开相机')
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    ok, frame = cap.read()
    if not ok:
        print('读帧失败')
        return

    print('>>> 请用鼠标在【方格纸】周围拉框，回车确认')
    roi = cv2.selectROI('set ROI', frame, showCrosshair=True,
                        fromCenter=False)
    x, y, w, h = roi
    if w <= 0 or h <= 0:
        x, y, w, h = 0, 0, frame.shape[1], frame.shape[0]
    print(f'>>> ROI: x={x} y={y} w={w} h={h}')
    cv2.destroyWindow('set ROI')

    cv2.namedWindow('Trackbars')
    cv2.createTrackbar('S_MAX', 'Trackbars', INIT['S_MAX'], 179, lambda v: None)
    cv2.createTrackbar('V_MIN', 'Trackbars', INIT['V_MIN'], 255, lambda v: None)
    cv2.createTrackbar('V_MAX', 'Trackbars', INIT['V_MAX'], 255, lambda v: None)

    print('>>> 诊断模式：显示所有轮廓（不过滤）。q 退出，s 保存。')

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        s_max = cv2.getTrackbarPos('S_MAX', 'Trackbars')
        v_min = cv2.getTrackbarPos('V_MIN', 'Trackbars')
        v_max = cv2.getTrackbarPos('V_MAX', 'Trackbars')

        roi_img = frame[y:y + h, x:x + w]
        hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, v_min), (179, s_max, v_max))
        # 小核（3x3）：保护小物块不被腐蚀掉
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        # 所有轮廓（area > MIN_AREA），按面积降序
        cands = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > MIN_AREA:
                bx, by, bw, bh = cv2.boundingRect(cnt)
                cands.append((area, bx, by, bw, bh))
        cands.sort(reverse=True)

        # 面积列表落盘（每帧更新，供外部诊断）
        try:
            with open('/tmp/contours_info.txt', 'w') as f:
                f.write(f'S_MAX={s_max} V_MIN={v_min} V_MAX={v_max}\n')
                f.write(f'total_contours={len(contours)} '
                        f'kept={len(cands)}\n')
                for area, bx, by, bw, bh in cands[:10]:
                    f.write(f'area={int(area):6d} bbox=({bx},{by}) '
                            f'{bw}x{bh}\n')
        except OSError:
            pass

        display = frame.copy()
        cv2.rectangle(display, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # 所有轮廓都画（绿=面积>200，青=小轮廓）
        for area, bx, by, bw, bh in cands[:8]:
            gx, gy = x + bx + bw // 2, y + by + bh // 2
            color = (0, 255, 0) if area > 200 else (255, 255, 0)
            cv2.rectangle(display, (x + bx, y + by),
                          (x + bx + bw, y + by + bh), color, 2)
            cv2.putText(display, f'{int(area)}px',
                        (x + bx, y + by - 6), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, color, 1)

        cv2.putText(display, f'contours={len(cands)} | '
                    f'S={s_max} V={v_min}-{v_max}',
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 255), 2)
        cv2.putText(display, 'cyan=small(<200px) green=big',
                    (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (200, 200, 200), 1)

        cv2.imshow('Diagnose', display)
        cv2.imshow('mask', mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite('/tmp/diag_result.png', display)
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
