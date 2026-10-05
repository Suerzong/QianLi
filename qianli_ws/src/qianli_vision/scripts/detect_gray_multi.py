#!/usr/bin/env python3
"""第2步v6：多物块检测 + mask 叠加（找出小物块为什么漏检）

改进：
1. 显示 ROI 内【所有】灰色区域（前 5 个），带面积数字 —— 小物块不再被隐藏
2. mask 半透明叠加在原图上（红色半透明=灰色区域）—— 直观看到判定结果
3. MIN_AREA 调小范围（0~1000），方便看小物块

按键：q 退出 | s 保存 | r 重框选
"""

import cv2
import numpy as np

INIT = {'S_MAX': 127, 'V_MIN': 69, 'V_MAX': 167, 'MIN_AREA': 602}


def main():
    cap = cv2.VideoCapture(0)
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
    cv2.createTrackbar('MIN_AREA', 'Trackbars', INIT['MIN_AREA'], 1000,
                       lambda v: None)

    print('>>> 拖动滑块调参。q 退出，s 保存，r 重框选。')
    last_params = None

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        s_max = cv2.getTrackbarPos('S_MAX', 'Trackbars')
        v_min = cv2.getTrackbarPos('V_MIN', 'Trackbars')
        v_max = cv2.getTrackbarPos('V_MAX', 'Trackbars')
        min_area = cv2.getTrackbarPos('MIN_AREA', 'Trackbars')

        params = (s_max, v_min, v_max, min_area, x, y, w, h)
        if params != last_params:
            last_params = params
            try:
                with open('/tmp/current_params.txt', 'w') as f:
                    f.write(f'S_MAX={s_max}\nV_MIN={v_min}\nV_MAX={v_max}\n'
                            f'MIN_AREA={min_area}\nROI={x},{y},{w},{h}\n')
            except OSError:
                pass

        roi_img = frame[y:y + h, x:x + w]
        hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, v_min), (179, s_max, v_max))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        # 所有候选（≥MIN_AREA），按面积降序
        cands = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area >= min_area:
                bx, by, bw, bh = cv2.boundingRect(cnt)
                cands.append((area, bx, by, bw, bh))
        cands.sort(reverse=True)

        # 结果图
        display = frame.copy()
        cv2.rectangle(display, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.putText(display, 'ROI', (x + 5, y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        # 半透明叠加 mask（红）到全图对应区域
        overlay = np.zeros_like(display)
        overlay[y:y + h, x:x + w] = cv2.merge([mask, mask, mask])
        display = cv2.addWeighted(display, 0.7, overlay, 0.3, 0)
        # 叠加后重画 ROI 框（保持清晰）
        cv2.rectangle(display, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # 所有候选画绿框（最大=粗，其余=细），带面积
        for i, (area, bx, by, bw, bh) in enumerate(cands[:5]):
            gx, gy = x + bx + bw // 2, y + by + bh // 2
            thick = 3 if i == 0 else 1
            cv2.rectangle(display, (x + bx, y + by), (x + bx + bw, y + by + bh),
                          (0, 255, 0), thick)
            cv2.putText(display, f'#{i} a={int(area)} ({gx},{gy})',
                        (x + bx, y + by - 6), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (0, 255, 0), 1)

        cv2.putText(display, f'candidates: {len(cands)} | '
                    f'S={s_max} V={v_min}-{v_max} MINA={min_area}',
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 255), 2)

        cv2.imshow('Multi Detect', display)
        cv2.imshow('mask', mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite('/tmp/multi_result.png', display)
            print(f'已保存。参数: S_MAX={s_max} V_MIN={v_min} '
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
