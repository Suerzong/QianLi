#!/usr/bin/env python3
"""第2步v3：用 HSV 饱和度+亮度识别灰色立方体（教学可视化版）

灰色 = 低饱和度(S) + 适中亮度(V)，在 HSV 里定义灰色，而不是 RGB。

窗口布局：
  [0] 原图    [1] S通道(灰度显示)   [2] V通道
  [3] 灰色mask(白=灰色)            [4] 检测结果(绿框)

按键：
  q 退出
  s 保存当前帧到 /tmp/gray_det_result.png
"""

from project_paths import open_video_capture

from project_paths import default_camera

import cv2
import numpy as np

# 灰色定义（HSV 空间）
S_MAX = 45          # 饱和度上限：S 低于此值 = "颜色很淡"
V_MIN = 60          # 亮度下限：低于此 = 太暗（黑/阴影）
V_MAX = 200         # 亮度上限：高于此 = 太亮（白）

MIN_AREA = 800      # 最小面积（像素）


def detect_gray(frame):
    """返回 (mask, contours_sorted) 灰色区域二值图 + 轮廓（按面积降序）"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    # 灰色 mask = (S 低) 且 (V 适中)
    mask = cv2.inRange(hsv, (0, 0, V_MIN), (179, S_MAX, V_MAX))

    # 形态学清理：开运算去噪点 + 闭运算填洞
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) > MIN_AREA]
    contours.sort(key=cv2.contourArea, reverse=True)
    return mask, contours, h, s, v


def main():
    cap = open_video_capture(default_camera())
    if not cap.isOpened():
        print('无法打开相机')
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print('灰色识别已启动（S<45, V:60-200）。放灰立方体到镜头前，按 q 退出。')

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        mask, contours, h, s, v = detect_gray(frame)

        # 可视化：S 和 V 通道转成灰度图给人看
        s_disp = cv2.normalize(s, None, 0, 255, cv2.NORM_MINMAX)
        v_disp = cv2.normalize(v, None, 0, 255, cv2.NORM_MINMAX)

        # 原图画框
        result = frame.copy()
        for i, cnt in enumerate(contours[:3]):
            x, y, w, hh = cv2.boundingRect(cnt)
            color = (0, 255, 0) if i == 0 else (0, 200, 255)
            cv2.rectangle(result, (x, y), (x + w, y + hh), color, 3)
            cv2.putText(result, f'#{i} ({x},{y}) {w}x{hh}',
                        (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, color, 2)

        # 信息
        info = f'gray regions: {len(contours)}'
        if contours:
            x, y, w, hh = cv2.boundingRect(contours[0])
            info += f' | biggest center=({x+w//2},{y+hh//2}) size={w}x{hh}'
        cv2.putText(result, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 255), 2)
        cv2.putText(result, 'S<45 V:60-200 | q quit, s save',
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # 四窗口显示
        cv2.imshow('0-original', frame)
        cv2.imshow('1-S-channel(bright=colorful)', s_disp)
        cv2.imshow('2-V-channel(bright=lit)', v_disp)
        cv2.imshow('3-gray-mask(white=gray)', mask)
        cv2.imshow('4-result', result)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite('/tmp/gray_det_result.png', result)
            print('已保存 /tmp/gray_det_result.png')

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
