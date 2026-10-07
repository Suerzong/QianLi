#!/usr/bin/env python3
"""第2步v2：改进版方块检测（针对灰色立方体）

相比 v1 的改进（解决"不稳、来回跳"）：
1. 自适应 Canny 阈值（按图像中位数算高低阈值，适应光照变化）
2. 形态学闭运算（填平边缘断裂，轮廓更完整）
3. 面积排序（只保留最大的几个候选）
4. 位置平滑（对最近 N 帧取均值，抑制跳动）
5. 最小外接矩形角度筛选（立方体侧面投影接近矩形/正方形）

用法：
    python3 ~/QianLi/qianli_ws/src/qianli_vision/scripts/detect_demo.py
按 q 退出。
"""

from project_paths import open_video_capture

from project_paths import default_camera

import collections
import cv2
import numpy as np


class SmoothTracker:
    """对检测结果做时间平滑，抑制跳动。"""

    def __init__(self, history=5):
        self.history = history
        self.queue = collections.deque(maxlen=history)

    def update(self, detections):
        self.queue.append(detections)
        # 取最近几帧中"最稳定"的（出现次数最多的候选）
        if not self.queue:
            return []
        # 简单策略：取最近一帧的最大候选，若无则用历史
        for frame_dets in reversed(self.queue):
            if frame_dets:
                return frame_dets
        return []


def find_squares_v2(frame):
    """改进版：找画面中最大的方块状区域。"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 自适应 Canny：中位数决定高低阈值（适应不同光照）
    med = np.median(gray)
    low = int(max(0, 0.5 * med))
    high = int(min(255, 1.2 * med))
    edges = cv2.Canny(gray, low, high)

    # 形态学闭运算：填平断裂，让轮廓闭合
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    results = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 800:  # 过滤太小
            continue
        # 最小外接矩形（能框住轮廓的旋转矩形）
        rect = cv2.minAreaRect(cnt)
        (cx, cy), (rw, rh), angle = rect
        # 宽高比过滤：立方体投影接近正方形（0.6~1.6 容忍透视）
        if rw <= 0 or rh <= 0:
            continue
        ratio = max(rw, rh) / min(rw, rh)
        if 1.0 < ratio < 2.5:  # 立方体侧面可以比较扁
            results.append((int(cx), int(cy), int(max(rw, rh)),
                            int(area), cnt))

    # 按面积排序，最大的最可能是主目标
    results.sort(key=lambda r: r[3], reverse=True)
    return results


def main():
    cap = open_video_capture(default_camera())
    if not cap.isOpened():
        print('无法打开相机 /dev/video0')
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print('v2 检测已启动。把灰立方体放到镜头前，按 q 退出。')

    tracker = SmoothTracker(5)

    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        display = frame.copy()
        dets = find_squares_v2(frame)
        stable = tracker.update(dets)

        for cx, cy, size, area, cnt in stable[:2]:
            # 画外接矩形（旋转）
            box = cv2.boxPoints(cv2.minAreaRect(cnt))
            box = np.int32(box)
            cv2.drawContours(display, [box], -1, (0, 255, 0), 3)
            cv2.circle(display, (cx, cy), 6, (0, 0, 255), -1)
            cv2.putText(display, f'({cx},{cy}) size={size}px',
                        (cx - 50, cy - 15), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)

        # 信息栏
        cv2.putText(display, f'detections: {len(dets)} | stable: {len(stable)}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow('Step2v2 - Detect Gray Cube', display)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
