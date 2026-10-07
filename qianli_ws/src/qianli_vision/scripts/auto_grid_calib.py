#!/usr/bin/env python3
"""第3步v2：全自动网格标定（无需点击/输入）

流程：
  1. 框选 ROI（方格纸区域）
  2. 程序自动在 ROI 内检测网格线（Hough 直线检测）
  3. 自动建立 像素↔物理坐标 映射（格子 = 3.3cm），求 Homography
  4. 实时检测物块 → 显示物理坐标 (Xcm, Ycm)

按键：q 退出 | s 保存 | r 重框 ROI | a 重新标定
"""

from project_paths import calibration_path, default_camera

import cv2
import numpy as np

CELL_CM = 3.3       # 网格边长（厘米）
MIN_AREA = 150      # 物块最小面积


def detect_grid_lines(roi_img):
    """检测 ROI 内网格线，返回 (行线y列表, 列线x列表)。"""
    gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=80,
                            minLineLength=30, maxLineGap=10)

    rows_y, cols_x = [], []   # 水平线 y、垂直线 x
    if lines is not None:
        lines = np.asarray(lines)
        # 兼容形状：(N,1,4) 旧版 / (N,4) 新版
        if lines.ndim == 3:
            lines = lines.reshape(-1, 4)
        for x1, y1, x2, y2 in lines:
            dx, dy = x2 - x1, y2 - y1
            length = np.hypot(dx, dy)
            if length < 30:
                continue
            angle = np.degrees(np.arctan2(abs(dy), abs(dx)))
            if angle < 15:           # 接近水平 → 行线
                rows_y.append((y1 + y2) / 2)
            elif angle > 75:         # 接近垂直 → 列线
                cols_x.append((x1 + x2) / 2)

    return rows_y, cols_x


def cluster_lines(values, tol=5):
    """把相近的线坐标聚成一条（同一格线的多个线段合并）。"""
    if not values:
        return []
    values = sorted(values)
    clusters = [[values[0]]]
    for v in values[1:]:
        if v - clusters[-1][-1] <= tol:
            clusters[-1].append(v)
        else:
            clusters.append([v])
    return [np.mean(c) for c in clusters]


def build_mapping(rows_y, cols_x):
    """由行线/列线建立 像素↔物理 映射点对。

    物理约定：第 0 条行线 = y=0，第 i 条 = y=i*CELL_CM；
             第 0 条列线 = x=0，第 j 条 = x=j*CELL_CM。
    """
    src, dst = [], []
    for i, y in enumerate(rows_y):
        for j, x in enumerate(cols_x):
            src.append((x, y))
            dst.append((j * CELL_CM, i * CELL_CM))
    return np.array(src, np.float32), np.array(dst, np.float32)


def compute_homography(roi_img, show_callback=None):
    """自动标定：返回 (H, n_lines, info)。"""
    rows_y, cols_x = detect_grid_lines(roi_img)
    rows_y = cluster_lines(rows_y)
    cols_x = cluster_lines(cols_x)
    if len(rows_y) < 2 or len(cols_x) < 2:
        return None, (len(rows_y), len(cols_x))
    src, dst = build_mapping(rows_y, cols_x)
    H, _ = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
    if H is None:
        return None, (len(rows_y), len(cols_x))
    return H, (len(rows_y), len(cols_x))


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

    print('>>> 框选【方格纸】区域，回车确认')
    r = cv2.selectROI('set ROI', frame, showCrosshair=True, fromCenter=False)
    rx, ry, rw, rh = r
    if rw <= 0 or rh <= 0:
        rx, ry, rw, rh = 0, 0, frame.shape[1], frame.shape[0]
    cv2.destroyWindow('set ROI')

    print('>>> 自动检测网格线并标定...')
    roi_img = frame[ry:ry + rh, rx:rx + rw]
    H, info = compute_homography(roi_img)
    if H is None:
        print(f'!!! 网格线检测失败（行{info[0]}条 列{info[1]}条），'
              '按 r 重框 ROI 或调整光照')
    else:
        print(f'>>> 标定成功！行线 {info[0]} 条、列线 {info[1]} 条')
        print('    物块将实时显示物理坐标 (cm)')

    print('>>> a=重新标定  r=重框ROI  s=保存  q=退出')

    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        display = frame.copy()
        cv2.rectangle(display, (rx, ry), (rx + rw, ry + rh), (255, 0, 0), 2)

        # 物块检测 + 物理坐标
        obj_info = None
        if H is not None:
            roi_img = frame[ry:ry + rh, rx:rx + rw]
            hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, (0, 0, 60), (179, 127, 167))
            k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            best = None
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < MIN_AREA:
                    continue
                bx, by, bw, bh = cv2.boundingRect(cnt)
                if best is None or area > best[4]:
                    best = (bx + bw // 2, by + bh // 2, bw, bh, area)
            if best:
                cx, cy, bw, bh, area = best
                gx, gy = rx + cx, ry + cy
                p = np.array([[[gx, gy]]], dtype=np.float64)
                phy = cv2.perspectiveTransform(p, H)
                Xcm, Ycm = phy[0][0]
                obj_info = (gx, gy, Xcm, Ycm, bw, bh)
                cv2.rectangle(display, (gx - bw // 2, gy - bh // 2),
                              (gx + bw // 2, gy + bh // 2), (0, 255, 0), 3)
                cv2.putText(display, f'({Xcm:.1f}, {Ycm:.1f}) cm',
                            (gx + 10, gy - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 0), 2)
                try:
                    with open('/tmp/object_pose.txt', 'w') as f:
                        f.write(f'X_cm={Xcm:.2f}\nY_cm={Ycm:.2f}\n'
                                f'center_px=({gx},{gy})\n'
                                f'size_px={bw}x{bh}\n')
                except OSError:
                    pass

        # 状态栏
        if H is not None:
            status = (f'CALIBRATED | obj: '
                      f'({obj_info[2]:.1f}, {obj_info[3]:.1f}) cm'
                      if obj_info else 'CALIBRATED | obj not found')
        else:
            status = 'calibration failed - press r to re-select ROI'
        cv2.putText(display, status, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(display, 'a=recalib r=ROI s=save q=quit',
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (200, 200, 200), 1)

        cv2.imshow('Auto Grid Calib', display)
        if H is not None:
            cv2.imshow('mask', mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite('/tmp/autocalib_result.png', display)
            if H is not None:
                np.save(calibration_path('homography.npy'), H)
                print('已保存 H 与截图')
        elif key == ord('a'):
            ok2, frame2 = cap.read()
            if ok2:
                roi_img = frame2[ry:ry + rh, rx:rx + rw]
                H, info = compute_homography(roi_img)
                print(f'>>> 重新标定：' +
                      (f'成功（行{info[0]}列{info[1]}）' if H is not None
                       else f'失败（行{info[0]}列{info[1]}）'))
        elif key == ord('r'):
            ok2, frame2 = cap.read()
            if ok2:
                r = cv2.selectROI('set ROI', frame2, showCrosshair=True,
                                  fromCenter=False)
                rx, ry, rw, rh = r
                if rw > 0 and rh > 0:
                    roi_img = frame2[ry:ry + rh, rx:rx + rw]
                    H, info = compute_homography(roi_img)
                    print('>>> 新 ROI + 标定：' +
                          (f'成功（行{info[0]}列{info[1]}）'
                           if H is not None else f'失败（行{info[0]}列{info[1]}）'))

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
