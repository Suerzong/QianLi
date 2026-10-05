#!/usr/bin/env python3
"""第3步：网格纸标定 + 物块物理定位

用 3.3cm 网格纸的交叉点做 Homography 标定：
  像素坐标 (u,v)  ↔  物理坐标 (row×3.3, col×3.3) cm

交互流程：
  1. 框选 ROI（方格纸区域）
  2. 进入标定点采集模式：
     - 鼠标左键点击一个【网格线交叉点】
     - 终端输入该点所在的行列号（空格分隔，如 "3 4"）
     - 至少采 4 个点（推荐 6-9 个，分布开）
     - 采完按键盘 t 计算标定
  3. 实时检测物块 → 画面显示物理坐标 (Xcm, Ycm)
     同时写入 /tmp/object_pose.txt

按键：t=算标定  s=保存  q=退出  r=重框 ROI  c=清除点重采
"""

import cv2
import numpy as np
import sys

CELL_CM = 3.3          # 网格边长（厘米）—— 你的网格纸
MIN_AREA = 150         # 物块最小面积（诊断实测 391px）
MIN_CALIB_PTS = 4      # 最少标定点数

# 状态
src_pts = []           # 标定点像素坐标 [(u,v), ...]
dst_pts = []           # 标定点物理坐标 [(x_cm, y_cm), ...]
homography = None      # 3x3 变换矩阵
roi = None             # (x, y, w, h)


def on_mouse(event, x, y, flags, param):
    """鼠标回调：在标定模式下记录交叉点像素坐标。"""
    global src_pts, homography
    if homography is not None:
        return  # 标定完成后点击无效
    if event == cv2.EVENT_LBUTTONDOWN:
        # 只在 ROI 内取点
        rx, ry, rw, rh = roi
        if rx <= x <= rx + rw and ry <= y <= ry + rh:
            src_pts.append((x, y))
            print(f'>>> 已记录像素点 #{len(src_pts)}: ({x}, {y})')
            print('    请输入该点所在【行列号】(如 2 3 = 第2行第3列交叉点):')
        else:
            print('!!! 点击在 ROI 外，请点在方格纸区域内')


def main():
    global roi, homography
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

    print('>>> 第一步：用鼠标框选【方格纸】区域，回车确认')
    r = cv2.selectROI('set ROI', frame, showCrosshair=True, fromCenter=False)
    rx, ry, rw, rh = r
    if rw <= 0 or rh <= 0:
        rx, ry, rw, rh = 0, 0, frame.shape[1], frame.shape[0]
    roi = (rx, ry, rw, rh)
    cv2.destroyWindow('set ROI')

    cv2.namedWindow('Calibrate')
    cv2.setMouseCallback('Calibrate', on_mouse)

    print('>>> 第二步：采集标定点（最少 4 个，推荐 6-9 个）')
    print('    左键点击网格交叉点 → 终端输入行列号（如 "2 3"）')
    print('    采完后按键盘 t 计算标定，c 清空重采')

    def input_loop():
        """后台线程：读行列号输入并换算物理坐标。"""
        global src_pts, dst_pts
        while True:
            line = sys.stdin.readline().strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) == 2 and parts[0].isdigit() and parts[1].isdigit():
                row, col = int(parts[0]), int(parts[1])
                # 物理坐标：x = col*CELL_CM, y = row*CELL_CM（厘米）
                x_cm = col * CELL_CM
                y_cm = row * CELL_CM
                dst_pts.append((x_cm, y_cm))
                print(f'>>> 点 #{len(dst_pts)} 物理坐标: '
                      f'({x_cm:.1f}, {y_cm:.1f}) cm '
                      f'(第{row}行第{col}列)')
                if len(src_pts) == len(dst_pts):
                    print('    当前已配对标定点 '
                          f'{len(dst_pts)}/{len(src_pts)}，'
                          '继续点击或按 t 计算')
            else:
                print('格式错误，请输入两个整数，如 "2 3"')

    import threading
    threading.Thread(target=input_loop, daemon=True).start()

    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        display = frame.copy()
        cv2.rectangle(display, (rx, ry), (rx + rw, ry + rh), (255, 0, 0), 2)

        # 画标定点
        for (u, v) in src_pts:
            cv2.circle(display, (u, v), 5, (0, 255, 255), -1)
            cv2.putText(display, 'P', (u + 5, v - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # 标定完成后：检测物块并换算物理坐标
        object_pose = None
        if homography is not None:
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
                # 像素 → 物理
                p = np.array([[[gx, gy]]], dtype=np.float64)
                phy = cv2.perspectiveTransform(p, homography)
                Xcm, Ycm = phy[0][0]
                object_pose = (gx, gy, Xcm, Ycm, bw, bh)
                # 画物块
                cv2.rectangle(display, (gx - bw // 2, gy - bh // 2),
                              (gx + bw // 2, gy + bh // 2), (0, 255, 0), 3)
                cv2.putText(display,
                            f'({Xcm:.1f}cm, {Ycm:.1f}cm)',
                            (gx + 10, gy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 0), 2)
                # 物理坐标落盘
                try:
                    with open('/tmp/object_pose.txt', 'w') as f:
                        f.write(f'X_cm={Xcm:.2f}\nY_cm={Ycm:.2f}\n'
                                f'center_px=({gx},{gy})\n'
                                f'size_px={bw}x{bh}\n')
                except OSError:
                    pass

        # 状态栏
        status = 'collecting calib pts'
        if homography is not None:
            status = f'CALIBRATED | obj: ' + (
                f'({object_pose[2]:.1f}, {object_pose[3]:.1f})cm'
                if object_pose else 'not found')
        cv2.putText(display, status, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(display, f'calib pts: {len(src_pts)} | '
                    't=calib c=clear q=quit',
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (200, 200, 200), 1)

        cv2.imshow('Calibrate', display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            src_pts = []
            dst_pts = []
            homography = None
            print('>>> 已清空标定点，重新采集')
        elif key == ord('t'):
            if len(src_pts) < MIN_CALIB_PTS:
                print(f'!!! 标定点不足（{len(src_pts)} < {MIN_CALIB_PTS}），'
                      '请再采几个点')
            elif len(src_pts) != len(dst_pts):
                print(f'!!! 有 {len(src_pts)} 个像素点但只有 '
                      f'{len(dst_pts)} 个物理坐标（有未输入行列号的点）')
            else:
                src = np.array(src_pts, dtype=np.float32)
                dst = np.array(dst_pts, dtype=np.float32)
                H, inliers = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
                if H is None:
                    print('!!! 标定失败：点分布不好，按 c 清空重采')
                else:
                    homography = H
                    print('>>> 标定完成！H = ')
                    print(np.round(H, 4))
                    print(f'    用 {inliers} 个内点。'
                          '现在物块会实时显示物理坐标(cm)')
        elif key == ord('s'):
            cv2.imwrite('/tmp/calib_result.png', display)
            if homography is not None:
                np.save('/tmp/homography.npy', homography)
                print('已保存标定结果 /tmp/homography.npy 和截图')

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
