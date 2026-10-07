#!/usr/bin/env python3
"""把触标要用的 5 个格点画在相机画面上

为什么不能只用文字描述格点
--------------------------
`grid (0,0)` 定义成"棋盘检测出的第一个内角点"，而这个"第一个"是
`findChessboardCorners` 定的，**不一定等于用户眼里的左上角**。
只用文字说"从左往右第 3 个十字"，只要检测顺序和直觉差 90° 或翻转，
用户就会照着做、还觉得自己做对了 —— 五个点全错位。

所以：直接把目标点投影回像素、画在画面上、编号，让用户照着图点。
"""

from __future__ import annotations

import argparse
import sys
import time

import cv2
import numpy as np

# 与 extrinsic_calib_multi.py 的 DEFAULT_POINTS_CM 保持一致
POINTS_CM = [(0.0, 0.0), (9.9, 0.0), (0.0, 6.6), (9.9, 6.6), (3.3, 3.3)]


def grab_from_url(url, timeout=10.0):
    """从正在运行的相机前端的 MJPEG 流里借一帧。

    为什么需要：V4L2 设备是**独占**的。只要标定前端 / 颜色识别前端还开着，
    别的程序 `cv2.VideoCapture(0)` 就会失败（`can't open camera by index`）。
    而把那些前端一个个杀掉既粗暴、又会打断别人正在做的事。
    既然它们本身就在把画面以 MJPEG 推出来，直接借一帧最省事。
    """
    import urllib.request
    buf = b''
    with urllib.request.urlopen(url, timeout=timeout) as resp:
        while len(buf) < 4_000_000:
            chunk = resp.read(4096)
            if not chunk:
                break
            buf += chunk
            s = buf.find(b'\xff\xd8')          # JPEG 起始
            e = buf.find(b'\xff\xd9', s + 2)   # JPEG 结束
            if s >= 0 and e > s:
                arr = np.frombuffer(buf[s:e + 2], np.uint8)
                return cv2.imdecode(arr, cv2.IMREAD_COLOR)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cols', type=int, default=7)
    ap.add_argument('--rows', type=int, default=5)
    ap.add_argument('--cell-mm', type=float, default=33.0)
    ap.add_argument('--out', default='/tmp/touch_points.png')
    ap.add_argument('--undistort', action='store_true', default=True)
    ap.add_argument('--frame-url',
                    help='从正在运行的相机前端借一帧，例如 '
                         'http://127.0.0.1:8097/stream.mjpg '
                         '（避免和它抢 V4L2 独占）')
    ap.add_argument('--points', default=None,
                    help='要画的格点，"x,y;x,y;..." 单位 cm。'
                         '**必须和 extrinsic_calib_multi.py --points 完全一致**，'
                         '否则你对着图打点、程序按另一套坐标解，结果全错。'
                         '不给则用内置 5 点。')
    ap.add_argument('--camera', type=int, default=0,
                    help='直接开相机（没有 --frame-url 时用）')
    args = ap.parse_args()

    global POINTS_CM
    if args.points:
        try:
            POINTS_CM = [tuple(float(v) for v in p.split(','))
                         for p in args.points.split(';') if p.strip()]
            assert all(len(t) == 2 for t in POINTS_CM)
        except Exception as exc:  # noqa: BLE001
            print(f'❌ --points 解析失败: {exc}')
            return 1
        print(f'使用 {len(POINTS_CM)} 个自定义格点')

    if args.frame_url:
        print(f'从 {args.frame_url} 借一帧 …')
        frame = grab_from_url(args.frame_url)
        if frame is None:
            print('❌ 借帧失败')
            return 1
        print(f'  ✅ 拿到 {frame.shape[1]}x{frame.shape[0]}')
    else:
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not cap.isOpened():
            print('❌ 打不开相机。')
            print('   如果标定/颜色识别前端正在运行，V4L2 是独占的 —— '
                  '用 --frame-url 从它那儿借一帧，别去杀它。')
            return 1
        time.sleep(1.0)
        for _ in range(8):
            cap.read()
        ok, frame = cap.read()
        cap.release()
        if not ok:
            print('❌ 读帧失败')
            return 1

    # 借来的帧可能是"原图 | 掩码"并排的**合成图**（实测颜色识别前端推的是
    # 1280x480 = 640 原图 + 640 掩码）。内参是对 640x480 标定的，
    # 不切的话去畸变会用错尺寸、棋盘也检不出来。
    if args.frame_url and frame.shape[1] > frame.shape[0]:
        half = frame.shape[1] // 2
        if half == frame.shape[0] or half == 640:
            print(f'  检测到并排合成图 {frame.shape[1]}x{frame.shape[0]}，'
                  f'切出左半边 {half}x{frame.shape[0]}')
            frame = frame[:, :half]

    # 去畸变后再检 —— 否则投影回像素时会把畸变算重
    if args.undistort:
        try:
            from block_pipeline import load_intrinsics
            K, D, meta = load_intrinsics()
            if K is not None:
                frame = cv2.undistort(frame, K, D)
                print(f'已用内参去畸变（{meta.get("note", "")[:60]}）')
            else:
                print(f'⚠️ 内参不可用（{D}），用原图 —— 目标点会有几个像素偏差')
        except Exception as exc:  # noqa: BLE001
            print(f'⚠️ 去畸变跳过：{exc}')

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, (args.cols, args.rows),
                                               flags)
    if not found:
        cv2.imwrite(args.out, frame)
        print(f'❌ 没检测到棋盘（是不是被机械臂挡住了？）。'
              f'原图已存 {args.out}')
        return 1
    c = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                          30, 0.001))
    px = c.reshape(-1, 2)
    cell = args.cell_mm / 1000.0
    obj = np.zeros((args.rows * args.cols, 2), np.float32)
    obj[:, :] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    obj *= cell
    H, _ = cv2.findHomography(px.astype(np.float32), obj, 0)
    Hinv = np.linalg.inv(H)

    vis = frame.copy()
    # 先画出检测到的第一个和最后一个角点，帮助判断检测顺序
    for idx, col, tag in ((0, (255, 128, 0), 'first'), (len(px) - 1, (0, 165, 255),
                                                       'last')):
        p = tuple(np.round(px[idx]).astype(int))
        cv2.circle(vis, p, 5, col, 2)
        cv2.putText(vis, tag, (p[0] + 6, p[1] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv2.LINE_AA)

    print()
    print(f'{"#":>2}{"grid(cm)":>14}{"像素":>16}{"到最近角点":>12}')
    print('-' * 46)
    worst = 0.0
    for i, (gx, gy) in enumerate(POINTS_CM, 1):
        g = np.array([gx / 100.0, gy / 100.0, 1.0])
        p = Hinv @ g
        p = p[:2] / p[2]
        pt = tuple(np.round(p).astype(int))
        # 自检：这 5 个点都是**格点**（整数格坐标），所以必须和检测到的
        # 内角点重合。偏得多就说明单应拟合有问题或角点顺序不对 ——
        # 那种情况下用户照着图点会全错，得先拦住。
        d = float(np.linalg.norm(px - p, axis=1).min())
        worst = max(worst, d)
        cv2.circle(vis, pt, 13, (0, 0, 255), 2)
        cv2.circle(vis, pt, 3, (0, 0, 255), -1)
        cv2.putText(vis, str(i), (pt[0] - 6, pt[1] - 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2,
                    cv2.LINE_AA)
        print(f'{i:>2}{f"({gx:.1f}, {gy:.1f})":>14}'
              f'{f"({pt[0]}, {pt[1]})":>16}{d:>11.2f}px')
    print()
    if worst > 2.0:
        print(f'  ❌ 最大偏离 {worst:.2f}px —— 目标点没落在检测角点上，'
              f'角点顺序或单应有问题，**先别照图点**')
        return 2
    print(f'  ✅ 5 个点都落在检测角点上（最大偏离 {worst:.2f}px）')

    cv2.imwrite(args.out, vis)
    print()
    print(f'📄 已写 {args.out}')
    print('  红圈+编号 1~5 = 要触标的五个十字交点，照着图点即可')
    print('  橙圈 first / last = 检测器认定的首末角点（用来判断编号方向）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
