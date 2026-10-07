#!/usr/bin/env python3
"""相机侧：解棋盘在相机坐标系下的 6 自由度位姿（纯棋盘，标准 OpenCV）

为什么走这条路而不是 ArUco 手眼标定
------------------------------------
标准手眼标定（AX=XB）要求**目标物与相机之间有相对运动**。我们的相机和
棋盘**都固定**，两者之间没有相对运动，AX=XB 根本解不出来。ArUco 那套能
工作只是因为把标记贴到了夹爪上、让它跟着机械臂动 —— 那是变通，不是必需。

棋盘本身就是极好的标定物：35 个角点、亚像素精度、几何已知。所以：
    相机侧  cv2.solvePnP          → T_cam_board   （本脚本）
    机械臂侧 触点 + Umeyama        → T_base_board  （extrinsic_calib_multi.py）
    合成                          → T_base_cam

为什么多帧平均
--------------
单帧 solvePnP 实测重投影 RMS 1.04px、最大 2.95px；而内参本身的 RMS 只有
0.39px。多抓几帧取位姿中值可以把角点检测的随机噪声压下去（棋盘静止，
所以取中值是合法的）。同时报告帧间离散度 —— 那才是这次测量的真实不确定度。

用法::

    ~/mj/bin/python board_pose_cam.py --frames 20 --out ~/QianLi/calib/board_cam.npz
"""

from __future__ import annotations

from project_paths import open_video_capture

from project_paths import calibration_path, camera_source, default_camera

import argparse
import os
import sys
import time

import cv2
import numpy as np

COLS, ROWS = 7, 5
CELL = 0.033
INTR_DEFAULT = os.path.expanduser(calibration_path('camera_intrinsics.yaml'))


def load_intrinsics(path):
    K = D = None
    size = [None, None]
    mats, cur = [], None
    for ln in open(path, encoding='utf-8-sig'):
        s = ln.strip()
        if s.startswith('image_width:'):
            size[0] = int(float(s.split(':')[1]))
        elif s.startswith('image_height:'):
            size[1] = int(float(s.split(':')[1]))
        elif s.startswith('camera_matrix:'):
            cur = 'K'
        elif s.startswith('distortion_coefficients:'):
            cur = 'D'
        elif s.startswith('- [') and cur == 'K':
            mats.append([float(v)
                         for v in s[s.index('[') + 1:s.rindex(']')].split(',')])
        elif s.startswith('- [') and cur == 'D':
            D = np.array([float(v)
                          for v in s[s.index('[') + 1:s.rindex(']')].split(',')])
    return np.array(mats), D, size


def board_object_points():
    """棋盘坐标：原点在**第一个内角点**，x 沿列、y 沿行、z 垂直板面。
    这和 extrinsic_calib_multi.py 的 grid 约定一致，所以两边能直接合成。"""
    obj = np.zeros((ROWS * COLS, 3), np.float32)
    obj[:, :2] = np.array([[c, r] for r in range(ROWS)
                           for c in range(COLS)]) * CELL
    return obj


def detect(frame, K, D, mode='raw_points'):
    """检棋盘角点。

    mode='raw_points'（默认，标准做法）
        在**原始畸变图**上检角点，再对亚像素角点做 undistortPoints。
        为什么这样更好：在去畸变图上检会先重采样一次，插值会把角点糊掉
        （尤其画面边缘畸变大的地方），角点定位精度直接受损。
    mode='undist_image'
        先 undistort 整幅图再检。旧做法，留作对比。
    """
    if mode == 'undist_image':
        und = cv2.undistort(frame, K, D, None, K)
        gray = cv2.cvtColor(und, cv2.COLOR_BGR2GRAY)
        ok, corners = cv2.findChessboardCorners(
            gray, (COLS, ROWS),
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            return None, None
        corners = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        return und, corners
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ok, corners = cv2.findChessboardCorners(
        gray, (COLS, ROWS),
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        return None, None
    corners = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
    # 亚像素角点：直接用畸变系数映射到理想像素坐标，不经图像重采样
    und_corners = cv2.undistortPoints(corners, K, D, P=K)
    und = cv2.undistort(frame, K, D, None, K)
    return und, und_corners


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--intrinsics', default=INTR_DEFAULT)
    ap.add_argument('--camera', type=camera_source, default=default_camera())
    ap.add_argument('--frames', type=int, default=20)
    ap.add_argument('--out', default=os.path.expanduser(
        calibration_path('board_cam.npz')))
    ap.add_argument('--save-vis', default='/tmp/board_cam_vis.png')
    ap.add_argument('--mode', default='raw_points',
                    choices=['raw_points', 'undist_image', 'both'],
                    help='角点检测方式；both = 两种都跑并对比')
    ap.add_argument('--pnp', default='ITERATIVE',
                    choices=['ITERATIVE', 'IPPE', 'IPPE_SQUARE'],
                    help='solvePnP 方法。IPPE 是专为平面靶标设计的，'
                         '通常比通用的 ITERATIVE 更准')
    args = ap.parse_args()

    K, D, size = load_intrinsics(args.intrinsics)
    w, h = size
    obj = board_object_points()
    print(f'内参 {w}x{h}  fx={K[0,0]:.2f} fy={K[1,1]:.2f}')
    print(f'棋盘 {COLS}x{ROWS} 内角点，格边长 {CELL*1000:.1f}mm')

    cap = open_video_capture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    time.sleep(1.5)
    for _ in range(10):
        cap.read()

    poses, errs, vis = [], [], None
    tried = 0
    FLAG = {'ITERATIVE': cv2.SOLVEPNP_ITERATIVE,
            'IPPE': cv2.SOLVEPNP_IPPE,
            'IPPE_SQUARE': cv2.SOLVEPNP_IPPE_SQUARE}[args.pnp]
    while len(poses) < args.frames and tried < args.frames * 6:
        tried += 1
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue
        und, corners = detect(frame, K, D, args.mode)
        if corners is None:
            continue
        okp, rvec, tvec = cv2.solvePnP(obj, corners, K, D, flags=FLAG)
        if not okp:
            continue
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
        e = np.linalg.norm(proj.reshape(-1, 2) - corners.reshape(-1, 2), axis=1)
        poses.append(np.concatenate([rvec.ravel(), tvec.ravel()]))
        errs.append((e.mean(), e.max()))
        if vis is None:
            vis = und.copy()
            cv2.drawChessboardCorners(vis, (COLS, ROWS), corners, True)
            cv2.imwrite('/tmp/board_cam_last.png', vis)
    cap.release()

    if len(poses) < 3:
        print(f'❌ 只采到 {len(poses)} 帧有效（需要 ≥3）。'
              f'检查棋盘是否完整可见、光照是否够。')
        return 1

    P = np.array(poses)
    rvec_med = np.median(P[:, :3], axis=0)
    tvec_med = np.median(P[:, 3:], axis=0)
    rvec_std = P[:, :3].std(axis=0)
    tvec_std = P[:, 3:].std(axis=0)
    errs = np.array(errs)

    print()
    print(f'有效帧 {len(poses)}/{tried}')
    print(f'单帧重投影: RMS {errs[:,0].mean():.3f}px  '
          f'最大 {errs[:,1].max():.3f}px')
    print(f'位姿中值 t = ({tvec_med[0]:+.5f}, {tvec_med[1]:+.5f}, '
          f'{tvec_med[2]:+.5f}) m')
    print(f'帧间离散 (标准差): x {tvec_std[0]*1000:.3f}  '
          f'y {tvec_std[1]*1000:.3f}  z {tvec_std[2]*1000:.3f} mm')
    print(f'  → 距离相机 {np.linalg.norm(tvec_med):.4f} m')

    R, _ = cv2.Rodrigues(rvec_med)
    n = R[:, 2]
    print(f'棋盘法向(相机系) = ({n[0]:+.4f}, {n[1]:+.4f}, {n[2]:+.4f})  '
          f'偏离相机轴 {np.degrees(np.arccos(min(1.0, abs(n[2])))):.2f}°')

    # 用中值位姿重新算一遍重投影，这才是"最终用出去的那个位姿"的误差
    proj, _ = cv2.projectPoints(obj, rvec_med, tvec_med, K, D)
    if vis is not None:
        vis = cv2.undistort(cv2.imread('/tmp/board_cam_last.png')
                            if os.path.exists('/tmp/board_cam_last.png')
                            else vis, K, D, None, K)
    np.savez(args.out, rvec=rvec_med, tvec=tvec_med, K=K, D=D,
             rvec_std=rvec_std, tvec_std=tvec_std,
             n_frames=len(poses), cell=CELL, cols=COLS, rows=ROWS)
    print(f'\n📄 已写 {args.out}')
    print('   下一步：用 extrinsic_calib_multi.py 触标得到 T_base_board，'
          '再跑 compose_extrinsic.py 合成并做反投影校验。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
