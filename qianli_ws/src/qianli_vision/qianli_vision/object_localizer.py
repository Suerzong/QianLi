#!/usr/bin/env python3
"""qianli_vision object_localizer：全自动物块定位（棋盘格标定）

流程（全自动，无需框 ROI / 手点格点）：
  1. 自动识别棋盘格内角点（findChessboardCorners，亚像素精度）
  2. 建立 像素 ↔ 物理 映射（Homography）
     原点 = 棋盘第一个内角点（左上角"十字"交点）
     x 轴沿棋盘列方向，y 轴沿棋盘行方向，格边长 = cell_cm
  3. 检测灰色物块（HSV 阈值 + 面积区间 + 正方形 + 边长过滤）
  4. 物块中心 → Homography → 物理坐标 (Xcm, Ycm)
  5. 发布 /object_pose（geometry_msgs/PointStamped，frame_id='grid'）

参数：
  cell_cm   棋盘方格边长（厘米，默认 3.3）
  cols      棋盘内角点列数（默认 7，即 8 个方格）
  rows      棋盘内角点行数（默认 5，即 6 个方格）
  gui       是否显示窗口（默认 true）
  min_area / max_area   物块轮廓面积区间（默认 330 / 350）
  max_diff  长宽差上限（默认 5，立方体投影接近正方形）
  min_size / max_size   物块边长区间（默认 20 / 50）

用法：
  ros2 run qianli_vision object_localizer
  ros2 run qianli_vision object_localizer --ros-args -p gui:=false
"""

import os
import threading
import time

import math
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64

# ---- 默认参数（已实机调优）----
CELL_CM = 3.3        # 用户确认单格 33mm（2026-10-05）
BOARD_COLS = 7       # 内角点列数
BOARD_ROWS = 5       # 内角点行数
MIN_AREA, MAX_AREA = 250, 600    # 物块轮廓面积区间（像素）
MIN_SIZE, MAX_SIZE = 15, 60      # 物块边长区间（像素）
MAX_DIFF = 15                    # 长宽差上限（bg+gray 混合下放宽）
S_MAX, V_MIN, V_MAX = 127, 140, 167   # 灰色判定（HSV）

# 棋盘格亚像素精化参数
_SUBPIX = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
_CB_FLAGS = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
             | cv2.CALIB_CB_FAST_CHECK)


class ObjectLocalizer(Node):
    def __init__(self):
        super().__init__('object_localizer')

        # ---- 参数 ----
        self.declare_parameter('cell_cm', CELL_CM)
        self.declare_parameter('cols', BOARD_COLS)
        self.declare_parameter('rows', BOARD_ROWS)
        self.declare_parameter('gui', True)
        self.declare_parameter('min_area', MIN_AREA)
        self.declare_parameter('max_area', MAX_AREA)
        self.declare_parameter('min_size', MIN_SIZE)
        self.declare_parameter('max_size', MAX_SIZE)
        self.declare_parameter('max_diff', MAX_DIFF)
        self.declare_parameter('adaptive', True)   # 自适应灰度带（抗光照变化）
        self.declare_parameter('bg_file', '/tmp/board_bg.png')  # 背景差分
        self.declare_parameter('bg_thresh', 25)    # 背景差分阈值（灰度级）
        self.declare_parameter('hybrid', True)     # 背景差分∩灰色带（排除影子/彩色物）
        self.declare_parameter('restrict_to_board', True)  # 候选须在棋盘邻域内
        self.declare_parameter('board_margin', 25)         # 邻域外扩像素

        self.gui = self.get_parameter('gui').value
        self.H = None            # 像素 → 物理（厘米）
        self.origin_px = None    # 原点像素坐标
        self.corners = None      # 内角点（像素）
        self.reproj_err = None   # 重投影误差（厘米）
        self.frame_i = 0
        self.calibrated_at = None
        self.reference_corners = None
        self.calibration_moved = False
        self._last_warn = 0.

        self.pub = self.create_publisher(PointStamped, '/object_pose', 10)
        # 灰色度上限：皮肤饱和度常在 100 上下，收紧到 60 可排除手/彩色工具
        self.declare_parameter('max_sat', 120)
        # 棋盘点位范围（cm）：超出即视为误检丢弃（棋盘 22.8x16.2cm）
        self.declare_parameter('grid_x_limit_cm', BOARD_COLS*CELL_CM)
        self.declare_parameter('grid_y_limit_cm', BOARD_ROWS*CELL_CM)
        self.declare_parameter('object_size_cm', 2.)
        self.declare_parameter('size_tolerance_cm', .8)
        self.pub_yaw = self.create_publisher(Float64, '/object_yaw', 10)
        self.box_pts = None

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('无法打开相机 /dev/video0')
            raise SystemExit(1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.timer = self.create_timer(0.1, self.tick)
        # 棋盘格标定放独立线程做一次（不阻塞 executor）
        # Calibration happens on the executor's first frame. Concurrent
        # VideoCapture.read() calls from a startup thread can race with tick.

    # ---------- 标定 ----------
    def _initial_calibrate(self):
        """启动时抓几帧取中值，做一次棋盘格标定。"""
        import time
        time.sleep(1.0)
        frames = []
        for _ in range(7):
            ok, f = self.cap.read()
            if ok:
                frames.append(f)
        if not frames:
            self.get_logger().error('读帧失败，无法标定')
            return
        frame = np.median(np.array(frames), axis=0).astype(np.uint8)
        self._calibrate(frame)

    def _calibrate(self, frame):
        """识别棋盘格并建立 Homography。成功返回 True。"""
        cols = self.get_parameter('cols').value
        rows = self.get_parameter('rows').value
        cell = self.get_parameter('cell_cm').value

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ok, corners = cv2.findChessboardCorners(gray, (cols, rows), _CB_FLAGS)
        if not ok:
            self.get_logger().warn(
                f'未识别到棋盘格（内角点 {cols}x{rows}）；'
                '请确认棋盘完整可见、光照均匀')
            return False
        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), _SUBPIX)
        pts = corners.reshape(-1, 2)

        # 像素 → 物理：内角点 (i,j) → (j*cell, i*cell)
        src, dst = [], []
        for i in range(rows):
            for j in range(cols):
                px, py = pts[i * cols + j]
                src.append((px, py))
                dst.append((j * cell, i * cell))
        src = np.array(src, np.float32)
        dst = np.array(dst, np.float32)
        H, inliers = cv2.findHomography(src, dst, cv2.RANSAC, 0.15)
        if H is None:
            self.get_logger().warn('Homography 计算失败')
            return False

        proj = cv2.perspectiveTransform(
            src.reshape(-1, 1, 2), H).reshape(-1, 2)
        err = np.linalg.norm(proj - dst, axis=1)
        if inliers is None or inliers.mean() < .8 or err.mean() > .1 or err.max() > .3:
            self._warn_throttled('棋盘标定质量不足，拒绝更新 H')
            return False
        if self.reference_corners is not None:
            shift = np.linalg.norm(pts-self.reference_corners, axis=1).max()
            if shift > 3.:
                self.calibration_moved = True
                self._warn_throttled('相机/棋盘相对位置变化；旧 base 外参失效，需重标定')
                return False
        else:
            self.reference_corners = pts.copy()

        self.H = H
        self.corners = pts
        self.origin_px = tuple(pts[0])
        self.reproj_err = (float(err.mean()), float(err.max()))
        self.calibrated_at = time.monotonic()
        self.get_logger().info(
            f'✅ 棋盘格标定成功：内角点 {cols}x{rows}，'
            f'覆盖 {cols*cell:.1f}x{rows*cell:.1f} cm，'
            f'重投影误差 平均{err.mean():.3f}cm/最大{err.max():.3f}cm；'
            f'原点像素 ({pts[0][0]:.1f},{pts[0][1]:.1f})（左上第一个内角点）')
        return True

    # ---------- 主循环 ----------
    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self._write_invalid('camera_read_failed')
            return
        self.frame_i += 1

        # 每 30 帧（约 3 秒）重新标定一次，抵抗棋盘被碰动
        if self.H is None or self.frame_i % 30 == 0:
            self._calibrate(frame)

        obj = self._detect_object(frame)
        if (self.H is None or self.calibration_moved or self.calibrated_at is None
                or time.monotonic()-self.calibrated_at > 10.):
            self._write_invalid('calibration_missing_moved_or_stale')
            return
        if obj is None:
            self._write_invalid(getattr(self,'_detection_reason','object_not_found'))

        if obj is not None and self.H is not None:
            gx, gy, bw, bh, area = obj[:5]
            p = np.array([[[gx, gy]]], dtype=np.float64)
            Xcm, Ycm = cv2.perspectiveTransform(p, self.H)[0][0]

            # ★ 棋盘点位范围校验：棋盘 22.8x16.2cm。
            #   注意：物块可能被碰落到棋盘外，所以这里**只警告不丢弃**，
            #   范围放宽到整个桌面（用户要求：动之前检测一次，之后照坐标执行）
            xlim = float(self.get_parameter('grid_x_limit_cm').value)
            ylim = float(self.get_parameter('grid_y_limit_cm').value)
            lower = -float(self.get_parameter('cell_cm').value)
            if not (lower <= Xcm <= xlim and lower <= Ycm <= ylim):
                self._warn_throttled(
                    f'目标在棋盘点位范围外 ({Xcm:.1f},{Ycm:.1f})cm'
                    f'（棋盘 ±({xlim:.0f},{ylim:.0f})cm），拒绝发布')
                self._write_invalid('outside_board')
                return

            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'grid'
            msg.point.x = Xcm / 100.0
            msg.point.y = Ycm / 100.0
            msg.point.z = 0.0
            self.pub.publish(msg)

            # ---- 物块朝向：minAreaRect → 单应变换到 grid 系 → 角度 ----
            # 物块是立方体，90° 旋转等价，所以角度归一化到 [-45°, +45°)
            yaw_deg = None
            self.box_px = None
            try:
                rect = cv2.minAreaRect(obj[5])
                box = cv2.boxPoints(rect).astype(np.float64)
                self.box_px = box.astype(np.int32)
                g = cv2.perspectiveTransform(
                    box.reshape(-1, 1, 2), self.H).reshape(-1, 2)
                e = g[1] - g[0]
                a = math.degrees(math.atan2(e[1], e[0]))
                yaw_deg = (a + 45.0) % 90.0 - 45.0
                self.box_pts = g
            except Exception:
                self.box_pts = None
            if yaw_deg is not None:
                self.pub_yaw.publish(Float64(data=math.radians(yaw_deg)))

            if self.gui:
                cv2.putText(frame, f'({Xcm:.1f}, {Ycm:.1f}) cm',
                            (gx + 12, gy - 12), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 0), 2)
                if yaw_deg is not None:
                    cv2.putText(frame, f'yaw {yaw_deg:+.1f}deg',
                                (gx + 12, gy + 14),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 128, 0), 2)
            # 落盘便于 SSH 侧读取
            try:
                tmp = f'/tmp/object_pose.txt.tmp{os.getpid()}'
                with open(tmp, 'w') as f:
                    f.write(f'valid=1\ntimestamp={time.time():.6f}\n'
                            f'cell_cm={self.get_parameter("cell_cm").value}\n'
                            'coordinate_method=board_plane_projection\n'
                            f'calibration_age_s={time.monotonic()-self.calibrated_at:.3f}\n'
                            f'X_cm={Xcm:.2f}\nY_cm={Ycm:.2f}\n'
                            f'yaw_deg={yaw_deg if yaw_deg is not None else 0.0:.2f}\n'
                            f'center_px=({gx},{gy})\n'
                            f'size_px={bw}x{bh}\narea={int(area)}\n')
                os.replace(tmp,'/tmp/object_pose.txt')
            except OSError:
                pass

        if self.gui:
            self._draw(frame, obj)
        # 每 15 帧存一张调试图（便于 SSH 侧确认检测是否正确）
        if self.frame_i % 15 == 0:
            try:
                cv2.imwrite('/tmp/detect.jpg', frame)
            except Exception:
                pass

    # ---------- 物块检测 ----------
    def _warn_throttled(self, message):
        now = time.monotonic()
        if now-self._last_warn > 2.:
            self.get_logger().warn(message)
            self._last_warn = now

    def _write_invalid(self, reason):
        tmp = f'/tmp/object_pose.txt.tmp{os.getpid()}'
        try:
            with open(tmp,'w') as stream:
                stream.write(f'valid=0\nreason={reason}\ntimestamp={time.time():.6f}\n')
            os.replace(tmp,'/tmp/object_pose.txt')
        except OSError:
            pass

    def _detect_object(self, frame):
        """返回候选 (cx, cy, bw, bh, area)，否则 None。

        灰色带的确定方式（adaptive=true，默认）：
          物块"比棋盘黑格亮、比白格暗" —— 用同一帧里棋盘区域的黑/白亮度
          分位数动态算出灰色带 [lo, hi]，因此**对光照变化免疫**
          （固定 HSV 阈值在灯光变化时会失效）。
        关掉 adaptive 则用固定 V_MIN/V_MAX。
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        v_ch = hsv[:, :, 2]
        s_ch = hsv[:, :, 1]

        # ---- 方法A：背景差分（首选，最稳健）----
        bg = self._load_bg(frame.shape[:2])
        if bg is not None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.int16)
            diff = gray - bg
            # 全局亮度补偿：用棋盘区域中位差抵消整体明暗漂移
            if self.corners is not None:
                xs, ys = self.corners[:, 0], self.corners[:, 1]
                x0, x1 = int(xs.min()), int(xs.max())
                y0, y1 = int(ys.min()), int(ys.max())
                offset = float(np.median(diff[max(0, y0):y1,
                                              max(0, x0):x1]))
            else:
                offset = float(np.median(diff))
            d = np.abs(diff - offset).astype(np.uint8)
            thr = int(self.get_parameter('bg_thresh').value)
            mask_bg = cv2.inRange(d, thr, 255)

            # 混合：与"灰色带"掩码取交集 → 只保留灰色物块本体，
            # 排除影子（偏暗）、彩色线缆（饱和度高）、棋盘边缘伪影
            if self.get_parameter('hybrid').value and self.corners is not None:
                xs, ys = self.corners[:, 0], self.corners[:, 1]
                x0, x1 = int(xs.min()), int(xs.max())
                y0, y1 = int(ys.min()), int(ys.max())
                vv = v_ch[max(0, y0):y1, max(0, x0):x1].ravel()
                if vv.size > 100:
                    v_white = float(np.percentile(vv, 92))
                    v_black = float(np.percentile(vv, 8))
                    span = max(1.0, v_white - v_black)
                    lo = int(v_black + 0.55 * span)
                    hi = int(v_white - 0.03 * span)
                    self._band = (lo, hi, v_black, v_white)
                    mask_gray = cv2.inRange(v_ch, lo, hi)
                    mask_gray &= cv2.inRange(s_ch, 0, self.get_parameter('max_sat').value)
                    mask = cv2.bitwise_and(mask_bg, mask_gray)
                    self._method = (f'bg+gray thr={thr} '
                                    f'band={lo}-{hi}')
                else:
                    mask = mask_bg
                    self._method = f'bg-sub thr={thr}'
            else:
                mask = mask_bg
                self._method = f'bg-sub thr={thr} offset={offset:+.0f}'

            k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
            self._mask = mask
            return self._filter_contours(mask)

        # ---- 方法B：自适应灰度带（无背景图时的回退）----
        if self.get_parameter('adaptive').value and self.corners is not None:
            xs, ys = self.corners[:, 0], self.corners[:, 1]
            x0, x1 = int(xs.min()), int(xs.max())
            y0, y1 = int(ys.min()), int(ys.max())
            vv = v_ch[max(0, y0):y1, max(0, x0):x1].ravel()
            if vv.size > 100:
                v_white = float(np.percentile(vv, 92))
                v_black = float(np.percentile(vv, 8))
                span = max(1.0, v_white - v_black)
                lo = int(v_black + 0.55 * span)
                hi = int(v_white - 0.03 * span)
                self._band = (lo, hi, v_black, v_white)
            else:
                lo, hi = V_MIN, V_MAX
                self._band = None
        else:
            lo, hi = V_MIN, V_MAX
            self._band = None

        mask = cv2.inRange(v_ch, lo, hi)
        if S_MAX < 255:      # 饱和度上限（可选）
            mask &= cv2.inRange(s_ch, 0, self.get_parameter('max_sat').value)
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        self._mask = mask
        self._method = 'adaptive-gray' if self._band else 'fixed-hsv'
        return self._filter_contours(mask)

    def _load_bg(self, shape):
        """懒加载背景图（灰度，int16）。文件变化时自动重载。"""
        path = self.get_parameter('bg_file').value
        if not path or not os.path.exists(path):
            return None
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            return None
        if getattr(self, '_bg_mtime', None) != mtime:
            bg = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if bg is None:
                return None
            if bg.shape[:2] != tuple(shape):
                self.get_logger().warn(
                    f'背景图尺寸 {bg.shape[:2]} != 画面 {tuple(shape)}，忽略')
                self._bg_mtime = mtime
                self._bg = None
                return None
            self._bg = bg.astype(np.int16)
            self._bg_mtime = mtime
            self.get_logger().info(f'已加载背景图: {path}')
        return getattr(self, '_bg', None)

    def _filter_contours(self, mask):
        """按 面积区间 / 正方形性 / 边长区间 / 棋盘邻域 过滤，返回最佳候选。

        restrict_to_board（默认开）：候选中心必须落在棋盘外扩 margin 的
        矩形内 —— 这样线缆/桌面杂物的移动即使面积合适也会被排除
        （它们离棋盘远）。
        """
        a_lo = self.get_parameter('min_area').value
        a_hi = self.get_parameter('max_area').value
        s_lo = self.get_parameter('min_size').value
        s_hi = self.get_parameter('max_size').value
        d_max = self.get_parameter('max_diff').value
        use_board = self.get_parameter('restrict_to_board').value
        margin = int(self.get_parameter('board_margin').value)

        bx0 = by0 = -10 ** 6
        bx1 = by1 = 10 ** 6
        if use_board and self.corners is not None:
            xs, ys = self.corners[:, 0], self.corners[:, 1]
            bx0, bx1 = int(xs.min()) - margin, int(xs.max()) + margin
            by0, by1 = int(ys.min()) - margin, int(ys.max()) + margin

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        self._detection_reason = 'object_not_found'
        for c in cnts:
            area = cv2.contourArea(c)
            if not (a_lo <= area <= a_hi):
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            if d_max > 0 and abs(bw - bh) > d_max:
                continue
            if not (s_lo <= max(bw, bh) <= s_hi):
                continue
            cx, cy = x + bw // 2, y + bh // 2
            if not (bx0 <= cx <= bx1 and by0 <= cy <= by1):
                continue          # 离棋盘太远（线缆/桌面杂物）
            if area/(bw*bh) < .6:
                continue
            if self.H is not None:
                box = cv2.boxPoints(cv2.minAreaRect(c)).astype(np.float32)
                mapped = cv2.perspectiveTransform(box.reshape(-1,1,2),self.H).reshape(-1,2)
                sides = np.linalg.norm(np.roll(mapped,-1,axis=0)-mapped,axis=1)
                expected = float(self.get_parameter('object_size_cm').value)
                tolerance = float(self.get_parameter('size_tolerance_cm').value)
                if np.any(np.abs(sides-expected) > tolerance):
                    continue
            candidates.append((cx,cy,bw,bh,area,c))
        if len(candidates) > 1:
            self._detection_reason = 'ambiguous_candidates'
            return None
        if candidates:
            self._detection_reason = 'detected'
            return candidates[0]
        return None

    # ---------- GUI ----------
    def _draw(self, frame, obj):
        # 棋盘格内角点 + 原点
        if self.corners is not None:
            for idx, (px, py) in enumerate(self.corners):
                cv2.circle(frame, (int(px), int(py)), 3, (0, 255, 0), -1)
            ox, oy = self.origin_px
            cv2.circle(frame, (int(ox), int(oy)), 10, (0, 0, 255), 2)
            cv2.putText(frame, 'ORIGIN(0,0)', (int(ox) + 12, int(oy) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(frame, 'X->right  Y->down (cm)',
                        (int(ox) + 12, int(oy) + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

        if obj is not None:
            cx, cy, bw, bh, area = obj[:5]
            cv2.rectangle(frame, (cx - bw // 2, cy - bh // 2),
                          (cx + bw // 2, cy + bh // 2), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)
            if getattr(self, 'box_px', None) is not None:
                cv2.polylines(frame, [self.box_px], True, (255, 128, 0), 2)

        status = ('CALIB OK' if self.H is not None else 'NO CALIB')
        if self.reproj_err:
            status += f' err {self.reproj_err[0]:.3f}cm'
        if getattr(self, '_band', None):
            lo, hi, vb, vw = self._band
            status += f' | band {lo}-{hi} (blk{int(vb)}/wht{int(vw)})'
        status += f' | {getattr(self, "_method", "?")}'
        status += ' | obj FOUND' if obj else ' | obj not found'
        cv2.putText(frame, status, (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        cv2.putText(frame, 'grid origin = red circle', (10, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        cv2.imshow('object_localizer', frame)
        if hasattr(self, '_mask'):
            cv2.imshow('mask', self._mask)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            raise SystemExit(0)


def main():
    rclpy.init()
    node = ObjectLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
