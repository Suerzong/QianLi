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

import threading

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

# ---- 默认参数（已实机调优）----
CELL_CM = 3.3        # 棋盘格边长（厘米）
BOARD_COLS = 7       # 内角点列数
BOARD_ROWS = 5       # 内角点行数
MIN_AREA, MAX_AREA = 330, 350    # 物块轮廓面积区间（像素）
MIN_SIZE, MAX_SIZE = 20, 50      # 物块边长区间（像素）
MAX_DIFF = 5                     # 长宽差上限（正方形性）
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

        self.gui = self.get_parameter('gui').value
        self.H = None            # 像素 → 物理（厘米）
        self.origin_px = None    # 原点像素坐标
        self.corners = None      # 内角点（像素）
        self.reproj_err = None   # 重投影误差（厘米）
        self.frame_i = 0

        self.pub = self.create_publisher(PointStamped, '/object_pose', 10)

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('无法打开相机 /dev/video0')
            raise SystemExit(1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.timer = self.create_timer(0.1, self.tick)
        # 棋盘格标定放独立线程做一次（不阻塞 executor）
        threading.Thread(target=self._initial_calibrate, daemon=True).start()

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
        H, _ = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
        if H is None:
            self.get_logger().warn('Homography 计算失败')
            return False

        proj = cv2.perspectiveTransform(
            src.reshape(-1, 1, 2), H).reshape(-1, 2)
        err = np.linalg.norm(proj - dst, axis=1)

        self.H = H
        self.corners = pts
        self.origin_px = tuple(pts[0])
        self.reproj_err = (float(err.mean()), float(err.max()))
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
            return
        self.frame_i += 1

        # 每 30 帧（约 3 秒）重新标定一次，抵抗棋盘被碰动
        if self.H is None or self.frame_i % 30 == 0:
            self._calibrate(frame)

        obj = self._detect_object(frame)

        if obj is not None and self.H is not None:
            gx, gy, bw, bh, area = obj
            p = np.array([[[gx, gy]]], dtype=np.float64)
            Xcm, Ycm = cv2.perspectiveTransform(p, self.H)[0][0]

            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'grid'
            msg.point.x = Xcm / 100.0
            msg.point.y = Ycm / 100.0
            msg.point.z = 0.0
            self.pub.publish(msg)

            if self.gui:
                cv2.putText(frame, f'({Xcm:.1f}, {Ycm:.1f}) cm',
                            (gx + 12, gy - 12), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 0), 2)
            # 落盘便于 SSH 侧读取
            try:
                with open('/tmp/object_pose.txt', 'w') as f:
                    f.write(f'X_cm={Xcm:.2f}\nY_cm={Ycm:.2f}\n'
                            f'center_px=({gx},{gy})\n'
                            f'size_px={bw}x{bh}\narea={int(area)}\n')
            except OSError:
                pass

        if self.gui:
            self._draw(frame, obj)

    # ---------- 物块检测 ----------
    def _detect_object(self, frame):
        """返回最长边符合、面积符合的候选 (cx, cy, bw, bh, area)，否则 None。"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, V_MIN), (179, S_MAX, V_MAX))
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        self._mask = mask  # 供 GUI 显示

        a_lo = self.get_parameter('min_area').value
        a_hi = self.get_parameter('max_area').value
        s_lo = self.get_parameter('min_size').value
        s_hi = self.get_parameter('max_size').value
        d_max = self.get_parameter('max_diff').value

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        best = None
        for c in cnts:
            area = cv2.contourArea(c)
            if not (a_lo <= area <= a_hi):      # 面积区间
                continue
            bx, by, bw, bh = cv2.boundingRect(c)
            if d_max > 0 and abs(bw - bh) > d_max:   # 正方形性
                continue
            if not (s_lo <= max(bw, bh) <= s_hi):    # 边长区间
                continue
            if best is None or area > best[4]:
                best = (bx + bw // 2, by + bh // 2, bw, bh, area)
        return best

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
            cx, cy, bw, bh, area = obj
            cv2.rectangle(frame, (cx - bw // 2, cy - bh // 2),
                          (cx + bw // 2, cy + bh // 2), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)

        status = ('CALIB OK' if self.H is not None else 'NO CALIB')
        if self.reproj_err:
            status += f' err {self.reproj_err[0]:.3f}cm'
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
