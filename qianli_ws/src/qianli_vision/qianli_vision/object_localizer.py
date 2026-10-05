#!/usr/bin/env python3
"""qianli_vision object_localizer：RGB 物块检测 + 网格纸标定 + 物理坐标发布

功能：
  1. 自动检测网格纸网格线（Hough）→ Homography 标定（像素 ↔ 厘米）
  2. 检测灰色物块 → 计算物理坐标 (Xcm, Ycm)
  3. 发布到 ROS 话题 /object_pose（geometry_msgs/PointStamped）
     frame_id = 'grid'（网格纸坐标系，原点 = 网格纸某角）
  4. 可选 GUI 显示（--gui）

用法：
  ros2 run qianli_vision object_localizer --ros-args -p cell_cm:=3.3 -p gui:=true
"""

import argparse
import threading

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

CELL_CM = 3.3       # 网格边长（厘米）
MIN_AREA = 100      # 物块最小面积（像素）
MIN_SIZE = 30       # 物块最小边长（像素，按最长边）
MAX_SIZE = 50       # 物块最大边长（像素，按最长边）
S_MAX, V_MIN, V_MAX = 127, 60, 167   # 灰色判定（HSV）


def detect_grid_lines(roi_img):
    """检测 ROI 内网格线，返回 (行线y列表, 列线x列表)。"""
    gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=80,
                            minLineLength=30, maxLineGap=10)
    rows_y, cols_x = [], []
    if lines is not None:
        lines = np.asarray(lines)
        if lines.ndim == 3:
            lines = lines.reshape(-1, 4)
        for x1, y1, x2, y2 in lines:
            dx, dy = x2 - x1, y2 - y1
            length = np.hypot(dx, dy)
            if length < 30:
                continue
            angle = np.degrees(np.arctan2(abs(dy), abs(dx)))
            if angle < 15:
                rows_y.append((y1 + y2) / 2)
            elif angle > 75:
                cols_x.append((x1 + x2) / 2)
    return rows_y, cols_x


def cluster_lines(values, tol=5):
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
    src, dst = [], []
    for i, y in enumerate(rows_y):
        for j, x in enumerate(cols_x):
            src.append((x, y))
            dst.append((j * CELL_CM, i * CELL_CM))
    return np.array(src, np.float32), np.array(dst, np.float32)


class ObjectLocalizer(Node):
    def __init__(self, cell_cm, gui, min_size, max_size):
        super().__init__('object_localizer')
        self.cell_cm = cell_cm
        self.gui = gui
        self.min_size = min_size
        self.max_size = max_size
        self.H = None
        self.roi = None

        # 发布物块物理坐标（网格纸坐标系）
        self.pub = self.create_publisher(PointStamped, '/object_pose', 10)

        # 相机初始化
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('无法打开相机 /dev/video0')
            raise SystemExit(1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # 定时器：10Hz 处理
        self.timer = self.create_timer(0.1, self.tick)
        # selectROI 在独立线程做，避免阻塞 executor
        threading.Thread(target=self._init_interactive, daemon=True).start()

    def _init_interactive(self):
        """交互初始化：框选 ROI + 自动标定（在独立线程，不阻塞 spin）。"""
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().error('读帧失败')
            return
        r = cv2.selectROI('set ROI', frame, showCrosshair=True,
                          fromCenter=False)
        rx, ry, rw, rh = r
        if rw <= 0 or rh <= 0:
            rx, ry, rw, rh = 0, 0, frame.shape[1], frame.shape[0]
        self.roi = (rx, ry, rw, rh)
        self.get_logger().info(f'ROI: x={rx} y={ry} w={rw} h={rh}')
        self._calibrate(frame)
        cv2.destroyWindow('set ROI')
        if self.gui:
            cv2.namedWindow('localizer')
            self.get_logger().info('初始化完成，开始发布 /object_pose')

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            return

        if self.H is None or self.roi is None:
            return  # 等待初始化线程完成

        rx, ry, rw, rh = self.roi
        roi_img = frame[ry:ry + rh, rx:rx + rw]

        # 灰色物块检测
        hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, V_MIN), (179, S_MAX, V_MAX))
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
            # 尺寸过滤：只保留边长在 [min_size, max_size] 的物块
            side = max(bw, bh)
            if not (self.min_size <= side <= self.max_size):
                continue
            if best is None or area > best[4]:
                best = (bx + bw // 2, by + bh // 2, bw, bh, area)

        if best:
            cx, cy, bw, bh, area = best
            gx, gy = rx + cx, ry + cy
            p = np.array([[[gx, gy]]], dtype=np.float64)
            phy = cv2.perspectiveTransform(p, self.H)
            Xcm, Ycm = phy[0][0]

            # 发布物块物理坐标（米，网格纸坐标系）
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'grid'
            msg.point.x = Xcm / 100.0
            msg.point.y = Ycm / 100.0
            msg.point.z = 0.0
            self.pub.publish(msg)

            if self.gui:
                display = frame.copy()
                cv2.rectangle(display, (rx, ry), (rx + rw, ry + rh),
                              (255, 0, 0), 2)
                cv2.rectangle(display, (gx - bw // 2, gy - bh // 2),
                              (gx + bw // 2, gy + bh // 2), (0, 255, 0), 3)
                cv2.putText(display, f'({Xcm:.1f}, {Ycm:.1f}) cm',
                            (gx + 10, gy - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 0), 2)
                cv2.putText(display, f'pub /object_pose 10Hz',
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (200, 200, 200), 1)
                cv2.imshow('localizer', display)
                cv2.imshow('mask', mask)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.get_logger().info('q pressed, quitting')
                    raise SystemExit(0)

    def _calibrate(self, frame):
        rx, ry, rw, rh = self.roi
        roi_img = frame[ry:ry + rh, rx:rx + rw]
        rows_y, cols_x = detect_grid_lines(roi_img)
        rows_y = cluster_lines(rows_y)
        cols_x = cluster_lines(cols_x)
        if len(rows_y) < 2 or len(cols_x) < 2:
            self.get_logger().warn(
                f'网格线检测失败（行{len(rows_y)} 列{len(cols_x)}）')
            return
        src, dst = build_mapping(rows_y, cols_x)
        H, _ = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
        if H is None:
            self.get_logger().warn('Homography 计算失败')
            return
        self.H = H
        self.get_logger().info(
            f'标定成功！行线 {len(rows_y)} 条、列线 {len(cols_x)} 条。'
            '发布 /object_pose (grid 坐标系)')


def main():
    rclpy.init()
    node = ObjectLocalizer(cell_cm=CELL_CM, gui=True,
                           min_size=MIN_SIZE, max_size=MAX_SIZE)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
