#!/usr/bin/env python3
"""发布标定可视化 Marker：棋盘(按外参) + 打点 + 当前固定爪顶端。

RViz 里加载后即可对照真实世界检查"建模是否符合物理"。
发布到 /calib_markers (MarkerArray)：
  - 棋盘薄板(半透明) + 内角点(黑球) + 原点(橙球)
  - 已打点(红球, 0.8cm)
  - 当前固定爪顶端(绿球, 1cm)
"""
import json
import math
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.expanduser(
    '~/QianLi/qianli_ws/src/qianli_vision/scripts'))
from gripper_model import GripperModel, JOINTS

CELL_M = 0.033
BOARD_COLS, BOARD_ROWS = 7, 5
BOARD_W = (BOARD_COLS - 1) * CELL_M
BOARD_H = (BOARD_ROWS - 1) * CELL_M
TABLE_Z = -0.06909

# 当前标定结果（merged marks 求解）
CALIB = {'origin': (0.2246, 0.0218), 'yaw_deg': -83.499}
MARKS_PATH = '/tmp/extrinsic_marks_merged.json'


class CalibMarkers(Node):
    def __init__(self):
        super().__init__('calib_markers')
        self.pub = self.create_publisher(MarkerArray, '/calib_markers', 10)
        self.model = GripperModel(stride=14)
        self.joints = {}

        # 订阅关节角（回调存储，供爪尖 marker 用）
        from sensor_msgs.msg import JointState

        def on_js(msg):
            if len(msg.name) == len(msg.position):
                self.joints = dict(zip(msg.name, msg.position))

        self.create_subscription(JointState, '/joint_states', on_js, 10)
        self.timer = self.create_timer(0.2, self.publish)
        self._last = None

    def publish(self):
        marks = []
        if os.path.exists(MARKS_PATH):
            marks = json.load(open(MARKS_PATH))
        arr = self.build(marks)
        # 每 0.2s 发布一次，爪尖绿球实时跟随拖动
        now = self.get_clock().now().nanoseconds
        if self._last is None or now - self._last > 0.2e9:
            self._last = now
            self.pub.publish(arr)

    def build(self, marks):
        arr = MarkerArray()
        origin, yaw_deg = CALIB['origin'], CALIB['yaw_deg']
        yaw = math.radians(yaw_deg)
        cy, sy = math.cos(yaw), math.sin(yaw)

        def frame(gx_m, gy_m, z=TABLE_Z + 0.001):
            return (origin[0] + cy * gx_m - sy * gy_m,
                    origin[1] + sy * gx_m + cy * gy_m, z)

        # 棋盘薄板（半透明，稍大）
        m = Marker()
        m.header.frame_id = 'base_link'
        m.ns = 'calib'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        cx, cyy, _ = frame(BOARD_W / 2, BOARD_H / 2)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = \
            cx, cyy, TABLE_Z + 0.0003
        m.pose.orientation.w = math.cos(yaw / 2)
        m.pose.orientation.z = math.sin(yaw / 2)
        m.scale.x, m.scale.y, m.scale.z = BOARD_W, BOARD_H, 0.0006
        m.color.a, m.color.r, m.color.g, m.color.b = 0.5, 0.85, 0.85, 0.8
        arr.markers.append(m)

        # 内角点
        for r in range(BOARD_ROWS):
            for c in range(BOARD_COLS):
                px, py, _ = frame(c * CELL_M, r * CELL_M)
                mk = Marker()
                mk.header.frame_id = 'base_link'
                mk.ns = 'calib'
                mk.id = 100 + r * 10 + c
                mk.type = Marker.SPHERE
                mk.action = Marker.ADD
                mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = \
                    px, py, TABLE_Z + 0.001
                mk.scale.x = mk.scale.y = mk.scale.z = 0.003
                is_origin = (r == 0 and c == 0)
                mk.color.a = 1.0
                mk.color.r, mk.color.g, mk.color.b = \
                    (1.0, 0.5, 0.0) if is_origin else (0.1, 0.1, 0.1)
                arr.markers.append(mk)

        # 打点（红球）
        for i, p in enumerate(marks):
            c = p['contact_m']
            mk = Marker()
            mk.header.frame_id = 'base_link'
            mk.ns = 'calib'
            mk.id = 200 + i
            mk.type = Marker.SPHERE
            mk.action = Marker.ADD
            mk.pose.position.x = c[0]
            mk.pose.position.y = c[1]
            mk.pose.position.z = TABLE_Z + 0.002
            mk.scale.x = mk.scale.y = mk.scale.z = 0.008
            mk.color.a, mk.color.r, mk.color.g, mk.color.b = 1.0, 1.0, 0.1, 0.1
            arr.markers.append(mk)

        # 当前固定爪顶端（绿球，从 self.joints 读）
        try:
            if self.joints:
                js = {k: float(self.joints[k]) for k in JOINTS}
                T = self.model.solve(js)
                M = T['gripper_link']
                pts = self.model.parts['gripper_link']
                world = (M[:3, :3] @ pts.T).T + M[:3, 3]
                k = int(np.argmin(world[:, 2]))
                tip = world[k]
                mk = Marker()
                mk.header.frame_id = 'base_link'
                mk.ns = 'calib'
                mk.id = 300
                mk.type = Marker.SPHERE
                mk.action = Marker.ADD
                mk.pose.position.x = tip[0]
                mk.pose.position.y = tip[1]
                mk.pose.position.z = TABLE_Z + 0.002
                mk.scale.x = mk.scale.y = mk.scale.z = 0.012
                mk.color.a, mk.color.r, mk.color.g, mk.color.b = \
                    1.0, 0.1, 1.0, 0.1
                arr.markers.append(mk)
        except Exception as e:
            self.get_logger().warn(f'当前爪尖 marker 失败: {e}')
        return arr


def main():
    rclpy.init()
    node = CalibMarkers()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
