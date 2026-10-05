#!/usr/bin/env python3
"""外参标定助手：实时记录夹爪 TCP 在 base_link 下的位置，并支持"打点"

原理：
  棋盘格坐标系(grid) → 机械臂坐标系(base_link) 的外参 =
     平移 (x0,y0,z0) + 绕 z 轴旋转 θ
  由两个点确定：
    点A：TCP 对准 grid (0,0)      → base_link (x0, y0, z0)
    点B：TCP 对准 grid (cell, 0)  → base_link (x1, y1, *)
    θ = atan2(y1-y0, x1-x0)
    z0 = 桌面高度（点A的 z）

用法（用户用手把机械臂搬到目标位置，SSH 侧触发打点）：
  1. 启动本脚本（持续把 TCP 位置写入 /tmp/tcp_pose.txt）
  2. 用户手扶机械臂对准 grid 原点 → 外部执行 `touch /tmp/mark_A`
     → 脚本把当前位姿存到 /tmp/marked_A.txt
  3. 用户对准 grid (cell,0) = 右边一格 → `touch /tmp/mark_B`
     → 存到 /tmp/marked_B.txt
  4. 脚本自动算出外参并打印 + 写入 /tmp/extrinsic.txt

参数：
  cell_cm  棋盘格边长（用于计算 θ 的参考距离，默认 3.3）
  frame    TCP 坐标系（默认 gripper_frame_link）
"""

import math
import os

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

TRIGGERS = {'/tmp/mark_A': '/tmp/marked_A.txt',
            '/tmp/mark_B': '/tmp/marked_B.txt'}


class ExtrinsicProbe(Node):
    def __init__(self):
        super().__init__('extrinsic_probe')
        self.declare_parameter('frame', 'gripper_frame_link')
        self.declare_parameter('cell_cm', 3.3)
        self.frame = self.get_parameter('frame').value
        self.cell = self.get_parameter('cell_cm').value

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.marks = {}
        self.create_timer(0.2, self.tick)

    def tick(self):
        try:
            tr = self.buf.lookup_transform('base_link', self.frame,
                                           rclpy.time.Time())
        except Exception:
            return
        t = tr.transform.translation
        x, y, z = t.x, t.y, t.z

        # 持续写入当前位姿
        pid = os.getpid()
        tmp = f'/tmp/tcp_pose.txt.tmp{pid}'
        try:
            with open(tmp, 'w') as f:
                f.write(f'frame={self.frame}\n'
                        f'x={x:.4f}\ny={y:.4f}\nz={z:.4f}\n')
            os.replace(tmp, '/tmp/tcp_pose.txt')
        except OSError:
            pass

        # 检查打点触发
        for trig, out in TRIGGERS.items():
            if os.path.exists(trig):
                try:
                    os.remove(trig)
                except OSError:
                    pass
                self.marks[out] = (x, y, z)
                with open(out, 'w') as f:
                    f.write(f'x={x:.4f}\ny={y:.4f}\nz={z:.4f}\n')
                self.get_logger().info(
                    f'📍 已打点 → {out}: base_link=({x:.4f}, {y:.4f}, '
                    f'{z:.4f}) m')
                self._try_solve()

    def _try_solve(self):
        """两点都有则算外参。"""
        a = self.marks.get('/tmp/marked_A.txt')
        b = self.marks.get('/tmp/marked_B.txt')
        if not a or not b:
            return
        x0, y0, z0 = a
        x1, y1, _ = b
        theta = math.degrees(math.atan2(y1 - y0, x1 - x0))
        # 实测两点距离 vs 理论格宽（校验）
        measured = math.hypot(x1 - x0, y1 - y0) * 100.0
        lines = [
            f'# 外参（grid → base_link）由两点标定',
            f'grid_origin_x={x0:.4f}',
            f'grid_origin_y={y0:.4f}',
            f'grid_origin_z={z0:.4f}',
            f'grid_theta_deg={theta:.2f}',
            f'# 校验：A→B 实测距离 {measured:.2f} cm '
            f'(理论 {self.cell} cm，误差 {measured - self.cell:+.2f} cm)',
        ]
        with open('/tmp/extrinsic.txt', 'w') as f:
            f.write('\n'.join(lines) + '\n')
        self.get_logger().info(
            '✅ 外参标定完成：origin=(%.4f, %.4f, %.4f) θ=%.2f°；'
            'A→B 实测 %.2fcm（理论 %.2fcm）'
            % (x0, y0, z0, theta, measured, self.cell))


def main():
    rclpy.init()
    node = ExtrinsicProbe()
    node.get_logger().info(
        '外参助手运行中：持续写 /tmp/tcp_pose.txt；'
        '打点用 touch /tmp/mark_A 或 /tmp/mark_B')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
