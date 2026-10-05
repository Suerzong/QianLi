#!/usr/bin/env python3
"""机械臂精确移动工具：把 TCP 移到指定位置（或相对位移）

用途：抓取时**分步下压**，每步可由人眼确认，避免盲撞物块。

用法：
  python3 arm_move.py --dz -0.01          # 当前高度下降 1cm
  python3 arm_move.py --z -0.031          # 绝对高度
  python3 arm_move.py --x 0.345 --y 0.06 --z 0.019   # 绝对位置
  python3 arm_move.py --gripper 1.2       # 张开夹爪（0.0=闭合）

选项：
  --enable   先调用 /arm/enable（串口重连后必需）
  --wait     移动后等待并打印最终 TCP 位置（秒，默认 6）
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

TCP_FRAME = 'gripper_frame_link'


class Mover(Node):
    def __init__(self):
        super().__init__('arm_move')
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.pub = self.create_publisher(PointStamped, '/arm/target_position',
                                         10)
        self.pub_grip = self.create_publisher(Float64, '/gripper_command', 10)
        self.enabled = None
        self.create_subscription(String, '/arm/status', self._on_status, 10)
        self.cli = self.create_client(SetBool, '/arm/enable')

    def _on_status(self, msg):
        self.enabled = '"enabled": true' in msg.data

    def tcp(self):
        for _ in range(50):
            try:
                tr = self.buf.lookup_transform('base_link', TCP_FRAME,
                                               rclpy.time.Time())
                t = tr.transform.translation
                return (t.x, t.y, t.z)
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
        return None

    def enable(self):
        if not self.cli.wait_for_service(timeout_sec=5.0):
            print('⚠️ /arm/enable 服务不可用')
            return False
        req = SetBool.Request()
        req.data = True
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10)
        time.sleep(1.5)
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.enabled:
                return True
        return bool(self.enabled)

    def move(self, x, y, z):
        m = PointStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.point.x, m.point.y, m.point.z = x, y, z
        self.pub.publish(m)

    def gripper(self, pos):
        m = Float64()
        m.data = pos
        self.pub_grip.publish(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float)
    ap.add_argument('--y', type=float)
    ap.add_argument('--z', type=float)
    ap.add_argument('--dx', type=float, default=0.0)
    ap.add_argument('--dy', type=float, default=0.0)
    ap.add_argument('--dz', type=float, default=0.0)
    ap.add_argument('--gripper', type=float)
    ap.add_argument('--enable', action='store_true')
    ap.add_argument('--wait', type=float, default=6.0)
    a = ap.parse_args()

    rclpy.init()
    n = Mover()
    time.sleep(1.5)   # 等 TF/status 就绪

    if a.enable:
        if n.enable():
            print('✅ 运动已使能')
        else:
            print('❌ 使能失败，中止')
            rclpy.shutdown()
            sys.exit(1)

    if a.gripper is not None:
        n.gripper(a.gripper)
        print(f'夹爪指令: {a.gripper}')
        time.sleep(2)
        rclpy.shutdown()
        return

    cur = n.tcp()
    if cur is None:
        print('❌ 读不到 TCP 位置')
        rclpy.shutdown()
        sys.exit(1)
    x = a.x if a.x is not None else cur[0] + a.dx
    y = a.y if a.y is not None else cur[1] + a.dy
    z = a.z if a.z is not None else cur[2] + a.dz
    print(f'当前 TCP = ({cur[0]:.4f}, {cur[1]:.4f}, {cur[2]:.4f})')
    print(f'目标 TCP = ({x:.4f}, {y:.4f}, {z:.4f})')

    n.move(x, y, z)
    # 持续发几帧，确保被接收
    t0 = time.time()
    while time.time() - t0 < a.wait:
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.time() - t0 < 1.0:
            n.move(x, y, z)
    fin = n.tcp()
    if fin:
        print(f'到达 TCP = ({fin[0]:.4f}, {fin[1]:.4f}, {fin[2]:.4f})')
        err = math.sqrt(sum((f - t) ** 2 for f, t in zip(fin, (x, y, z))))
        print(f'位置误差 = {err*1000:.1f} mm')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
