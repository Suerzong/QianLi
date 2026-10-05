#!/usr/bin/env python3
"""抓取位姿示教：松开扭矩，让用户手动把夹爪摆到完美夹住物块的位置，
然后打点记录该 TCP 位姿作为抓取基准。

原理：
  抓取高度（z）无法从视觉直接得到 —— 单目相机测不出物块/桌面高度。
  与其用尺子量再推算，不如让"人眼确认 + 机械臂自己报告坐标"：
  用户把夹爪摆到正确位置 → 读 TF 得到 base_link 下的 (x, y, z) → 这就是基准。

流程：
  1. 本脚本先关闭扭矩（/arm/enable data=false）→ 机械臂可手动搬动
  2. 用户手扶机械臂，把两片夹爪摆到物块两侧（正确夹取高度）
  3. 外部 `touch /tmp/mark_grasp` → 记录 TCP 到 /tmp/grasp_pose.txt
  4. 同时估算桌面高度 board_z = grasp_z - object_h/2（用于安全下限）

参数：
  object_h  物块高度（米，默认 0.02）
  release   是否自动松扭矩（默认 true）
"""

import math
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

TCP_FRAME = 'gripper_frame_link'
TRIGGER = '/tmp/mark_grasp'
OUT = '/tmp/grasp_pose.txt'


class TeachGrasp(Node):
    def __init__(self):
        super().__init__('teach_grasp')
        self.declare_parameter('object_h', 0.02)
        self.declare_parameter('release', True)
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.enabled = None
        self.create_subscription(String, '/arm/status', self._on_status, 10)

        if self.get_parameter('release').value:
            self._release_torque()

        self.create_timer(0.2, self.tick)

    def _on_status(self, msg):
        self.enabled = '"enabled": true' in msg.data

    def _release_torque(self):
        cli = self.create_client(SetBool, '/arm/enable')
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('/arm/enable 服务不可用')
            return
        req = SetBool.Request()
        req.data = False
        cli.call_async(req)
        time.sleep(1.5)
        self.get_logger().info('🔓 已松开扭矩 —— 机械臂现在可以手动搬动')

    def tick(self):
        try:
            tr = self.buf.lookup_transform('base_link', TCP_FRAME,
                                           rclpy.time.Time())
        except Exception:
            return
        t = tr.transform.translation
        x, y, z = t.x, t.y, t.z
        tmp = f'/tmp/tcp_pose.txt.tmp{os.getpid()}'
        try:
            with open(tmp, 'w') as f:
                f.write(f'frame={TCP_FRAME}\nx={x:.4f}\ny={y:.4f}\nz={z:.4f}\n')
            os.replace(tmp, '/tmp/tcp_pose.txt')
        except OSError:
            pass

        if os.path.exists(TRIGGER):
            try:
                os.remove(TRIGGER)
            except OSError:
                pass
            oh = float(self.get_parameter('object_h').value)
            board_z = z - oh / 2.0
            with open(OUT, 'w') as f:
                f.write(f'# 抓取位姿（拖动示教，人眼确认）\n'
                        f'grasp_x={x:.4f}\ngrasp_y={y:.4f}\n'
                        f'grasp_z={z:.4f}\n'
                        f'# 由物块高度 {oh*100:.1f}cm 估算的桌面高度\n'
                        f'board_z={board_z:.4f}\n'
                        f'safe_z_min={board_z + 0.003:.4f}\n')
            self.get_logger().info(
                '📍 抓取位姿已记录：grasp=(%.4f, %.4f, %.4f) m；'
                '估算 board_z=%.4f，安全下限 z>=%.4f'
                % (x, y, z, board_z, board_z + 0.003))


def main():
    rclpy.init()
    node = TeachGrasp()
    node.get_logger().info(
        f'示教中：把夹爪摆到物块两侧，然后外部执行 touch {TRIGGER}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
