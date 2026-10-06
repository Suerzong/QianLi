#!/usr/bin/env python3
"""回归测试：驱动**不允许**静默吞掉超限指令

修复前的行为：``_on_command`` 用 ``np.clip`` 把超限目标悄悄改成限位值，
不报错、不记日志、状态里也看不到 —— 表现为"机械臂莫名其妙停在那儿"。

本测试对 ``sim`` 模式的驱动跑一遍：
  1. 先发一条完全合法的指令 → clip_events 必须保持 0
  2. 再发一条明显超限的指令 → clip_events 必须增加，
     且 /arm/status 里的 clip_last 要给出被吞掉的关节与角度

只在 sim 模式运行，不碰硬件。用法::

    # 终端 A
    ros2 run so101_bringup driver_node --ros-args -p mode:=sim \
        --params-file ~/legacy/arm/arm-final/ros2_ws/install/so101_bringup/share/so101_bringup/config/driver_params.yaml
    # 终端 B
    ~/mj/bin/python verify_limit_reporting.py
"""

from __future__ import annotations

import json
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
          'wrist_roll', 'gripper']
REST = [0.0, -1.0, 0.5, -0.5, 0.0, 0.5]


class LimitReporterProbe(Node):

    def __init__(self):
        super().__init__('limit_reporting_probe')
        self.pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.create_subscription(String, '/arm/status', self._on_status, 10)
        self.status = None
        self.mode = None

    def _on_status(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(payload, dict):
            self.status = payload
            self.mode = payload.get('mode')

    def send(self, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINTS)
        msg.position = [float(v) for v in positions]
        self.pub.publish(msg)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    rclpy.init()
    node = LimitReporterProbe()
    failures = []

    try:
        node.spin_for(3.0)
        if node.status is None:
            print('❌ 收不到 /arm/status —— 驱动没在跑？')
            return 1
        if node.mode != 'sim':
            print(f'❌ 驱动模式是 {node.mode!r}，本测试只在 sim 模式下运行')
            return 1

        base_events = int(node.status.get('clip_events', -1))
        if base_events < 0:
            print('❌ /arm/status 里没有 clip_events 字段 —— 跑的还是旧驱动')
            return 1
        print(f'✅ 驱动在线（sim），初始 clip_events={base_events}')

        # 1) 合法指令不应产生任何 clip
        node.send(REST)
        node.spin_for(1.5)
        events = int(node.status.get('clip_events', -1))
        if events != base_events:
            failures.append(
                f'合法指令被误判为超限：clip_events {base_events} → {events} '
                f'({node.status.get("clip_last")})')
            print(f'❌ {failures[-1]}')
        else:
            print(f'✅ 合法指令未被裁剪（clip_events 仍为 {events}）')

        # 2) 超限指令必须被显式报告
        wild = [3.0, -1.0, 0.5, -0.5, 3.0, 0.5]     # pan/wrist_roll 远超限位
        node.send(wild)
        node.spin_for(2.0)
        events = int(node.status.get('clip_events', -1))
        if events <= base_events:
            failures.append('超限指令被静默吞掉：clip_events 没有增加')
            print(f'❌ {failures[-1]}')
        else:
            last = node.status.get('clip_last') or {}
            swallowed = abs(float(last.get('swallowed_deg', 0.0)))
            print(f'✅ 超限指令被显式报告：clip_events={events}，'
                  f'最近一次 {last.get("joint")} 被吞 {swallowed:.1f}°')
            if swallowed < 5.0:
                failures.append(
                    f'报告的吞掉角度过小（{swallowed:.1f}°），可能没抓到真正越界的关节')
            if not node.status.get('clip_seen'):
                failures.append('clip_seen 为空，无法定位是哪个关节被限')
            else:
                print(f'✅ 被限关节可定位：{node.status["clip_seen"]}')

        # 3) 恢复合法指令后，状态仍可读
        node.send(REST)
        node.spin_for(1.0)
        print(f'   最终 /arm/status.clip_events = {node.status.get("clip_events")}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()

    print()
    if failures:
        print('❌ 失败：')
        for item in failures:
            print(f'   · {item}')
        return 1
    print('✅ 全部通过：超限指令会被报告，不会被静默吞掉')
    return 0


if __name__ == '__main__':
    sys.exit(main())
