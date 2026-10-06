#!/usr/bin/env python3
"""零风险测试安全闸门：驱动处于只读模式，指令不会真动

做法：读当前关节角当基准，然后依次发出
  1) 当前姿态                —— 应放行
  2) 远离桌面（抬高）         —— 应放行
  3) 压向桌面（降低）         —— 应拦截
  4) 从被拦处继续压           —— 应继续拦截
并从 /safety_gate_state 读回统计。
"""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
          'wrist_roll', 'gripper']


def main():
    rclpy.init()
    node = Node('gate_test')
    state = {}
    gate = {}
    node.create_subscription(
        JointState, '/joint_states',
        lambda m: state.update(dict(zip(m.name, m.position)))
        if len(m.name) == len(m.position) else None, 10)
    node.create_subscription(
        String, '/safety_gate_state',
        lambda m: gate.update(json.loads(m.data)), 10)
    pub = node.create_publisher(JointState, '/joint_commands_raw', 10)

    t0 = time.time()
    while rclpy.ok() and (len(state) < 6 or not gate) and time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(state) < 6:
        print('❌ 读不到 /joint_states')
        return 1
    if not gate:
        print('❌ 读不到 /safety_gate_state —— 闸门没在跑？')
        return 1
    print(f'闸门状态: {gate}')
    base = [float(state[k]) for k in JOINTS]
    print(f'基准关节: {[round(v,3) for v in base]}')

    def send(tag, q):
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = list(JOINTS)
        msg.position = [float(v) for v in q]
        a0, r0 = gate.get('accepted', 0), gate.get('rejected', 0)
        for _ in range(6):
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.08)
        print(f'{tag:<26} 放行 {gate.get("accepted",0)-a0:>2}  '
              f'拦截 {gate.get("rejected",0)-r0:>2}   '
              f'| {gate.get("last_reason","")[:88]}')

    print()
    send('1) 当前姿态', base)

    up = list(base)
    up[1] = base[1] - 0.25          # shoulder_lift 减小 → 抬高
    send('2) 抬高 0.25rad（远离桌面）', up)

    down = list(base)
    down[1] = base[1] + 0.25        # 反向 → 压向桌面
    send('3) 降低 0.25rad（压向桌面）', down)

    down2 = list(base)
    down2[1] = base[1] + 0.45
    send('4) 再降 0.45rad', down2)

    print()
    print(f'最终统计: 放行 {gate.get("accepted")}  拦截 {gate.get("rejected")}')
    try:
        node.destroy_node()
    except Exception:  # noqa: BLE001
        pass
    if rclpy.ok():
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
