#!/usr/bin/env python3
"""Live check: 当前姿态下模型 FK 的 TCP vs TF 的 tcp_link，以及固定爪顶端。

回答用户核心问题：固定爪顶端能不能被模型稳定算出来、准不准。
"""
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.expanduser('~/QianLi/qianli_ws/src/qianli_vision/scripts'))
from gripper_model import GripperModel, JOINTS

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    node = Node('live_fixed_tip_check')
    got = {}
    node.create_subscription(
        JointState, '/joint_states',
        lambda m: got.update(dict(zip(m.name, m.position)))
        if len(m.name) == len(m.position) else None, 10)
    buf = Buffer()
    TransformListener(buf, node)
    t0 = time.time()
    while rclpy.ok() and (len(got) < 6 or time.time() - t0 < 3):
        rclpy.spin_once(node, timeout_sec=0.05)

    joints = {k: float(got[k]) for k in JOINTS}
    print('关节角:', ', '.join(f'{k}={joints[k]:+.3f}' for k in JOINTS))

    model = GripperModel(stride=14)
    T = model.solve(joints)
    # 模型 TCP
    tcp_model = T['tcp_link'][:3, 3]
    # TF TCP
    tcp_tf = None
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            tr = buf.lookup_transform('base_link', 'tcp_link',
                                      rclpy.time.Time())
            tcp_tf = np.array([tr.transform.translation.x,
                               tr.transform.translation.y,
                               tr.transform.translation.z])
            break
        except Exception:
            pass
    print(f'\nTCP 模型 FK : ({tcp_model[0]:+.4f}, {tcp_model[1]:+.4f}, '
          f'{tcp_model[2]:+.4f})')
    if tcp_tf is not None:
        d = np.linalg.norm(tcp_model - tcp_tf)
        print(f'TCP TF     : ({tcp_tf[0]:+.4f}, {tcp_tf[1]:+.4f}, '
              f'{tcp_tf[2]:+.4f})   差 {d*1000:.2f} mm  '
              f'{"✅ FK 与 TF 一致" if d < 0.003 else "⚠️ 不一致"}')

    # 固定爪顶端（模型）
    pts = model.parts['gripper_link']
    M = T['gripper_link']
    world = (M[:3, :3] @ pts.T).T + M[:3, 3]
    k = int(np.argmin(world[:, 2]))
    tip = world[k]
    board = -0.06909 + 0.0005
    print(f'\n固定爪顶端 : ({tip[0]:+.4f}, {tip[1]:+.4f}, {tip[2]:+.4f})')
    print(f'  板面 z = {board*1000:+.2f} mm，顶端 z-板面 = '
          f'{(tip[2]-board)*1000:+.2f} mm')
    print(f'  工具轴偏离竖直 ≈ {np.degrees(np.arccos(abs(T["tcp_link"][2,2]))):.1f}°')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
