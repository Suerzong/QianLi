#!/usr/bin/env python3
"""连续采样 /joint_states，看用户拖动时数值是否变化（验证 driver 上报）。"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

rclpy.init()
n = rclpy.create_node('js_monitor')
samples = []


def cb(msg):
    if len(msg.name) == len(msg.position):
        samples.append((msg.header.stamp.sec, dict(zip(msg.name, msg.position))))

n.create_subscription(JointState, '/joint_states', cb, 10)

import time
t0 = time.time()
while time.time() - t0 < 15 and len(samples) < 30:
    rclpy.spin_once(n, timeout_sec=0.05)

n.destroy_node()
rclpy.try_shutdown()

print(f'采样 {len(samples)} 条 (15s)')
if len(samples) >= 2:
    first = samples[0][1]
    changed = 0
    for i in range(1, len(samples)):
        for k in first:
            if abs(samples[i][1][k] - first[k]) > 0.001:
                changed += 1
                break
    print(f'相对首条发生变化的条数: {changed}/{len(samples)-1}')
    # 打印首末对比
    last = samples[-1][1]
    print('关节        首条      末条      差')
    for k in first:
        print(f'{k:>22} {first[k]:+.4f} {last[k]:+.4f} '
              f'{last[k]-first[k]:+.4f}')
