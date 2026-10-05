#!/usr/bin/env python3
"""测量机械臂末端抖动（用于控制参数对照实验）

输出：TCP x/y/z 的峰峰值与标准差，以及夹爪关节的抖动。
用法：python3 measure_jitter.py [秒数]
"""

import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0

rclpy.init()
n = Node('jitter_probe')
buf = Buffer()
TransformListener(buf, n)
js = {}
n.create_subscription(JointState, '/joint_states',
                      lambda m: js.update(pos=list(m.position)), 10)

xs, ys, zs, gs = [], [], [], []
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.02)
    try:
        tr = buf.lookup_transform('base_link', 'gripper_frame_link',
                                  rclpy.time.Time())
        t = tr.transform.translation
        xs.append(t.x)
        ys.append(t.y)
        zs.append(t.z)
        if js.get('pos'):
            gs.append(js['pos'][5])
    except Exception:
        pass
n.destroy_node()
rclpy.shutdown()


def stat(name, a):
    if len(a) < 5:
        print(f'  {name}: 采样不足')
        return
    pp = max(a) - min(a)
    print(f'  {name}: 峰峰值={pp:.4f} 标准差={statistics.pstdev(a):.4f} '
          f'均值={statistics.mean(a):+.4f}')


print(f'样本 {len(xs)} 个 / {DUR:.0f} 秒')
stat('TCP.x ', xs)
stat('TCP.y ', ys)
stat('TCP.z ', zs)
if gs:
    stat('夹爪  ', gs)
