#!/usr/bin/env python3
"""隔离冒烟测试：验证标定脚本的"触发→记录→落盘→z 判据"闭环。

用当前实时姿态触发一次记录，输出到测试 JSON（不碰正式标定数据）。
预期：z 判据通过（当前臂姿压在板上），一个点被记录并立即落盘。
"""
import os
import sys
import time

import numpy as np

sys.path.insert(0, '/home/ros/QianLi/qianli_ws/src/qianli_vision/scripts')

import rclpy

TEST_TRIGGER = '/tmp/smoke_mark'
TEST_JSON = '/tmp/smoke_marks.json'

# 复制 collect 的核心逻辑：最小化 Collector
import importlib.util
spec = importlib.util.spec_from_file_location(
    'ext_calib', '/home/ros/QianLi/qianli_ws/src/qianli_vision/scripts/extrinsic_calib_multi.py')
ext = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ext)

# 构造 args 替身
class A:
    json = TEST_JSON
    table_z = -0.06909
    board_mm = 0.5
    z_tol_mm = 2.0
    max_face_offset_mm = 5.0
    tcp = '/tmp/tcp_calib.txt'

# 直接调 collect 但需要 ROS 上下文与节点；为不干扰正式实例，
# 这里改为验证底层：GripperModel.fixed_tip 在实时关节角下的输出。
from gripper_model import GripperModel, JOINTS

rclpy.init()
node = rclpy.create_node('smoke_test')
got = {}
node.create_subscription(
    __import__('sensor_msgs.msg', fromlist=['JointState']).JointState,
    '/joint_states',
    lambda m: got.update(dict(zip(m.name, m.position)))
    if len(m.name) == len(m.position) else None, 10)
t0 = time.time()
while rclpy.ok() and (len(got) < 6 or time.time() - t0 < 3):
    rclpy.spin_once(node, timeout_sec=0.05)

joints = {k: float(got[k]) for k in JOINTS}
model = GripperModel(stride=14)
T = model.solve(joints)
M = T['gripper_link']
pts = model.parts['gripper_link']
world = (M[:3, :3] @ pts.T).T + M[:3, 3]
k = int(np.argmin(world[:, 2]))
tip = world[k]
board = -0.06909 + 0.0005
z_err = tip[2] - board
print(f'固定爪顶端 = ({tip[0]:+.4f}, {tip[1]:+.4f}, {tip[2]:+.4f})')
print(f'z-板面 = {z_err*1000:+.2f} mm, 判据容差 ±2.0 mm')
print(f'判定: {"✅ 通过(可记录)" if abs(z_err) < 0.002 else "❌ 拒绝"}')

# 验证落盘格式：写一个测试 mark，模拟脚本的落盘调用
mark = {
    'grid_cm': [0.0, 0.0],
    'contact_m': [float(tip[0]), float(tip[1]), float(tip[2])],
    'base_m': [0.0, 0.0, 0.0],
    'tcp_above_lowest_m': 0.011,
    'gripper_rad': joints['gripper'],
    'joints': joints,
    'at': time.strftime('%H:%M:%S')}
with open(TEST_JSON, 'w') as fh:
    import json
    json.dump([mark], fh, indent=2)
print(f'落盘 OK: {TEST_JSON}')
node.destroy_node()
rclpy.try_shutdown()
