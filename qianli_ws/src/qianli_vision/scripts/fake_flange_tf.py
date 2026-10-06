#!/usr/bin/env python3
"""离线自检用：发布合成的 base_link → gripper_link TF，并按序触发作标记。

用途
----
`tcp_calibrate.py` / `extrinsic_calib_multi.py` 的数学部分已经用合成数据自检过，
但**TF 订阅、四元数转换、触发文件、打点落盘**这条 ROS 链路还没验证过。
真机标定需要人扶着机械臂摆十几分钟，不该在那时候才发现接线错误。

本节点灌入一组已知姿态，逐个稳定后写出触发文件，让真实的标定脚本去采集，
最后把标定结果与注入的真值对比 —— 全链路端到端验证。

用法::

    ~/mj/bin/python fake_flange_tf.py --poses /tmp/fake_poses.json \
        --trigger /tmp/tcp_mark --settle 1.5
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def R_to_quat(R):
    """旋转矩阵 → (x, y, z, w)。"""
    tr = float(np.trace(R))
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


class FakeFlange(Node):

    def __init__(self, poses, trigger, settle, repeat):
        super().__init__('fake_flange_tf')
        self.poses = poses
        self.trigger = trigger
        self.settle = settle
        self.repeat = repeat
        self.broadcaster = TransformBroadcaster(self)
        self.create_timer(0.02, self._tick)
        self.index = 0
        self.since = time.monotonic()
        self.triggered = False
        self.done = False
        self.get_logger().info(
            f'发布 {len(poses)} 个合成姿态 -> {trigger}')

    def _tick(self):
        if self.done:
            return
        if self.index >= len(self.poses):
            if not self.repeat:
                self.done = True
                return
            self.index = 0
        pose = self.poses[self.index]
        R = np.array(pose['R'], dtype=float)
        p = np.array(pose['p'], dtype=float)
        x, y, z, w = R_to_quat(R)
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.child_frame_id = 'gripper_link'
        msg.transform.translation.x = float(p[0])
        msg.transform.translation.y = float(p[1])
        msg.transform.translation.z = float(p[2])
        msg.transform.rotation.x = x
        msg.transform.rotation.y = y
        msg.transform.rotation.z = z
        msg.transform.rotation.w = w
        self.broadcaster.sendTransform(msg)

        now = time.monotonic()
        if self.triggered:
            if not os.path.exists(self.trigger):
                # 采集端已消费，进入下一个姿态
                self.triggered = False
                self.since = now
                self.index += 1
            return
        if now - self.since >= self.settle:
            with open(self.trigger, 'w') as fh:
                fh.write('1')
            self.triggered = True
            self.get_logger().info(
                f'  → 触发第 {self.index + 1}/{len(self.poses)} 个姿态 '
                f'p={np.round(p, 4).tolist()}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--poses', required=True)
    ap.add_argument('--trigger', required=True)
    ap.add_argument('--settle', type=float, default=1.5)
    ap.add_argument('--repeat', action='store_true')
    args = ap.parse_args()

    with open(args.poses) as fh:
        payload = json.load(fh)
    # 兼容两种格式：裸的姿态列表，或 {"truth": ..., "poses": [...]}
    if isinstance(payload, dict):
        poses = payload.get('poses')
        if poses is None:
            raise SystemExit(
                f'{args.poses}: 缺少 "poses" 字段（收到键 {list(payload)}）')
    else:
        poses = payload
    if not poses:
        raise SystemExit(f'{args.poses}: poses 为空')

    if os.path.exists(args.trigger):
        os.remove(args.trigger)

    rclpy.init()
    node = FakeFlange(poses, args.trigger, args.settle, args.repeat)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    try:
        node.destroy_node()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    if rclpy.ok():
        rclpy.try_shutdown()
    print('FAKE_TF_DONE')
    return 0


if __name__ == '__main__':
    sys.exit(main())
