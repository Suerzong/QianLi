#!/usr/bin/env python3
"""qianli_teach 拖动示教节点（record / playback）

闭环：拖 → 记 → 放
  record   : 订阅 /joint_states（真实编码器反馈），键盘触发开始/停止，保存 YAML 轨迹
  playback : 读取 YAML 轨迹，按原时间间隔发布 /joint_commands（由 driver 执行）

用法（先 source 环境）：
  ros2 run qianli_teach teach_node --mode record --file ~/traj1.yaml
  #   按 Enter 开始录制 → 手动拖动机械臂 → 再按 Enter 停止并保存
  ros2 run qianli_teach teach_node --mode playback --file ~/traj1.yaml --speed 1.0
  #   回放速度倍率：1.0 原速，0.5 半速，2.0 两倍速

安全说明：
  - 录制：纯读，零风险（扭矩关闭时可自由拖动机械臂）
  - 回放：发布 /joint_commands，是否真实执行由 driver 的 allow_motion 决定；
    在 sim 模式（allow_motion=false）下回放只在 RViz 中演示，不驱动真实舵机。
"""

import argparse
import threading
import time

import yaml
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import rclpy
from sensor_msgs.msg import JointState


def make_qos():
    # 状态类话题用 BestEffort 对不上就用默认；这里用可靠保证不丢帧
    return QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


class TeachNode(Node):
    def __init__(self, mode: str, file_path: str, speed: float):
        super().__init__('qianli_teach')
        self.mode = mode
        self.file_path = file_path
        self.speed = max(0.01, speed)

        # ---- 录制状态 ----
        self.recording = False
        self.traj = []           # [(dt, positions), ...]
        self.last_stamp = None
        self.joint_names = []
        self.lock = threading.Lock()

        # ---- 回放状态 ----
        self.play_index = 0
        self.play_start_time = None
        self.play_prev_dt = 0.0

        if mode == 'record':
            self.sub = self.create_subscription(
                JointState, '/joint_states', self._on_joint_states,
                make_qos(), raw=False)
            self.get_logger().info(
                'RECORD 模式就绪。按 Enter 开始录制，再次按 Enter 停止并保存。')
            self._keyboard_loop()
        elif mode == 'playback':
            self._load_trajectory()
            self.pub = self.create_publisher(
                JointState, '/joint_commands', make_qos())
            self.timer = self.create_timer(0.01, self._playback_tick)
            self.get_logger().info(
                f'PLAYBACK 模式就绪：{self.file_path}，速度 {self.speed}x，共 '
                f'{len(self.traj)} 帧。按 Enter 开始回放。')
            self._keyboard_loop()
        else:
            raise ValueError(f'未知 mode: {mode}')

    # ---------- 录制 ----------

    def _on_joint_states(self, msg: JointState):
        if not self.recording:
            return
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is None:
            # 第一帧只作起点，不记录（避免起点被异常时间戳污染）
            self.last_stamp = now
            return
        dt = now - self.last_stamp
        self.last_stamp = now
        # 防御：消息源切换 / 时钟跳变时 dt 会异常大，跳过该帧并重置起点
        if dt <= 0 or dt > 1.0:
            self.last_stamp = now
            return
        with self.lock:
            self.joint_names = list(msg.name)
            self.traj.append((dt, list(msg.position)))

    def _keyboard_loop(self):
        threading.Thread(target=self._kb, daemon=True).start()

    def _kb(self):
        while True:
            try:
                input()
            except EOFError:
                return
            if self.mode == 'record':
                if not self.recording:
                    with self.lock:
                        self.recording = True
                        self.traj = []
                        self.last_stamp = None
                    self.get_logger().info('▶ 开始录制，拖动机械臂……')
                else:
                    with self.lock:
                        self.recording = False
                    self._save()
                    self.get_logger().info('⏹ 已停止并保存。可再次按 Enter 重新录制。')
            else:  # playback
                self.play_start_time = time.monotonic()
                self.play_index = 0
                self.play_prev_dt = 0.0
                self.get_logger().info('▶ 开始回放……')

    def _save(self):
        with self.lock:
            traj = list(self.traj)
            names = list(self.joint_names)
        if not traj:
            self.get_logger().warn('没有录到任何帧（未检测到 /joint_states？）')
            return
        data = {
            'joint_names': names,
            'frames': [{'dt': round(dt, 4), 'positions': [round(p, 4) for p in pos]}
                       for dt, pos in traj],
        }
        with open(self.file_path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
        self.get_logger().info(
            f'💾 已保存 {len(traj)} 帧 → {self.file_path}（时长 '
            f'{sum(dt for dt, _ in traj):.2f}s）')

    # ---------- 回放 ----------

    def _load_trajectory(self):
        with open(self.file_path, encoding='utf-8') as f:
            data = yaml.safe_load(f)
        self.joint_names = data['joint_names']
        self.traj = [(fr['dt'], fr['positions']) for fr in data['frames']]
        self.get_logger().info(
            f'已加载 {len(self.traj)} 帧，关节：{self.joint_names}')

    def _playback_tick(self):
        if self.play_start_time is None or self.play_index >= len(self.traj):
            return
        # 按 0.01s tick 检查是否到了下一帧的时间
        now = time.monotonic() - self.play_start_time
        target_time = sum(dt for dt, _ in self.traj[:self.play_index]) / self.speed
        if now >= target_time:
            dt, positions = self.traj[self.play_index]
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = positions
            self.pub.publish(msg)
            self.play_index += 1
            if self.play_index >= len(self.traj):
                self.get_logger().info(
                    f'⏹ 回放完成（{len(self.traj)} 帧，速度 {self.speed}x）。'
                    f'再按 Enter 可重放。')


def main():
    rclpy.init()
    parser = argparse.ArgumentParser(description='QianLi 拖动示教')
    parser.add_argument('--mode', choices=['record', 'playback'], required=True)
    parser.add_argument('--file', required=True, help='轨迹 YAML 路径')
    parser.add_argument('--speed', type=float, default=1.0)
    args, unknown = parser.parse_known_args()
    node = TeachNode(args.mode, args.file, args.speed)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
