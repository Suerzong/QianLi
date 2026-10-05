#!/usr/bin/env python3
"""第4步：抓取桥接节点 —— 物块坐标(grid) → base_link → 机械臂抓取

数据流：
  object_localizer 发布 /object_pose (grid 系, 米)
      ↓ 本节点订阅
  外参变换：grid → base_link（自动读取 /tmp/extrinsic.txt）
      ↓
  发布 /arm/target_position (base_link 系)
      ↓
  ik_node 解算关节角 → 机械臂移动到物块上方

外参来源（优先级）：
  1. ROS 参数 grid_origin_x / grid_origin_y / grid_origin_z / grid_theta_deg
  2. /tmp/extrinsic.txt（extrinsic_calib.py 两点标定产出）

参数：
  approach_z   末端在物块上方多高（米，默认 0.05）
  publish      false = 只打印预览（不发给机械臂），安全调试用
  mode         oneshot（默认，锁存第一个稳定目标后停止发送）
               / stream（持续跟随物块）
  stable_n     判定"稳定"所需的连续一致帧数（默认 3）
  extrinsic_file  外参文件路径（默认 /tmp/extrinsic.txt）

用法：
  # 预览（不动机械臂）
  ros2 run qianli_vision grab_bridge --ros-args -p publish:=false
  # 真发目标
  ros2 run qianli_vision grab_bridge --ros-args -p publish:=true -p approach_z:=0.05
"""

import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool


def load_extrinsic(path):
    """从外参文件读取 grid→base_link 变换。返回 dict 或 None。"""
    if not os.path.exists(path):
        return None
    vals = {}
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#') or '=' not in line:
                    continue
                k, v = line.split('=', 1)
                try:
                    vals[k.strip()] = float(v.strip())
                except ValueError:
                    pass
    except OSError:
        return None
    need = ('grid_origin_x', 'grid_origin_y', 'grid_origin_z',
            'grid_theta_deg')
    if not all(k in vals for k in need):
        return None
    return vals


class GrabBridge(Node):
    def __init__(self):
        super().__init__('grab_bridge')

        self.declare_parameter('approach_z', 0.05)
        self.declare_parameter('publish', False)
        self.declare_parameter('mode', 'oneshot')
        self.declare_parameter('stable_n', 3)
        self.declare_parameter('auto_enable', False)  # 默认预览，不自动使能
        self.declare_parameter('extrinsic_file', '/tmp/extrinsic.txt')
        # 允许命令行直接覆盖外参
        self.declare_parameter('grid_origin_x', float('nan'))
        self.declare_parameter('grid_origin_y', float('nan'))
        self.declare_parameter('grid_origin_z', float('nan'))
        self.declare_parameter('grid_theta_deg', float('nan'))

        self.ext = self._resolve_extrinsic()
        if self.ext is None:
            self.get_logger().error(
                '缺少外参！请先运行 extrinsic_calib.py 完成两点标定，'
                '或用 -p grid_origin_x/y/z/... 传入')
        else:
            self.get_logger().info(
                '外参: origin=(%.4f, %.4f, %.4f)m θ=%.2f° (来源: %s)'
                % (self.ext['grid_origin_x'], self.ext['grid_origin_y'],
                   self.ext['grid_origin_z'], self.ext['grid_theta_deg'],
                   self.ext.get('source', 'file')))

        self.sub = self.create_subscription(
            PointStamped, '/object_pose', self.on_object_pose, 10)
        self.pub = self.create_publisher(
            PointStamped, '/arm/target_position', 10)
        # 自动使能：串口重连后 driver 会禁用运动，这里自动补一次 /arm/enable
        self.enabled = False
        self.enable_sent = False
        if self.get_parameter('auto_enable').value:
            self.cli = self.create_client(SetBool, '/arm/enable')
            self.create_subscription(String, '/arm/status', self.on_arm_status,
                                     10)
        self.history = []       # 最近几帧 grid 坐标（稳定性判断）
        self.latched = False    # oneshot 模式下是否已锁存
        self.get_logger().info(
            '抓取桥接就绪：/object_pose (grid) → /arm/target_position '
            '(base_link)；publish=%s mode=%s auto_enable=%s'
            % (self.get_parameter('publish').value,
               self.get_parameter('mode').value,
               self.get_parameter('auto_enable').value))

    def on_arm_status(self, msg: String):
        """跟踪 /arm/status 的 enabled 字段。"""
        try:
            import json
            self.enabled = bool(json.loads(msg.data).get('enabled', False))
        except Exception:
            self.enabled = '"enabled": true' in msg.data

    def _ensure_enabled(self):
        """若未使能则请求使能。返回 True 表示当前已使能。"""
        if self.enabled:
            return True
        if self.enable_sent:
            return False        # 已请求过，等待生效
        if not self.cli.service_is_ready():
            self.get_logger().warn('/arm/enable 服务未就绪，等待…')
            return False
        req = SetBool.Request()
        req.data = True
        self.cli.call_async(req)
        self.enable_sent = True
        self.get_logger().info('已请求 /arm/enable 使能运动')
        return False

    def _resolve_extrinsic(self):
        """参数优先，其次外参文件。"""
        px = self.get_parameter('grid_origin_x').value
        if not math.isnan(px):
            return {'grid_origin_x': px,
                    'grid_origin_y': self.get_parameter(
                        'grid_origin_y').value,
                    'grid_origin_z': self.get_parameter(
                        'grid_origin_z').value,
                    'grid_theta_deg': self.get_parameter(
                        'grid_theta_deg').value,
                    'source': 'params'}
        d = load_extrinsic(self.get_parameter('extrinsic_file').value)
        if d:
            d['source'] = self.get_parameter('extrinsic_file').value
        return d

    def on_object_pose(self, msg: PointStamped):
        if self.ext is None:
            return
        mode = self.get_parameter('mode').value
        if mode == 'oneshot' and self.latched:
            return           # 已锁存目标，忽略后续（抓取途中物块被挡也不影响）

        # 自动使能：未使能时先请求，等生效后再发目标
        if self.get_parameter('auto_enable').value and not self._ensure_enabled():
            return

        gx, gy = msg.point.x, msg.point.y   # 米（grid 系）

        # 稳定性判断：连续 stable_n 帧位置一致（<5mm）才认为可信
        n = int(self.get_parameter('stable_n').value)
        self.history.append((gx, gy))
        if len(self.history) > n:
            self.history.pop(0)
        if n > 1 and len(self.history) >= n:
            xs = [p[0] for p in self.history]
            ys = [p[1] for p in self.history]
            if (max(xs) - min(xs) > 0.005) or (max(ys) - min(ys) > 0.005):
                return       # 抖动中，等稳定

        # 绕 z 旋转 θ，再平移
        theta = math.radians(self.ext['grid_theta_deg'])
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        bx = cos_t * gx - sin_t * gy + self.ext['grid_origin_x']
        by = sin_t * gx + cos_t * gy + self.ext['grid_origin_y']
        bz = self.ext['grid_origin_z'] + self.get_parameter('approach_z').value

        # 落盘便于 SSH 读取
        try:
            with open('/tmp/grab_target.txt', 'w') as f:
                f.write(f'grid=({gx*100:.2f},{gy*100:.2f})cm\n'
                        f'base=({bx:.4f},{by:.4f},{bz:.4f})m\n'
                        f'mode={mode}\n')
        except OSError:
            pass

        if self.get_parameter('publish').value:
            out = PointStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = 'base_link'
            out.point.x = bx
            out.point.y = by
            out.point.z = bz
            self.pub.publish(out)
            self.get_logger().info(
                '物块 grid=(%.1f,%.1f)cm → base_link=(%.3f,%.3f,%.3f)m 已发送%s'
                % (gx * 100, gy * 100, bx, by, bz,
                   '（锁存，后续不再发送）' if mode == 'oneshot' else ''))
            if mode == 'oneshot':
                self.latched = True
        else:
            self.get_logger().info(
                '[预览] base_link=(%.3f,%.3f,%.3f)m（未发送）'
                % (bx, by, bz))
            if mode == 'oneshot':
                self.latched = True


def main():
    rclpy.init()
    node = GrabBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
