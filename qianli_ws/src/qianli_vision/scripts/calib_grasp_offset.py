#!/usr/bin/env python3
"""抓取偏移标定：把"视觉检测位置"和"夹爪实际该在哪"对齐

为什么要这一步：
  视觉给出物块中心 (x,y)，但 gripper_frame_link 这个 TCP 参考点
  未必正好在两片爪子的夹取中心 —— 直接拿 TCP 去对物块会夹空。
  与其去解析复杂的 URDF 坐标系嵌套，不如：
    物块在某处（视觉测得 x_o,y_o）→ 你亲手把夹爪摆到完美位置
    → 记录此时 TCP (x_t,y_t,z_t)
    → 偏移 = TCP - 物块检测位置
  之后抓取时用：目标 = 物块检测位置 + 偏移，高度 = z_t

流程：
  1. 需要 object_localizer 正在运行（提供 /object_pose）
  2. 脚本先记录当前物块检测位置（要求稳定）
  3. 松开扭矩 → 你用手把夹爪摆到正好夹住物块的位置
  4. 外部 touch /tmp/mark_grasp → 计算并保存 /tmp/grasp_offset.txt

产出 /tmp/grasp_offset.txt：
  off_x, off_y      水平偏移（米）—— 加到物块检测位置上
  grasp_z           抓取高度（米，TCP 绝对高度）
  board_z           桌面高度（推算 = grasp_z - 物块半高）
  safe_z_min        安全下限
"""

from project_paths import calibration_path

import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

TCP = 'gripper_frame_link'
TRIGGER = '/tmp/mark_grasp'
OUT = calibration_path('grasp_offset.txt')
EXT = calibration_path('extrinsic.txt')


def read_kv(path):
    d = {}
    try:
        for line in open(path):
            line = line.strip()
            if '=' in line and not line.startswith('#'):
                k, v = line.split('=', 1)
                try:
                    d[k.strip()] = float(v.strip())
                except ValueError:
                    pass
    except OSError:
        pass
    return d


class CalibGrasp(Node):
    def __init__(self, obj_h):
        super().__init__('calib_grasp_offset')
        self.obj_h = obj_h
        self.ext = read_kv(EXT)
        if not self.ext:
            raise SystemExit('缺少外参 /tmp/extrinsic.txt')
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.obj_hist = []
        self.cur = None
        self.enabled = False
        self.create_subscription(PointStamped, '/object_pose',
                                 self._on_obj, 10)
        self.create_subscription(String, '/arm/status', self._on_status, 10)
        self.cli = self.create_client(SetBool, '/arm/enable')
        self.create_timer(0.2, self._tick)

    def _on_obj(self, msg):
        self.obj_hist.append((msg.point.x, msg.point.y))
        if len(self.obj_hist) > 10:
            self.obj_hist.pop(0)

    def _on_status(self, msg):
        self.enabled = '"enabled": true' in msg.data

    def _tick(self):
        try:
            tr = self.buf.lookup_transform('base_link', TCP,
                                           rclpy.time.Time())
            t = tr.transform.translation
            self.cur = (t.x, t.y, t.z)
            with open('/tmp/tcp_pose.txt', 'w') as f:
                f.write(f'frame={TCP}\nx={t.x:.4f}\ny={t.y:.4f}\n'
                        f'z={t.z:.4f}\n')
        except Exception:
            pass
        if os.path.exists(TRIGGER):
            try:
                os.remove(TRIGGER)
            except OSError:
                pass
            self._save()

    def obj_base(self):
        """取稳定的物块检测位置，转换为 base_link 坐标。"""
        if len(self.obj_hist) < 5:
            return None
        xs = [p[0] for p in self.obj_hist[-5:]]
        ys = [p[1] for p in self.obj_hist[-5:]]
        if max(xs) - min(xs) > 0.005 or max(ys) - min(ys) > 0.005:
            return None
        gx, gy = sum(xs) / 5, sum(ys) / 5
        th = math.radians(self.ext['grid_theta_deg'])
        c, s = math.cos(th), math.sin(th)
        return (c * gx - s * gy + self.ext['grid_origin_x'],
                s * gx + c * gy + self.ext['grid_origin_y'])

    def release(self):
        if not self.cli.wait_for_service(timeout_sec=5.0):
            return False
        req = SetBool.Request()
        req.data = False
        self.cli.call_async(req)
        time.sleep(1.5)
        return True

    def _save(self):
        if self.cur is None:
            self.get_logger().warn('读不到 TCP，无法记录')
            return
        # 用记录时刻的物块位置
        o = self.obj_base()
        tx, ty, tz = self.cur
        if o is None:
            self.get_logger().warn('物块检测不稳定，仍按当前检测值计算')
            o = (float('nan'), float('nan'))
        off_x = tx - o[0] if not math.isnan(o[0]) else float('nan')
        off_y = ty - o[1] if not math.isnan(o[1]) else float('nan')
        board_z = tz - self.obj_h / 2.0
        lines = [
            '# 抓取偏移标定（视觉位置 → 夹爪实际位置）',
            f'# 记录时物块检测位置 (base_link): x={o[0]:.4f} y={o[1]:.4f}',
            f'# 记录时 TCP: x={tx:.4f} y={ty:.4f} z={tz:.4f}',
            f'off_x={off_x:.4f}',
            f'off_y={off_y:.4f}',
            f'grasp_z={tz:.4f}',
            f'board_z={board_z:.4f}',
            f'safe_z_min={board_z + 0.003:.4f}',
        ]
        with open(OUT, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        self.get_logger().info(
            '📍 标定完成：偏移=(%.4f, %.4f)m，抓取高度 z=%.4f，'
            '桌面 z≈%.4f' % (off_x, off_y, tz, board_z))


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--obj-h', type=float, default=0.02, help='物块高度（米）')
    ap.add_argument('--no-release', action='store_true', help='不松扭矩')
    a = ap.parse_args()

    rclpy.init()
    node = CalibGrasp(a.obj_h)
    time.sleep(2)
    o = node.obj_base()
    if o is None:
        print('正在等待稳定的物块检测…（确保 object_localizer 在跑、物块在棋盘上）')
        t0 = time.time()
        while time.time() - t0 < 20:
            rclpy.spin_once(node, timeout_sec=0.1)
            o = node.obj_base()
            if o:
                break
    if o is None:
        print('❌ 20 秒内没有稳定的物块检测，中止')
        node.destroy_node()
        rclpy.shutdown()
        return
    print(f'✅ 物块检测位置 (base_link) = ({o[0]:.4f}, {o[1]:.4f})')
    print(f'   当前 TCP = ({node.cur[0]:.4f}, {node.cur[1]:.4f}, '
          f'{node.cur[2]:.4f})')
    if not a.no_release:
        node.release()
        print('🔓 已松开扭矩 —— 请用手把夹爪摆到正好夹住物块的位置')
    print(f'摆好后执行: touch {TRIGGER}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
