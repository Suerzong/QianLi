#!/usr/bin/env python3
"""行动前的物块位置快照（用户提议：不要用流式读数，动作前定一次）

为什么需要：
  视觉是流式的，且背景图可能是很久以前拍的。机械臂移动后会在棋盘上留下
  阴影/本体，背景差分就会把它误判成物块 —— 实测读数在
  (4.5,1.2) → (7.4,8.9) → (11.9,9.2) → (10.3,3.8) cm 之间乱跳。

本脚本流程（全部只动相机与机械臂的"停靠位"，不抓取）：
  1. 把机械臂停到"离开棋盘视野"的停靠位（避免它的阴影干扰）
  2. 重新采集背景图 → 重启视觉节点加载新背景
  3. 连续采样 /object_pose + /object_yaw，要求**连续 N 帧稳定**（抖动 < 3mm）
  4. 校验：在棋盘范围内、尺寸合理
  5. 换算成 base_link 坐标，写入 /tmp/obj_base.txt（抓取脚本直接读它）

用法：
  ~/mj/bin/python vision_snapshot.py            # 完整流程
  ~/mj/bin/python vision_snapshot.py --no-park  # 不挪机械臂（它已在停靠位）
  ~/mj/bin/python vision_snapshot.py --no-bg    # 不重采背景
"""

import argparse
import json
import math
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool
import tf2_ros

PARK = np.array([0.06, 0.00, 0.26])      # 停靠位：正上方抬高
# 实测（用"棋盘标定成功次数"当指标，8 秒内）：
#   (0.06, 0.00, 0.26) → 2 次 ✅ 不挡棋盘（最好）
#   (0.17, 0.03, 0.16) → 1 次 ✅
#   (0.10, -0.10, 0.22) → 0 次 ❌ 挡住棋盘（会导致标定失败、完全不发布）
YAW_DEG = -90.0
GRID_X_MAX, GRID_Y_MAX = 27.0, 20.0     # 网格原点是棋盘【左上角】，不是中心！
                                        # 棋盘 26x19.5cm → 坐标范围 [0,26]x[0,19.5]
STABLE_N = 8                             # 需要连续稳定的帧数
TOL_MM = 3.0


def quat(yaw_deg):
    y = math.radians(yaw_deg)
    return (0.0, math.cos(y / 2), 0.0, math.sin(y / 2))


def read_extrinsic():
    ext = {}
    with open('/tmp/extrinsic.txt') as fh:
        for line in fh:
            line = line.strip()
            if line.startswith('#') or '=' not in line:
                continue
            k, v = line.split('=', 1)
            try:
                ext[k.strip()] = float(v)
            except ValueError:
                pass
    return ext


class Snap(rclpy.node.Node):
    def __init__(self):
        super().__init__('vision_snapshot')
        self.pub = self.create_publisher(PoseStamped, '/ik_target', 10)
        self.cli = self.create_client(SetBool, '/arm/enable')
        self.pose = None
        self.yaw = None
        self.status = None
        self.create_subscription(PointStamped, '/object_pose', self._p, 10)
        self.create_subscription(Float64, '/object_yaw', self._y, 10)
        self.create_subscription(String, '/arm/status', self._s, 10)
        self.tfb = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfb, self)

    def _p(self, m):
        self.pose = (m.point.x, m.point.y)

    def _y(self, m):
        self.yaw = m.data

    def _s(self, m):
        self.status = m.data

    def enable(self):
        self.cli.wait_for_service(timeout_sec=5.0)
        f = self.cli.call_async(SetBool.Request(data=True))
        t0 = time.time()
        while not f.done() and time.time() - t0 < 5:
            rclpy.spin_once(self, timeout_sec=0.05)
        return f.result() is not None and f.result().success

    def park(self, target=PARK, sec=25):
        q = quat(YAW_DEG)
        t0 = time.time()
        while time.time() - t0 < sec:
            m = PoseStamped()
            m.header.frame_id = 'base_link'
            m.header.stamp = self.get_clock().now().to_msg()
            m.pose.position.x, m.pose.position.y, m.pose.position.z = \
                map(float, target)
            (m.pose.orientation.w, m.pose.orientation.x,
             m.pose.orientation.y, m.pose.orientation.z) = q
            self.pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.2)

    def tcp(self):
        try:
            t = self.tfb.lookup_transform('base_link', 'gripper_frame_link',
                                          rclpy.time.Time())
            p = t.transform.translation
            return np.array([p.x, p.y, p.z])
        except Exception:
            return None

    def collect(self, sec=6.0, hz=10.0):
        """连续采样，返回稳定的一段样本。"""
        buf = []
        t0 = time.time()
        while time.time() - t0 < sec:
            self.pose = None
            rclpy.spin_once(self, timeout_sec=1.0 / hz)
            if self.pose is not None:
                buf.append((self.pose[0], self.pose[1], self.yaw))
        return buf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--no-park', action='store_true')
    ap.add_argument('--no-bg', action='store_true')
    a = ap.parse_args()

    rclpy.init()
    n = Snap()

    if not a.no_park:
        print('1) 使能机械臂并停到"离开棋盘视野"的停靠位 ...')
        if not n.enable():
            print('   ⚠️ 使能失败，仍然继续（只影响背景质量）')
        n.park()
        p = n.tcp()
        if p is not None:
            print(f'   停靠位 TCP = ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})')
    else:
        print('1) 跳过挪臂（--no-park）')

    if not a.no_bg:
        print('2) 重新采集背景图（必须先停视觉节点，否则相机被占用）...')
        # 先停视觉节点释放相机
        subprocess.run(['bash', '-lc', 'pkill -f object_localizer; sleep 2'],
                       capture_output=True, text=True)
        try:
            r = subprocess.run(
                ['bash', '-lc',
                 'cd ~/QianLi/qianli_ws/src/qianli_vision/scripts; '
                 'timeout 40 ~/mj/bin/python capture_background.py'],
                capture_output=True, text=True, timeout=60)
            out = (r.stdout.strip().splitlines() or ['(无输出)'])[-1]
            print('   ' + out)
        except Exception as e:
            print(f'   ⚠️ 采背景失败: {e}')
        # 重启视觉节点加载新背景
        xa = subprocess.run(
            ['bash', '-lc', 'ls /run/user/1000/.mutter-Xwaylandauth.* | head -1'],
            capture_output=True, text=True).stdout.strip()
        subprocess.run(['bash', '-lc',
                        'pkill -f object_localizer; sleep 2; '
                        'source /opt/ros/jazzy/setup.bash 2>/dev/null; '
                        'source ~/QianLi/qianli_ws/install/setup.bash 2>/dev/null; '
                        f'DISPLAY=:0 XAUTHORITY={xa} nohup ros2 run '
                        'qianli_vision object_localizer > /tmp/loc.log 2>&1 &'],
                       capture_output=True, text=True)
        print('   视觉节点已用新背景重启，等 15 秒稳定 ...')
        time.sleep(15)
    else:
        print('2) 跳过重采背景（--no-bg）')

    print(f'3) 连续采样，要求 {STABLE_N} 帧内抖动 < {TOL_MM}mm ...')
    data = n.collect(sec=8.0)
    if len(data) < STABLE_N:
        print(f'   ❌ 只采到 {len(data)} 帧，视觉可能没在发布')
        rclpy.shutdown()
        return
    # 找最稳定的一段
    best = None
    for i in range(len(data) - STABLE_N + 1):
        win = np.array([(d[0], d[1]) for d in data[i:i + STABLE_N]])
        spread = float(np.max(np.linalg.norm(win - win.mean(axis=0), axis=1)))
        if best is None or spread < best[0]:
            best = (spread, i, win.mean(axis=0))
    spread, idx, mean = best
    yaws = [d[2] for d in data[idx:idx + STABLE_N] if d[2] is not None]
    yaw = float(np.median(yaws)) if yaws else 0.0
    print(f'   最稳定一段抖动 = {spread*1000:.2f}mm，位置 = '
          f'({mean[0]*100:.2f}, {mean[1]*100:.2f}) cm，yaw = {yaw:+.1f}°')
    if spread * 1000 > TOL_MM:
        print(f'   ❌ 抖动 {spread*1000:.1f}mm 超过 {TOL_MM}mm → 检测不可信，'
              f'请检查背景/光照/是否有杂物')
        rclpy.shutdown()
        return

    print('4) 校验并换算到 base_link ...')
    gx_cm, gy_cm = mean[0] * 100, mean[1] * 100
    if not (-1.0 <= gx_cm <= GRID_X_MAX and -1.0 <= gy_cm <= GRID_Y_MAX):
        print(f'   ⚠️ 位置 ({gx_cm:.1f},{gy_cm:.1f})cm 超出棋盘 0~26 x 0~19.5cm，可能检测错了')
    ext = read_extrinsic()
    th = math.radians(ext['grid_theta_deg'])
    c, s = math.cos(th), math.sin(th)
    bx = ext['grid_origin_x'] + c * mean[0] - s * mean[1]
    by = ext['grid_origin_y'] + s * mean[0] + c * mean[1]
    bz = -0.0394
    with open('/tmp/obj_base.txt', 'w') as fh:
        fh.write(f'{bx:.5f} {by:.5f} {bz:.5f}\n')
        fh.write(f'# grid=({mean[0]*100:.2f},{mean[1]*100:.2f})cm '
                 f'yaw={yaw:.1f}deg spread={spread*1000:.2f}mm\n')
    print(f'   ✅ 物块 base = ({bx:.4f}, {by:.4f}, {bz:.4f})  '
          f'离基座 {math.hypot(bx, by)*1000:.0f}mm')
    print('   已写入 /tmp/obj_base.txt')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
