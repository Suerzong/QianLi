#!/usr/bin/env python3
"""真机抓取（孪生验证过的参数）

孪生验证结论（20mm 物块）：
  · TCP 目标 = 物块中心 + (8, -4, 0) mm  ← 关键参数（固定，不随偏航变）
  · 偏航不敏感：物块转 0~60°、命令偏航 -90~+60° 组合全部成功
    （张开时两指呈 V 字，自动对中物块）
  · 接近角 0.6 → 降到目标 → 夹紧(0.0) → 等 2.5s(settle) → 抬起
  · 重复性 3/3，升高 +105mm

物块位置来源：视觉节点写的 /tmp/object_pose.txt（grid 系 cm + yaw）
外参：/tmp/extrinsic.txt（grid → base_link）

安全：
  · 目标 z 必须高于棋盘面，否则拒绝
  · 全程打印实际 TCP（TF base_link→gripper_frame_link）与目标偏差

用法：
  ~/mj/bin/python real_grasp_ok.py --dry-run     # 只算不动
  ~/mj/bin/python real_grasp_ok.py               # 执行
"""

import argparse
import json
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState
import tf2_ros
from grasp_guard import down_quat_xyzw, finite_position, require_fresh, read_observation

DX_MM, DY_MM, DZ_MM = 8.0, -4.0, 2.0   # dz=+2mm：孪生实测能抓起且爪尖在棋盘面上方 2.2mm
APPROACH_GRIP = 0.6
CLOSE_GRIP = 0.0

# ---- 桌面 / 棋盘 / 物块高度：2026-10-06 全部重新实测 ----
# 旧值 BOARD_Z=-0.0494（"桌面 -0.0524 + 板厚 3mm"）里的桌面 -0.0524 是从
# "最下端离桌约 5cm" 反推的；实测（夹爪最低点碰桌、6 点拟合，残差 RMS
# 0.469mm）是 **-0.06909**，差 16.7mm。棋盘纸实测厚 0.5mm 不是 3mm。
TABLE_Z = -0.06909
BOARD_THICK = 0.0005
BOARD_Z = TABLE_Z + BOARD_THICK      # -0.06859
# 物块换成 **4cm EVA 泡棉方块**（旧值 2cm 是上一轮的塑料小方块）
OBJ_SIZE = 0.04
OBJ_HALF = OBJ_SIZE / 2              # 0.02；旧代码里硬编码的 .010 就是 2cm 的半高
OBJ_Z = BOARD_Z + OBJ_HALF           # -0.04859（物块中心）

SAFE_MARGIN = 0.001
YAW_DEG = -90.0            # 与孪生一致（实测偏航不敏感）


def quat_from_RzRx(yaw_deg):
    """R = Rz(yaw) @ Rx(pi) 的四元数：q = qz(yaw) ⊗ qx(pi)。"""
    x,y,z,w = down_quat_xyzw(yaw_deg)
    return w,x,y,z


def read_vision():
    """读 /tmp/object_pose.txt（grid 系 cm）与 /tmp/extrinsic.txt。"""
    X,Y,pose = read_observation()
    ext = {}
    with open('/tmp/extrinsic.txt') as fh:
        for line in fh:
            line = line.strip()
            if line.startswith('#') or '=' not in line:
                continue
            k, v = line.split('=', 1)
            ext[k.strip()] = float(v)
    yaw_grid = float(pose.get('yaw_deg', 0))
    th = math.radians(ext['grid_theta_deg'])
    c, s = math.cos(th), math.sin(th)
    bx = ext['grid_origin_x'] + c * X - s * Y
    by = ext['grid_origin_y'] + s * X + c * Y
    return np.array(finite_position([bx, by, OBJ_Z])), yaw_grid


class Grasp(Node):
    def __init__(self, dry_run=False):
        super().__init__('real_grasp_ok')
        self.pub_pose = self.create_publisher(PoseStamped, '/ik_target', 10)
        self.pub_grip = self.create_publisher(Float64, '/gripper_command', 10)
        self.cli_enable = self.create_client(SetBool, '/arm/enable')
        self.tfb = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfb, self)
        # ★ 看门狗：driver 的 command_timeout=0.5s，只要 0.5 秒没收到指令就会
        #   自动失能（之前抓取到"等夹爪合拢"那步就失能了，因为那儿没发目标）。
        #   所以开一个后台线程，**持续以 10Hz 重发**最后的位姿与夹爪指令。
        self._last = None
        self._last_grip = None
        self._stop = False
        self._yaw = YAW_DEG
        # 串口掉线自动恢复：VM 的 USB 透传不稳，驱动重连后会保持"失能"。
        # 这里监视 /arm/status，一旦从 enabled 掉到 disabled 就自动重新使能。
        self._was_enabled = False
        self._reenable_count = 0
        self._auto = False
        self._fault = None
        self._dry_run = dry_run
        self.create_subscription(String, '/arm/status', self._on_status, 10)
        self._th = threading.Thread(target=self._heartbeat, daemon=True)
        if not dry_run:
            self._th.start()

    def _on_status(self, msg):
        try:
            en = bool(json.loads(msg.data).get('enabled', False))
        except Exception:
            return
        if en:
            self._was_enabled = True
        elif self._was_enabled:
            self._fault = 'motion disabled or driver disconnected; automatic re-enable prohibited'
            self._last = self._last_grip = None
        elif self._was_enabled and self._auto:
            self._was_enabled = False
            self._reenable_count += 1
            print(f'    ⚠️ 检测到失能（第 {self._reenable_count} 次，'
                  f'可能是串口掉线）→ 自动重新使能', flush=True)
            threading.Thread(target=self._auto_enable, daemon=True).start()

    def _auto_enable(self):
        time.sleep(1.0)          # 等驱动的重连完成
        for _ in range(5):
            try:
                if self.enable(True):
                    self._was_enabled = True
                    return
            except Exception:
                pass
            time.sleep(1.5)

    def _heartbeat(self):
        while not self._stop:
            if self._last is not None:
                self._publish(self._last, self._yaw)
            if self._last_grip is not None:
                self.pub_grip.publish(Float64(data=float(self._last_grip)))
            time.sleep(0.1)

    def _publish(self, xyz, yaw):
        q = quat_from_RzRx(yaw)
        m = PoseStamped()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, xyz)
        (m.pose.orientation.w, m.pose.orientation.x,
         m.pose.orientation.y, m.pose.orientation.z) = q
        self.pub_pose.publish(m)

    def tcp(self):
        try:
            t = self.tfb.lookup_transform('base_link', 'gripper_frame_link',
                                          rclpy.time.Time())
            stamp = rclpy.time.Time.from_msg(t.header.stamp)
            if (self.get_clock().now()-stamp).nanoseconds > 500_000_000:
                return None
            p = t.transform.translation
            return np.array([p.x, p.y, p.z])
        except Exception:
            return None

    def send(self, xyz, yaw=YAW_DEG):
        self._last = np.asarray(xyz, dtype=float); self._yaw = yaw
        return
        q = quat_from_RzRx(yaw)
        m = PoseStamped()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, xyz)
        (m.pose.orientation.w, m.pose.orientation.x,
         m.pose.orientation.y, m.pose.orientation.z) = q
        self.pub_pose.publish(m)

    def grip(self, v):
        self._last_grip = float(v)

    def enable(self, v):
        """调用 /arm/enable **服务**（std_srvs/SetBool）——不是话题！"""
        if self._dry_run:
            return False
        if not self.cli_enable.wait_for_service(timeout_sec=5.0):
            print('    ⚠️ /arm/enable 服务不可用')
            return None
        req = SetBool.Request()
        req.data = bool(v)
        fut = self.cli_enable.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        r = fut.result()
        if r is not None:
            print(f'    enable({v}) -> success={r.success} msg={r.message}')
        return bool(r is not None and r.success)

    def wait(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def goto(self, target, tol=0.004, timeout=40.0, label=''):
        t0 = time.time()
        p = None
        while time.time() - t0 < timeout:
            if self._fault:
                print(f'    ❌ {self._fault}')
                return False
            self.send(target)
            self.wait(0.25)
            p = self.tcp()
            if p is not None and np.linalg.norm(p - target) < tol:
                print(f'    {label} 到位（误差 '
                      f'{np.linalg.norm(p-target)*1000:.1f}mm）')
                return True
        e = np.linalg.norm(p - target) * 1000 if p is not None else -1
        print(f'    ⚠️ {label} 超时，剩余误差 {e:.1f}mm')
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--obj', nargs=3, type=float, default=None)
    a = ap.parse_args()

    rclpy.init()
    n = Grasp(dry_run=a.dry_run)
    for _ in range(80):
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.tcp() is not None:
            break
    cur = n.tcp()

    if a.obj:
        obj, yaw_grid = np.array(a.obj), 0.0
    elif os.path.exists('/tmp/obj_base.txt'):
        # 优先用 vision_snapshot.py 写下的"行动前快照"（动之前定一次）
        require_fresh('/tmp/obj_base.txt')
        if os.path.getmtime('/tmp/extrinsic.txt') > os.path.getmtime('/tmp/obj_base.txt'):
            raise ValueError('snapshot predates extrinsic calibration')
        with open('/tmp/obj_base.txt') as fh:
            vals = fh.readline().split()[:3]
        obj = np.array(finite_position(vals))
        yaw_grid = float('nan')
        print('（使用行动前快照 /tmp/obj_base.txt）')
    else:
        try:
            obj, yaw_grid = read_vision()
        except Exception as e:
            print(f'❌ 读视觉/外参失败: {e}')
            print('   请确认视觉节点在跑（/tmp/object_pose.txt）且 extr 存在')
            return
    obj = np.array(finite_position(obj))
    tgt = obj + np.array([DX_MM, DY_MM, DZ_MM]) / 1000.0
    finite_position(tgt)
    print(f'物块 (base) = ({obj[0]:.4f}, {obj[1]:.4f}, {obj[2]:.4f})  '
          f'[grid yaw {yaw_grid:+.1f}°]')
    print(f'当前 TCP    = ' + (f'({cur[0]:.4f}, {cur[1]:.4f}, {cur[2]:.4f})'
                              if cur is not None else '未收到 TF'))
    print(f'抓取目标    = ({tgt[0]:.4f}, {tgt[1]:.4f}, {tgt[2]:.4f})  '
          f'[物块 + ({DX_MM:+.0f},{DY_MM:+.0f},{DZ_MM:+.0f})mm, '
          f'偏航 {YAW_DEG:+.0f}°]')
    print(f'棋盘面 z={BOARD_Z:+.4f}   目标距棋盘 {(tgt[2]-BOARD_Z)*1000:+.1f}mm')
    if tgt[2] < BOARD_Z + SAFE_MARGIN:
        print('❌ 目标 z 低于棋盘面，拒绝执行')
        return
    if a.dry_run:
        print('（dry-run：不发运动指令）')
        rclpy.shutdown()
        return

    print('\n开始执行：')
    print('  1) 使能机械臂')
    if cur is None:
        raise RuntimeError('fresh measured TCP unavailable; no motion allowed')
    if not n.enable(True):
        raise RuntimeError('enable request failed')
    n.wait(2.0)
    print(f'  2) 夹爪开到接近角 {APPROACH_GRIP}')
    n.grip(APPROACH_GRIP)
    n.wait(1.5)
    pre = tgt + np.array([0, 0, 0.06])
    print(f'  3) 到预抓取点 z={pre[2]:+.4f}')
    if not n.goto(pre, label='预抓取'):
        raise RuntimeError('pregrasp failed; aborting')
    print('  4) 降到抓取点')
    if not n.goto(tgt, label='抓取点'):
        raise RuntimeError('descent failed; aborting before closure')
    print('  5) 夹紧 + 等合拢')
    n.grip(CLOSE_GRIP)
    n.wait(2.5)
    print('  6) 抬起')
    if not n.goto(tgt + np.array([0, 0, 0.10]), label='抬起'):
        raise RuntimeError('lift failed')
    n.wait(0.5)
    p = n.tcp()
    if p is not None:
        print(f'\n完成。最终 TCP = ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})')
    n._stop = True
    n._last = n._last_grip = None
    n._th.join(timeout=1.)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except (ValueError,RuntimeError,OSError) as exc:
        print(f'❌ grasp rejected: {exc}', file=sys.stderr)
        raise SystemExit(1)
