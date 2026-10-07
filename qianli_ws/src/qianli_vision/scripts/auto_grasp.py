#!/usr/bin/env python3
"""一键自动抓取（姿态受控版）—— 按"固定爪在右、活动爪从左闭合"策略

策略（用户指定，工业标准做法）：
  1. 夹爪完全朝下（工具轴垂直向下）
  2. 固定爪在物块右侧（base -Y 方向）
  3. 左侧活动爪慢慢合上 → 物块被推向固定爪 → 自定心，夹持可靠

为什么必须控制姿态：
  之前只发位置（/arm/target_position），ik_node 的 orientation_mode
  默认是 'none' —— 完全忽略姿态，爪子朝向随机 → 经常夹空。
  现改为发完整位姿（/ik_target, PoseStamped）+ ik_node 的
  orientation_mode='Z'，姿态才受控（实测姿态误差 ~6°）。

坐标推导：
  gripper_frame_link 的 +Z = 夹爪接近方向（指向指尖）
  活动爪在 frame 的 -X 侧，固定爪在 +X 侧
  → R = Rz(yaw)·Rx(180°)；yaw=-90° 时 +X 指向 base -Y（右侧）

用法：
  python3 auto_grasp.py --dry-run      # 只看计划
  python3 auto_grasp.py                # 真抓
"""

import argparse
import math
import sys
import time
import json
from grasp_guard import down_quat_xyzw, finite_position, tool_down_error_deg, FeedbackGuard

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

TCP = 'gripper_frame_link'
POSE_FILE = '/tmp/grasp_pose.txt'
OFFSET_FILE = '/tmp/grasp_offset.txt'
EXT_FILE = '/tmp/extrinsic.txt'
GRIP_OPEN, GRIP_CLOSE = 1.2, 0.0
PARK_DEFAULT = (0.26, 0.01, 0.18)


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


def tool_down_quat(yaw_deg):
    """爪子朝下 + 指定偏航角的四元数。R = Rz(yaw)·Rx(pi)。"""
    a = math.radians(yaw_deg)
    # Rz(a)·Rx(pi) 的解析四元数
    # Rx(pi) = (x=1,y=0,z=0,w=0)；乘以 Rz(a)=(0,0,sin(a/2),cos(a/2))
    ca, sa = math.cos(a / 2), math.sin(a / 2)
    # q = qz * qx  （先绕X再绕Z → 四元数相乘顺序 qz*qx）
    qx = np.array([1.0, 0.0, 0.0, 0.0])          # (x,y,z,w)
    qz = np.array([0.0, 0.0, sa, ca])
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


class AutoGrasp(Node):
    def __init__(self, args):
        super().__init__('auto_grasp')
        self.a = args
        self.ext = read_kv(EXT_FILE)
        if not self.ext:
            raise SystemExit('缺少外参 /tmp/extrinsic.txt')
        pose = read_kv(POSE_FILE)
        off = read_kv(OFFSET_FILE)
        if 'grasp_z' not in pose and 'grasp_z' not in off:
            raise SystemExit('缺少抓取高度（先示教或标定偏移）')
        # 偏移标定优先（更准），否则用示教值
        self.off_x = off.get('off_x', 0.0)
        self.off_y = off.get('off_y', 0.0)
        if math.isnan(self.off_x):
            self.off_x = 0.0
        if math.isnan(self.off_y):
            self.off_y = 0.0
        self.grasp_z = off.get('grasp_z', pose.get('grasp_z'))
        self.safe_z = off.get('safe_z_min',
                              pose.get('safe_z_min', self.grasp_z - 0.01))
        self.quat = tool_down_quat(args.yaw)

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.pub_pose = self.create_publisher(PoseStamped, '/ik_target', 10)
        self.pub_g = self.create_publisher(Float64, '/gripper_command', 10)
        self.create_subscription(PointStamped, '/object_pose',
                                 self._on_obj, 10)
        self.create_subscription(String, '/arm/status', self._on_status, 10)
        self.cli = self.create_client(SetBool, '/arm/enable')
        self.obj_hist = []
        self.cur = None
        self.enabled = False
        self.target = None
        self.feedback = FeedbackGuard()
        self.tool_angle = float('inf')
        self.create_subscription(JointState, '/joint_states', self.feedback.on_joints, 10)

    # ---------- 回调 ----------
    def _on_obj(self, msg):
        if msg.header.frame_id != 'grid':
            return
        age = (self.get_clock().now()-rclpy.time.Time.from_msg(msg.header.stamp)).nanoseconds/1e9
        if age < -.5 or age > .5 or not np.isfinite([msg.point.x,msg.point.y]).all():
            return
        self.obj_hist.append((msg.point.x, msg.point.y))
        if len(self.obj_hist) > 8:
            self.obj_hist.pop(0)

    def _on_status(self, msg):
        self.feedback.on_status(msg)
        try:
            self.enabled = bool(json.loads(msg.data).get('enabled',False))
        except (ValueError,TypeError):
            self.enabled = False

    # ---------- 基础设施 ----------
    def spin_for(self, dur, stream=True):
        t0 = time.time()
        while time.time() - t0 < dur:
            rclpy.spin_once(self, timeout_sec=0.02)
            self._read_tcp()
            if stream and self.target and not self.feedback.fresh(enabled=True):
                self.target = None
                raise RuntimeError('physical feedback stale, faulted or disabled')
            if stream and self.target:
                self._publish_pose()

    def _read_tcp(self):
        self.cur = None
        self.tool_angle = float('inf')
        if not self.feedback.fresh() or self.count_publishers('/joint_states') != 1:
            return
        try:
            matrix = self.feedback.tcp_matrix()
            self.cur = tuple(matrix[:3,3])
            self.tool_angle = math.degrees(math.acos(float(np.clip(-matrix[2,2],-1.,1.))))
        except Exception:
            pass

    def _publish_pose(self):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        x, y, z = self.target
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        (m.pose.orientation.x, m.pose.orientation.y,
         m.pose.orientation.z, m.pose.orientation.w) = self.quat
        self.pub_pose.publish(m)

    def enable(self):
        if not self.cli.wait_for_service(timeout_sec=5.0):
            print('❌ /arm/enable 不可用')
            return False
        req = SetBool.Request()
        req.data = True
        self.cli.call_async(req)
        for _ in range(40):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.enabled:
                return True
        return self.enabled

    def gripper(self, pos, wait=2.5):
        if not self.feedback.fresh(enabled=True):
            raise RuntimeError('gripper requires current enabled physical feedback')
        m = Float64()
        m.data = pos
        for _ in range(5):
            self.pub_g.publish(m)
            self.spin_for(0.05)
        self.spin_for(wait)

    def gripper_close_slow(self, steps=5, dt=0.7):
        """慢慢合上：避免撞击物块把它推飞（用户策略第 3 条）。"""
        print(f'   慢速闭合：{GRIP_OPEN} → {GRIP_CLOSE}，{steps} 步')
        for i in range(1, steps + 1):
            if not self.feedback.fresh(enabled=True):
                raise RuntimeError('gripper closure interrupted by stale/disabled feedback')
            v = GRIP_OPEN + (GRIP_CLOSE - GRIP_OPEN) * i / steps
            m = Float64()
            m.data = v
            self.pub_g.publish(m)
            self.spin_for(dt)
        # 最后再来一次确保到位
        self.gripper(GRIP_CLOSE, wait=2.0)

    def goto(self, x, y, z, timeout=14.0, tol=0.006):
        """发位姿目标并等机械臂真正到位（姿态受控）。"""
        z = max(z, self.safe_z)
        finite_position([x,y,z],self.a.reach_limit)
        self.target = (x, y, z)
        t0 = time.time()
        stable = 0
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            self._read_tcp()
            if not self.enabled or not self.feedback.fresh(enabled=True):
                self.target = None
                print('      ❌ 运动失能/掉线，终止；禁止自动重新使能')
                return False, float('nan')
            self._publish_pose()
            if self.cur:
                d = math.dist(self.cur, (x, y, z))
                stable = stable + 1 if d < tol and self.tool_angle <= 5. else 0
                if stable >= 15:
                    return True, d
        d = math.dist(self.cur, (x, y, z)) if self.cur else float('nan')
        return False, d

    # ---------- 主流程 ----------
    def run(self):
        a = self.a
        if not a.dry_run and a.board_z is None:
            raise ValueError('real execution requires measured --board-z; taught height alone is insufficient')
        if a.board_z is not None:
            if not math.isfinite(a.board_z) or self.grasp_z < a.board_z+.005:
                raise ValueError('grasp height too close to or below measured board')
            self.safe_z = max(self.safe_z,a.board_z+.005)

        if not a.dry_run:
            print('⓪ 使能 + 归位（让开相机视野）')
            if not self.enable():
                print('❌ 使能失败')
                return False
            park = tuple(a.park) if a.park else PARK_DEFAULT
            ok,_ = self.goto(park[0], park[1], park[2], timeout=16)
            if not ok:
                return False
            self.spin_for(2.0)
        else:
            print(f'[dry-run] 先归位到 {a.park or PARK_DEFAULT}')

        print('① 等待物块定位…')
        self.obj_hist = []
        t0 = time.time()
        grid = None
        while time.time() - t0 < 25:
            rclpy.spin_once(self, timeout_sec=0.05)
            self._read_tcp()
            if len(self.obj_hist) >= 5:
                xs = [p[0] for p in self.obj_hist[-5:]]
                ys = [p[1] for p in self.obj_hist[-5:]]
                if max(xs) - min(xs) < 0.005 and max(ys) - min(ys) < 0.005:
                    grid = (sum(xs) / 5, sum(ys) / 5)
                    break
        if grid is None:
            print('❌ 25 秒内未获得稳定的物块位置')
            return False
        gx, gy = grid
        th = math.radians(self.ext['grid_theta_deg'])
        c, s = math.cos(th), math.sin(th)
        ox = c * gx - s * gy + self.ext['grid_origin_x'] + self.off_x
        oy = s * gx + c * gy + self.ext['grid_origin_y'] + self.off_y
        finite_position([ox,oy,self.grasp_z],a.reach_limit)
        reach = math.hypot(ox, oy)
        print(f'   物块 grid=({gx*100:.1f},{gy*100:.1f})cm → '
              f'夹爪目标 base=({ox:.4f},{oy:.4f}) 距基座 {reach*100:.1f}cm')
        print(f'   抓取高度 z={self.grasp_z:.4f}（安全下限 {self.safe_z:.4f}）')
        if reach > a.reach_limit:
            print(f'❌ 超出可达范围（上限 {a.reach_limit*100:.0f}cm）')
            return False

        plan = [('高空转运', ox, oy, self.grasp_z + 0.10),
                ('预抓取', ox, oy, self.grasp_z + 0.03)]
        n_step = 5
        for i in range(1, n_step + 1):
            zz = self.grasp_z + 0.03 - 0.03 * i / n_step
            plan.append((f'下压 {i}/{n_step}', ox, oy, zz))

        if a.dry_run:
            print('\n[dry-run] 计划（姿态：爪子朝下 + 固定爪在右）：')
            for name, x, y, z in plan:
                print(f'   {name}: ({x:.4f}, {y:.4f}, {z:.4f})')
            print(f'   慢速闭合夹爪 {GRIP_OPEN}→{GRIP_CLOSE}')
            print(f'   抬起到 z={self.grasp_z + 0.10:.4f}')
            return True

        print('② 张开夹爪')
        self.gripper(GRIP_OPEN, wait=2.5)

        print('③ 姿态受控接近（高空→预抓取→分步下压）')
        stage_ok = True
        for name, x, y, z in plan:
            ok, d = self.goto(x, y, z)
            print(f'   {name}: z={z:.4f} → {"✅" if ok else "❌"} '
                  f'误差 {d*1000:.1f}mm')
            if not ok:
                self.target = None
                print('   ❌ 动作未到位，中止，不继续下降或闭爪')
                return False

        print('④ 慢速闭合夹爪（活动爪从左侧合上）')
        self.gripper_close_slow()

        print('⑤ 抬起')
        ok_lift, d = self.goto(ox, oy, self.grasp_z + 0.10, timeout=16)
        print(f'   抬起 {"✅" if ok_lift else "❌"} 误差 {d*1000:.1f}mm')

        if stage_ok and ok_lift:
            print('\n运动流程到位；尚无物块抬升反馈，抓取是否成功未验证')
            return True
        print('\n⚠️ 流程结束，但有动作未到位 —— 请检查是否夹住')
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--drop', nargs=2, type=float, metavar=('X', 'Y'))
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--park', nargs=3, type=float, metavar=('X', 'Y', 'Z'))
    ap.add_argument('--reach-limit', type=float, default=0.36)
    ap.add_argument('--board-z',type=float,help='实测棋盘面 base_link Z（米），真机执行必填')
    ap.add_argument('--yaw', type=float, default=-90.0,
                    help='偏航角（度）—— 决定固定爪朝哪边；-90=固定爪在右')
    a = ap.parse_args()

    rclpy.init()
    node = AutoGrasp(a)
    try:
        ok = node.run()
    except KeyboardInterrupt:
        ok = False
        print('\n已中断（机械臂保持当前位置）')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
