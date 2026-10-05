#!/usr/bin/env python3
"""一键自动抓取：视觉定位 → 上方接近 → 下压 → 闭合 → 抬起（可选放到指定位置）

完整数据流：
  object_localizer 发布 /object_pose (grid, 米)
      ↓ 本脚本用外参变换到 base_link
  目标点 = (物块x, 物块y, 抓取高度)
      ↓ 分三段落位（高空转运 → 预抓取 → 抓取高度）
  闭合夹爪 → 抬起 → （可选）移动到放置点并松开

抓取高度来源：/tmp/grasp_pose.txt（teach_grasp.py 拖动示教产物）
  —— 单目相机测不出物块/桌面高度，这个高度由人眼确认一次后固化。

用法：
  # 只看计划，不动机械臂
  python3 auto_grasp.py --dry-run
  # 真抓
  python3 auto_grasp.py
  # 抓起来后放到指定位置（base_link 米）
  python3 auto_grasp.py --drop 0.20 -0.12
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

TCP = 'gripper_frame_link'
POSE_FILE = '/tmp/grasp_pose.txt'
EXT_FILE = '/tmp/extrinsic.txt'
GRIP_OPEN, GRIP_CLOSE = 1.2, 0.0
# 默认归位点：实测此处相机能看到整个棋盘（不遮挡）
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


class AutoGrasp(Node):
    def __init__(self, args):
        super().__init__('auto_grasp')
        self.a = args
        self.ext = read_kv(EXT_FILE)
        pose = read_kv(POSE_FILE)
        if not self.ext:
            raise SystemExit('缺少外参 /tmp/extrinsic.txt')
        if 'grasp_z' not in pose:
            raise SystemExit('缺少抓取基准 /tmp/grasp_pose.txt（先做拖动示教）')
        self.grasp_z = pose['grasp_z']
        self.safe_z = pose.get('safe_z_min', self.grasp_z - 0.01)

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.pub = self.create_publisher(PointStamped, '/arm/target_position',
                                         10)
        self.pub_g = self.create_publisher(Float64, '/gripper_command', 10)
        self.create_subscription(PointStamped, '/object_pose',
                                 self._on_obj, 10)
        self.create_subscription(String, '/arm/status', self._on_status, 10)
        self.cli = self.create_client(SetBool, '/arm/enable')

        self.obj_grid = None
        self.obj_hist = []
        self.cur = None
        self.enabled = False
        self.target = None

    # ---------- 回调 ----------
    def _on_obj(self, msg):
        self.obj_grid = (msg.point.x, msg.point.y)
        self.obj_hist.append((msg.point.x, msg.point.y))
        if len(self.obj_hist) > 8:
            self.obj_hist.pop(0)

    def _on_status(self, msg):
        self.enabled = '"enabled": true' in msg.data

    # ---------- 基础操作 ----------
    def spin_for(self, dur, stream=True):
        t0 = time.time()
        while time.time() - t0 < dur:
            rclpy.spin_once(self, timeout_sec=0.02)
            self._read_tcp()
            if stream and self.target:
                self._publish_target()

    def _read_tcp(self):
        try:
            tr = self.buf.lookup_transform('base_link', TCP,
                                           rclpy.time.Time())
            t = tr.transform.translation
            self.cur = (t.x, t.y, t.z)
        except Exception:
            pass

    def _publish_target(self):
        m = PointStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.point.x, m.point.y, m.point.z = self.target
        self.pub.publish(m)

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
        m = Float64()
        m.data = pos
        for _ in range(5):
            self.pub_g.publish(m)
            self.spin_for(0.05)
        self.spin_for(wait)

    def goto(self, x, y, z, timeout=10.0, tol=0.004, label=''):
        z = max(z, self.safe_z)          # 安全下限
        self.target = (x, y, z)
        t0 = time.time()
        stable = 0
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            self._read_tcp()
            self._publish_target()
            if self.cur:
                d = math.dist(self.cur, (x, y, z))
                stable = stable + 1 if d < tol else 0
                if stable >= 10:          # 连续 10 次(≈0.4s)在容差内
                    return True, d
        d = math.dist(self.cur, (x, y, z)) if self.cur else float('nan')
        return False, d

    # ---------- 主流程 ----------
    def run(self):
        a = self.a

        # 0) 使能 + 归位（关键：机械臂必须离开棋盘视野，否则相机看不到物块）
        if not a.dry_run:
            print('⓪ 使能运动…')
            if not self.enable():
                print('❌ 使能失败')
                return False
            park = tuple(a.park) if a.park else PARK_DEFAULT
            print(f'   归位到 ({park[0]:.3f}, {park[1]:.3f}, {park[2]:.3f})'
                  '（让开相机视野）')
            self.goto(*park, timeout=12.0)
            print('   等待相机视野恢复…')
            self.spin_for(2.0)
        else:
            park = tuple(a.park) if a.park else PARK_DEFAULT
            print(f'[dry-run] 会先归位到 {park}，再开始检测')

        # 1) 等稳定的物块检测
        print('① 等待物块定位…')
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
            print('❌ 25 秒内未获得稳定的物块位置（检查相机/背景/遮挡）')
            return False
        gx, gy = grid
        th = math.radians(self.ext['grid_theta_deg'])
        c, s = math.cos(th), math.sin(th)
        ox = c * gx - s * gy + self.ext['grid_origin_x']
        oy = s * gx + c * gy + self.ext['grid_origin_y']
        print(f'   物块 grid=({gx*100:.1f},{gy*100:.1f})cm → '
              f'base=({ox:.4f},{oy:.4f}) 抓取高度 z={self.grasp_z:.4f}')

        plan = [('高空转运', (ox, oy, self.grasp_z + 0.12)),
                ('预抓取', (ox, oy, self.grasp_z + 0.03)),
                ('下压到抓取高度', (ox, oy, self.grasp_z))]
        if a.dry_run:
            print('\n[dry-run] 计划：')
            print(f'   张开夹爪 {GRIP_OPEN}')
            for name, p in plan:
                print(f'   {name}: ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})')
            print(f'   闭合夹爪 {GRIP_CLOSE}')
            print(f'   抬起到 z={self.grasp_z + 0.12:.4f}')
            if a.drop:
                print(f'   移动到放置点 ({a.drop[0]}, {a.drop[1]}) 并松开')
            return True

        # 2) 使能
        print('② 使能运动…')
        if not self.enable():
            print('❌ 使能失败')
            return False

        # 3) 张开夹爪
        print('③ 张开夹爪')
        self.gripper(GRIP_OPEN, wait=2.0)

        # 4) 三段式接近
        for name, (x, y, z) in plan:
            ok, d = self.goto(x, y, z, label=name)
            print(f'   {name}: 目标 z={z:.4f} → '
                  f'{"✅" if ok else "⚠️ 超时"} 误差 {d*1000:.1f}mm')
            if name == '下压到抓取高度' and not ok:
                print('   ⚠️ 未到达抓取高度，仍继续闭合（可能夹不稳）')

        # 5) 闭合
        print('④ 闭合夹爪')
        self.gripper(GRIP_CLOSE, wait=2.5)

        # 6) 抬起
        print('⑤ 抬起')
        ok, d = self.goto(ox, oy, self.grasp_z + 0.12)
        print(f'   抬起 {"✅" if ok else "⚠️ 超时"} 误差 {d*1000:.1f}mm')

        # 7) 可选放置
        if a.drop:
            dx, dy = a.drop
            print(f'⑥ 移动到放置点 ({dx}, {dy})')
            self.goto(dx, dy, max(self.grasp_z + 0.12, self.safe_z + 0.12))
            print('   松开夹爪')
            self.gripper(GRIP_OPEN, wait=2.0)
            print('   撤回')
            self.target = None
            self.spin_for(1.5, stream=False)

        print('\n🎉 抓取流程完成')
        return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--drop', nargs=2, type=float, metavar=('X', 'Y'),
                    help='抓取后放到 base_link 的 (x, y) 米处')
    ap.add_argument('--dry-run', action='store_true', help='只打印计划')
    ap.add_argument('--park', nargs=3, type=float, metavar=('X', 'Y', 'Z'),
                    help='归位点（base_link 米），默认 (0.26, 0.01, 0.18)')
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
