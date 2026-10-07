#!/usr/bin/env python3
"""外参标定（多点法）：grid → base_link，替换旧的"两点法"

旧两点法的问题（见 docs/GRASP_REAL_AUDIT.md）
-------------------------------------------
* 只用 2 个点：只能定 θ，没有冗余、没有残差、无法发现摆错；
* 记录的是 ``gripper_frame_link`` 的位置，而人是按**爪口**去对格点的，
  两者差几十毫米 → 系统性误差；
* 实测 A→B 距离 34.6mm vs 格宽 33mm，两个内角点 Z 还差 9.5mm。

本工具
------
1. 用模型和新鲜真机关节反馈计算**固定爪顶端**，TCP 只用于诊断；
2. 按提示依次触碰 **N 个已知格点**（默认 5 个，L 形铺开）；
3. 闭式最小二乘（Umeyama，固定比例）解 θ 与平移；
4. 输出**残差**、**反推的格宽**——这两个数直接告诉你标定可不可信。

不需要使能扭矩：支撑机械臂，扭矩关闭后用手把固定爪尖摆到格点上。

用法::

    # 在独立 direct 只读 ROS 会话中使用现有模型
    python3 extrinsic_calib_multi.py --cell-cm 3.3
    # 按提示摆好每个格点后： touch /tmp/grid_mark

标定参考点：**固定爪顶端**（不是两爪的全局最低点）
---------------------------------------------------
为什么记录"固定爪顶端"而不是模型算的"整机最低点"：

* 用户眼睛对准格点的是**固定爪顶端**（SO-101 是单活动爪结构，
  固定爪 = gripper_link，活动爪 = moving_jaw）；
* 旧版记录 ``lowest_point()`` —— 它是 **gripper_link 与 moving_jaw
  两个网格里 z 更低的那个**。夹爪开度一大，最低点会落到**活动爪**
  上，水平错开 = 爪口开度（几十毫米），而且这个错开随每个点的
  姿态变化 → 记录点根本不是网格 → 拟合残差 61mm（2026-10-06 实测）；
* 修复：只取 gripper_link（固定爪）网格的最低点 = 固定爪顶端。

板面 z 判据（"划过桌面建模"的数学化）
------------------------------------
固定爪顶端**贴到板面上**时，它的 z 必须等于板面高度
（= 实测桌面 + 棋盘纸厚）。每个打点都要过这一关：

* 通过 → 说明此刻固定爪真的在板面上，记录的 xy 就是用户对准的格点；
* 不通过 → 要么没贴到板，要么当前姿态下模型 FK 算错，**本点作废**，
  提示重摆。

判据对应的操作就是"让爪子在板面上划过"：划的时候固定爪顶端
永远在板面 z 上，所以模型算出的顶端 z 也必须是板面 z。
"""

from __future__ import annotations

from project_paths import calibration_path

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np

TRIGGER = '/tmp/grid_mark'
TCP_CALIB = calibration_path('tcp_calib.txt')
OUT = calibration_path('extrinsic.txt')
OUT_JSON = calibration_path('extrinsic_marks.json')
DEFAULT_POINTS_CM = [(0.0, 0.0), (9.9, 0.0), (0.0, 6.6), (9.9, 6.6), (3.3, 3.3)]

# 固定爪顶端 = gripper_link 网格的最低点（SO-101 固定爪在 gripper_link 上）
FIXED_LINK = 'gripper_link'


def read_kv(path):
    values = {}
    try:
        for line in open(path):
            line = line.strip()
            if '=' in line and not line.startswith('#'):
                key, value = line.split('=', 1)
                try:
                    values[key.strip()] = float(value.strip())
                except ValueError:
                    pass
    except OSError:
        pass
    return values


def solve_planar(grid_xy, base_xy):
    """闭式解：base = Rz(θ)·grid + t（比例固定为 1）。

    返回 (theta_rad, t(2,), residuals_m)
    """
    G = np.asarray(grid_xy, dtype=float)
    B = np.asarray(base_xy, dtype=float)
    gc, bc = G.mean(axis=0), B.mean(axis=0)
    Gd, Bd = G - gc, B - bc
    # θ 使 R(θ)Gd 最贴近 Bd
    num = float(np.sum(Gd[:, 0] * Bd[:, 1] - Gd[:, 1] * Bd[:, 0]))
    den = float(np.sum(Gd[:, 0] * Bd[:, 0] + Gd[:, 1] * Bd[:, 1]))
    theta = math.atan2(num, den)
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, -s], [s, c]])
    t = bc - R @ gc
    predicted = (R @ G.T).T + t
    residuals = np.linalg.norm(predicted - B, axis=1)
    return theta, t, residuals, predicted


def implied_cell_size(grid_xy, base_xy, pairs):
    """用 base 系下的实测距离反推格宽，独立校验比例是否正确。"""
    out = []
    for i, j in pairs:
        g = math.dist(grid_xy[i], grid_xy[j])
        b = math.dist(base_xy[i], base_xy[j])
        if g > 1e-9:
            out.append(b / g)
    return out


def collect(args, points):
    import rclpy
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener
    from std_msgs.msg import String
    from qianli_vision.calibration import physical_collection_error

    # 参考点优先级：URDF 的 tcp_link > 旧的 /tmp/tcp_calib.txt + 法兰。
    #
    # 为什么必须优先用 tcp_link：它是权威定义，而且**会随目标物块宽度变**。
    # W=40mm（EVA 块）时偏移是 (-20.0, 0, -5.364)mm；W=14mm 时是 -7.0mm。
    # 差 13mm，拿旧的标定文件会让整套外参平移 13mm。
    calib = read_kv(args.tcp)
    offset = np.array([calib.get('offset_x', 0.0), calib.get('offset_y', 0.0),
                       calib.get('offset_z', 0.0)])
    flange = calib.get('flange', 'gripper_link')

    class Collector(Node):
        def __init__(self):
            super().__init__('extrinsic_calib_multi')
            self.buf = Buffer()
            self.listener = TransformListener(self.buf, self)
            self.marks = []
            self.use_tcp_link = False
            self.ref_desc = ''
            # 逐点记录"TCP 比固定爪顶端高多少"用的模型
            import sys as _sys
            import os as _os
            _sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
            from gripper_model import GripperModel, JOINTS as _J
            self.model = GripperModel(stride=14)
            self.joint_names = list(_J)
            self.joints = {}
            self.joints_at = None
            self.driver_status = None
            self.status_at = None
            self.create_subscription(
                __import__('sensor_msgs.msg', fromlist=['JointState'])
                .JointState, '/joint_states', self._on_joints, 1)
            self.create_subscription(String, '/arm/status', self._on_status, 1)

        def _on_joints(self, m):
            if len(m.name) == len(m.position):
                self.joints = dict(zip(m.name, m.position))
                age = (self.get_clock().now() - rclpy.time.Time.from_msg(m.header.stamp)).nanoseconds / 1e9
                self.joints_at = time.monotonic() - age

        def _on_status(self, m):
            try:
                self.driver_status = json.loads(m.data)
            except (TypeError, ValueError):
                self.driver_status = None
            self.status_at = time.monotonic()

        def collection_error(self):
            now = time.monotonic()
            reason = physical_collection_error(
                self.driver_status, self.joints,
                None if self.status_at is None else now - self.status_at,
                None if self.joints_at is None else now - self.joints_at)
            if reason:
                return reason
            for topic in ('/joint_states', '/arm/status'):
                sources = self.get_publishers_info_by_topic(topic)
                if len(sources) != 1 or sources[0].node_name != 'so101_driver':
                    return f'{topic} 必须由唯一 so101_driver 发布，暂停记录'
            return None

        def tcp_above_lowest(self):
            """TCP 比"固定爪顶端"高多少（米），逐点按当前姿态算。

            为什么必须逐点算、不能用一个常数：用户是手拖的，工具轴的倾角
            每个点都不一样；而 TCP 相对固定爪顶端的高度**随倾角变化**。
            用常数会引入随姿态漂移的系统性误差。
            """
            if len(self.joints) < 6:
                return None
            try:
                j = {k: float(self.joints[k]) for k in self.joint_names}
                low, _ = self.fixed_tip(j)
            except Exception:  # noqa: BLE001
                return None
            tcp = self.tcp_base()
            if tcp is None:
                return None
            return float(tcp[2] - low[2])

        def fixed_tip(self, joints):
            """固定爪顶端 = gripper_link 网格最低点，在 base_link 下的完整位置。

            ★★ 用户眼睛对准格点、真正碰到板面的就是它，不是 TCP。

            为什么必须是 gripper_link 单独算、不能取两爪全局最低：
            夹爪开度一大，"整机最低点"会落到**活动爪**（moving_jaw）上，
            水平错开几十毫米，而且随每个点的姿态变化 —— 记录点根本不构成
            网格（2026-10-06 实测 RMS 61mm 就是这么来的）。

            固定爪是刚体（gripper_link），它顶端相对 base_link 的位置是纯
            正运动学，与夹爪开度无关 —— 这才是稳定、可复现的参考点。

            返回 (顶端 xyz, 该点所在 link)。
            """
            T = self.model.solve(joints)
            M = T.get(FIXED_LINK)
            pts = self.model.parts.get(FIXED_LINK)
            if M is None or pts is None or not len(pts):
                return None, None
            world = (M[:3, :3] @ pts.T).T + M[:3, 3]
            k = int(np.argmin(world[:, 2]))
            return world[k].copy(), FIXED_LINK

        def fixed_tip_base(self):
            """固定爪顶端在 base_link 下的位置（收集端封装）。"""
            if len(self.joints) < 6:
                return None
            try:
                j = {k: float(self.joints[k]) for k in self.joint_names}
                low, _ = self.fixed_tip(j)
                return low
            except Exception:  # noqa: BLE001
                return None

        def probe(self):
            """启动时探一次：TF 里有没有 tcp_link。"""
            for _ in range(50):
                rclpy.spin_once(self, timeout_sec=0.1)
                if isinstance(self.driver_status, dict) and self.driver_status.get('mode') == 'sim':
                    raise RuntimeError('检测到 sim 驱动，拒绝采集真机标定点；请使用独立 direct 只读会话')
                if self.collection_error():
                    continue
                try:
                    self.buf.lookup_transform('base_link', 'tcp_link',
                                              rclpy.time.Time())
                    self.use_tcp_link = True
                    self.ref_desc = 'tcp_link（URDF 权威定义）'
                    return
                except Exception:
                    pass
            if calib:
                self.ref_desc = (f'{flange} + 旧标定偏移 '
                                 f'({offset[0]*1000:+.1f}, '
                                 f'{offset[1]*1000:+.1f}, '
                                 f'{offset[2]*1000:+.1f}) mm')
            else:
                self.ref_desc = f'{flange}（无 TCP 偏移）'

        def tcp_base(self):
            """参考点在 base_link 下的位置。"""
            if self.use_tcp_link:
                try:
                    tr = self.buf.lookup_transform('base_link', 'tcp_link',
                                                   rclpy.time.Time())
                    t = tr.transform.translation
                    return np.array([t.x, t.y, t.z])
                except Exception:
                    pass
            # 兜底：法兰 + 旧标定偏移
            try:
                tr = self.buf.lookup_transform('base_link', flange,
                                               rclpy.time.Time())
            except Exception:
                return None
            t = tr.transform.translation
            q = tr.transform.rotation
            n = math.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2)
            x, y, z, w = q.x / n, q.y / n, q.z / n, q.w / n
            R = np.array([
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
            p = np.array([t.x, t.y, t.z])
            if calib:
                p = p + R @ offset
            return p

    rclpy.init()
    node = Collector()
    try:
        node.probe()
        reason = node.collection_error()
        if reason:
            raise RuntimeError(reason)
    except Exception:
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()
        raise
    print(f'使用参考点：{node.ref_desc}')
    if not node.use_tcp_link:
        print(f'⚠️ TF 里查不到 tcp_link（robot_state_publisher 起了吗？）。'
              f'退回旧参考点会引入系统性偏移。')
    print()

    for index, (gx, gy) in enumerate(points, 1):
        prompt = (f'▶ 第 {index}/{len(points)} 点：把**固定爪顶端**（gripper_link '
                  f'最低点，即压到板上的那个爪尖）对准 grid '
                  f'({gx:.1f}, {gy:.1f}) cm，并**压到板面上**')
        print(prompt)
        print('    （固定爪=不动的那个爪，不是活动爪；顶端贴板 → 模型算的 '
              '顶端 z 必须等于板面，本工具据此校验）')
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if os.path.exists(args.trigger):
                try:
                    request = Path(args.trigger).read_text(encoding='utf-8').strip()
                    os.remove(args.trigger)
                except OSError:
                    continue
                if request:
                    try:
                        requested_point = json.loads(request)['point']
                    except (KeyError, TypeError, ValueError):
                        print('   ⛔ 记录请求格式错误，本次不记录')
                        continue
                    if requested_point != index:
                        print(f'   ⛔ 旧的第 {requested_point} 点请求，当前已到第 {index} 点，本次不记录')
                        continue
                reason = node.collection_error()
                if reason:
                    print(f'   ⛔ {reason}；本次不记录')
                    continue
                p = node.tcp_base()
                if p is None:
                    print('   ⚠️ 读不到 TF，本次忽略，请重试')
                    continue
                dz = node.tcp_above_lowest()
                if dz is None:
                    print('   ⚠️ 算不出"TCP 高于固定爪顶端"，'
                          '检查 /joint_states 与 gripper_model')
                    continue
                low = node.fixed_tip_base()
                if low is None:
                    print('   ⚠️ 算不出固定爪顶端，检查 /joint_states 与 '
                          'gripper_model')
                    continue

                gj = None
                try:
                    gj = float(node.joints.get('gripper'))
                except (TypeError, ValueError):
                    pass

                # ---- 板面 z 判据（= "划过桌面建模"） ----
                #
                # 固定爪顶端**贴到板面上**时，它的 z 必须等于板面高度
                # （= 实测桌面 + 棋盘纸厚）。这是唯一真正要保证的事：
                #
                # * 通过 → 此刻固定爪真的在板面上，记录的 xy 就是用户
                #   眼睛对准的格点；
                # * 不通过 → 要么没贴到板，要么当前姿态下模型 FK 算错。
                #   旧版只查"接触点与 TCP 的水平错开"，而那个错开在
                #   姿态倾斜时看不出来（5 个点一致地偏，RMS 照样小）；
                #   直接对板面 z 就没有这个盲区。
                board_z = args.table_z + args.board_mm / 1000.0
                z_tol = args.z_tol_mm / 1000.0
                z_err = low[2] - board_z
                if abs(z_err) > z_tol:
                    print(f'   ⛔ 固定爪顶端 z = {low[2]*1000:+.2f} mm，'
                          f'板面应 {board_z*1000:+.2f} mm，'
                          f'差 {z_err*1000:+.2f} mm（容差 ±{args.z_tol_mm:.1f}）')
                    print(f'      要么没贴到板面（悬空了？），要么当前姿态'
                          f'下模型 FK 算错。**本次不记录。**')
                    print(f'      重新摆：固定爪顶端真正压到板面上再触发。')
                    continue

                # ---- 参考点 = 固定爪顶端（不是两爪全局最低点） ----
                node.marks.append({
                    'grid_cm': [gx, gy],
                    # ★ 主参考点 = 固定爪顶端（用户眼睛对准、压到格点的
                    #   就是它；gripper_link 网格最低点，与开度无关）
                    'contact_m': [float(low[0]), float(low[1]), float(low[2])],
                    # TCP 只作诊断留档：它沿工具轴比固定爪顶端后退，
                    # 倾斜时水平会偏，**不能**拿来当对齐参考。
                    'base_m': [float(p[0]), float(p[1]), float(p[2])],
                    'tcp_above_lowest_m': dz,
                    'gripper_rad': gj,
                    # 存完整关节角，便于事后复算/诊断
                    'joints': {k: float(node.joints[k])
                               for k in node.joint_names},
                    'driver_mode': node.driver_status['mode'],
                    'at': time.strftime('%H:%M:%S')})
                # 每点立即落盘：即使中途被打断，已记录的点也不丢，
                # 之后可用 --solve-only 直接续算。
                with open(args.json, 'w') as _fh:
                    json.dump(node.marks, _fh, indent=2)
                print(f'   ✅ 固定爪顶端 = ({low[0]:.4f}, {low[1]:.4f}, '
                      f'{low[2]:.4f})')
                print(f'      z-板面 = {z_err*1000:+.2f} mm'
                      + (f'   (夹爪角 {math.degrees(gj):.1f}°，仅参考)'
                         if gj is not None else ''))
                print(f'      TCP = ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})  '
                      f'高于固定爪顶端 {dz*1000:+.2f} mm')
                # 工具倾角仅供现场参考：固定爪顶端贴板时 z=板面，与倾角
                # 无关（这正是本方法相对"用 TCP 对齐"稳健的原因）。
                try:
                    import math as _m
                    tl = abs(low[2] - p[2])
                    span = _m.sqrt((low[0]-p[0])**2 + (low[1]-p[1])**2
                                   + (low[2]-p[2])**2)
                    if span > 1e-9:
                        tilt = _m.degrees(_m.acos(min(1.0, tl / span)))
                        print(f'      工具轴偏离竖直 ≈ {tilt:.1f}°（仅参考，'
                              f'不影响本方法）')
                except Exception:  # noqa: BLE001
                    pass
                break
    marks = node.marks
    node.destroy_node()
    if rclpy.ok():
        rclpy.try_shutdown()
    with open(args.json, 'w') as fh:
        json.dump(marks, fh, indent=2)
    return marks


def selftest(cell_cm, trials=500, seed=0):
    """离线自检：已知真值 + 注入打点噪声，看解出来的参数偏多少。

    为什么需要：触标法唯一的误差来源是"手把爪口摆到格点上的重复性"。
    这里假设每个打点位置有 σ 的高斯误差，做蒙特卡洛，看解出的平移/旋转
    偏离真值多少 —— 这个数直接回答"这次标定有多可信"，比单次残差更有意义
    （单次残差只反映这一次的一致性，不反映对真值的偏差）。
    """
    rng = np.random.default_rng(seed)
    pts = np.array(DEFAULT_POINTS_CM) / 100.0
    theta_true = math.radians(-97.75)
    t_true = np.array([0.3420, 0.0584])
    c, s = math.cos(theta_true), math.sin(theta_true)
    R = np.array([[c, -s], [s, c]])
    base_true = (R @ pts.T).T + t_true

    print('=' * 74)
    print('  外参工具离线自检（蒙特卡洛，已知真值）')
    print('=' * 74)
    print(f'  打点数 {len(pts)}   真值 θ={math.degrees(theta_true):+.2f}°  '
          f't=({t_true[0]:.4f}, {t_true[1]:.4f})')
    print()
    print(f'{"打点σ(mm)":>11}{"平移误差均值(mm)":>18}{"平移误差95分位":>17}'
          f'{"θ误差均值(°)":>15}')
    print('-' * 74)
    ok_all = True
    for sigma_mm in (0.5, 1.0, 2.0, 3.0):
        sig = sigma_mm / 1000.0
        et, eth = [], []
        for _ in range(trials):
            noisy = base_true + rng.normal(0, sig, base_true.shape)
            th, t, _res, _p = solve_planar(pts, noisy)
            et.append(float(np.linalg.norm(t - t_true)) * 1000)
            eth.append(abs(math.degrees(th - theta_true)))
        p95 = float(np.percentile(et, 95))
        print(f'{sigma_mm:>11.1f}{np.mean(et):>18.3f}{p95:>17.3f}'
              f'{np.mean(eth):>15.4f}')
        if sigma_mm == 1.0 and np.mean(et) > 3.0:
            ok_all = False
    print()
    print(f'  判据：σ=1mm 时平移误差应仍在 1mm 量级（<3mm）→ '
          f'{"✅ 通过" if ok_all else "❌ 不通过"}')
    print('=' * 74)
    return 0 if ok_all else 2


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cell-cm', type=float, default=3.3)
    ap.add_argument('--tcp', default=TCP_CALIB)
    ap.add_argument('--json', default=OUT_JSON)
    ap.add_argument('--trigger', default=TRIGGER,
                    help='触标前端写入的记录请求文件')
    ap.add_argument('--out', default=OUT)
    ap.add_argument('--solve-only')
    ap.add_argument('--table-z', type=float, default=-0.06909,
                    help='实测桌面高度（base_link 下），用于交叉验证')
    ap.add_argument('--board-mm', type=float, default=0.5,
                    help='棋盘厚度（触标点打在板面上，比桌面高这么多）。'
                         '实测：棋盘是打印纸贴在桌上，厚度 <1mm，故取 0.5mm。'
                         '若换成裱在泡沫板上的硬板，这个值必须改。')
    ap.add_argument('--z-tol-mm', type=float, default=2.0,
                    help='录点判据 + 交叉验证容差。录点时要求**固定爪顶端**'
                         '的 z 落在板面 ±该值内（= "划过桌面建模"：贴到板'
                         '面时顶端 z 必须等于板面高度）；最后交叉验证也用它。'
                         '桌面 z 本身残差 RMS 0.469mm，棋盘厚度不确定度 '
                         '±0.5mm，合成后取 2mm。')
    ap.add_argument('--object-mm', type=float, default=40.0,
                    help='目标物块宽度。tcp_link 是按这个宽度定的开合中心。')
    ap.add_argument('--max-face-offset-mm', type=float, default=5.0,
                    help='[保留兼容] 旧版"接触点与固定爪内侧面水平错开"判据，'
                         '已被板面 z 判据（--z-tol-mm）取代，不再使用。')
    ap.add_argument('--selftest', action='store_true',
                    help='离线自检：注入打点噪声，看解出的参数误差')
    ap.add_argument('--points', default=None,
                    help='自定义触标格点，分号分隔、逗号分 x/y，单位 **cm**。'
                         '例："0,0;9.9,0;19.8,0;0,6.6;9.9,6.6;19.8,6.6;'
                         '0,13.2;9.9,13.2;19.8,13.2" 就是铺满棋盘的 3x3。'
                         '不给则用内置的 5 点 L 形。'
                         '点越多、铺得越开，残差越能压住（触点噪声是主要误差源）。')
    args = ap.parse_args()

    if args.selftest:
        return selftest(args.cell_cm)

    if args.solve_only:
        with open(args.solve_only) as fh:
            marks = json.load(fh)
    else:
        if args.points:
            # 自定义格点："x,y;x,y;..." 单位 cm。
            # 为什么要支持：触点噪声是外参的主要误差源，点**越多、铺得越开**
            # 越能压住。内置的 5 点 L 形只覆盖棋盘一角。
            try:
                pts = [tuple(float(v) for v in p.split(','))
                       for p in args.points.split(';') if p.strip()]
                assert all(len(t) == 2 for t in pts)
            except Exception as exc:  # noqa: BLE001
                print(f'❌ --points 解析失败: {exc}')
                return 1
            print(f'使用自定义 {len(pts)} 个格点: {pts}')
        else:
            pts = DEFAULT_POINTS_CM
        try:
            marks = collect(args, pts)
        except RuntimeError as exc:
            print(f'❌ 不能开始真机标定：{exc}')
            return 1

    if len(marks) < 2:
        print(f'❌ 至少需要 2 个点（现在 {len(marks)}）')
        return 1

    grid_m = [np.array(m['grid_cm']) / 100.0 for m in marks]

    # ★ 用"固定爪顶端"(contact_m) 而不是 TCP。
    # 用户眼睛对齐、压到板上的就是固定爪顶端；TCP 沿工具轴比它后退，
    # 工具一斜水平就错开。用 TCP 会让拟合出现 18% 的假各向异性、
    # RMS 62mm；而同一张棋盘经相机+内参验证是**正方形**（比值 1.0007）。
    _use_contact = all('contact_m' in m for m in marks)
    if _use_contact:
        key = 'contact_m'
    else:
        key = 'base_m'
        print('⚠️ 打点里没有 contact_m（旧格式），退回用 TCP —— '
              '工具倾斜时会引入水平误差，建议重打。')
    base_xy = [np.array(m[key][:2]) for m in marks]
    base_z = [m[key][2] for m in marks]

    # 兼容旧格式：旧点里 base_m 是 TCP，z 要扣掉 tcp_above_lowest。
    # 新格式 contact_m 已经是接触点本身，不需要再扣。
    if _use_contact:
        board_z = list(base_z)
        dzs = [float(m.get('tcp_above_lowest_m', 0.0)) for m in marks]
    else:
        dzs = [float(m.get('tcp_above_lowest_m', 0.0)) for m in marks]
        board_z = [bz - dz for bz, dz in zip(base_z, dzs)]
        if any(abs(d) < 1e-9 for d in dzs):
            print('⚠️ 有打点没记录 tcp_above_lowest_m（旧格式 JSON），'
                  'z 无法扣掉 TCP 偏移，交叉验证会不准')

    theta, t, residuals, predicted = solve_planar(grid_m, base_xy)
    theta_deg = math.degrees(theta)
    grid_origin_z = float(np.mean(board_z))

    # 反推格宽：用相邻点距离比
    pairs = [(i, i + 1) for i in range(len(marks) - 1)]
    scales = implied_cell_size(grid_m, base_xy, pairs)
    implied = [s * args.cell_cm for s in scales]

    print()
    print('=' * 74)
    print('  外参标定结果（grid → base_link）')
    print('=' * 74)
    print(f'  打点数            {len(marks)}')
    print(f'  旋转 θ            {theta_deg:+.3f}°')
    print(f'  平移              ({t[0]:+.4f}, {t[1]:+.4f}) m')
    print(f'  参考点            {"固定爪顶端(contact_m)" if _use_contact else "TCP(旧格式)"}')
    print(f'  接触点平均 z      {np.mean(base_z):+.4f} m   '
          f'(各点极差 {((max(base_z)-min(base_z))*1000):.1f} mm)')
    print(f'  TCP 高于固定爪顶端 {np.mean(dzs)*1000:+.2f} mm '
          f'(逐点 {np.round(np.array(dzs)*1000, 1).tolist()})  ← 仅诊断')
    print(f'  → 板面高度 z      {grid_origin_z:+.4f} m')
    print()
    print(f'  残差 RMS          {float(np.sqrt(np.mean(residuals**2)))*1000:.2f} mm'
          f'   最大 {float(np.max(residuals))*1000:.2f} mm')
    print(f'  单点残差(mm)      {np.round(residuals*1000, 2).tolist()}')
    if implied:
        print(f'  反推格宽          {np.round(implied, 2).tolist()} mm '
              f'(标称 {args.cell_cm*10:.1f} mm)')
        deviation = abs(np.mean(implied) - args.cell_cm * 10)
        print(f'  格宽偏差          {deviation:.2f} mm '
              f'({"✅ 比例一致" if deviation < 1.0 else "⚠️ 比例不符，检查标称格宽或 TCP"})')
    rms = float(np.sqrt(np.mean(residuals ** 2))) * 1000
    zspread = (max(board_z) - min(board_z)) * 1000

    # ---- 交叉验证：**板面**高度应该等于"桌面 + 棋盘厚度" ----
    # 这是独立于相机、也独立于本工具拟合的第三方校验：桌面高度 -69.09mm 是
    # 用"夹爪几何最低点碰桌、多点拟合平面"单独测出来的（残差 RMS 0.469mm）。
    # 注意比的是**扣掉 TCP 偏移之后**的板面高度，不是 TCP 的 z。
    z_expect = args.table_z + args.board_mm / 1000.0
    z_err = grid_origin_z - z_expect
    z_tol = args.z_tol_mm / 1000.0
    z_ok = abs(z_err) < z_tol
    print()
    print(f'  ── 交叉验证（独立于相机、也独立于本工具拟合） ──')
    print(f'  板面高度          {grid_origin_z*1000:+.2f} mm')
    print(f'  期望值            {z_expect*1000:+.2f} mm '
          f'(桌面 {args.table_z*1000:+.2f} + 棋盘纸 {args.board_mm:.1f})')
    print(f'  偏差              {z_err*1000:+.2f} mm   '
          f'容差 ±{args.z_tol_mm:.1f} mm  '
          f'{"✅ 自洽" if z_ok else "❌ 不自洽"}')
    if not z_ok:
        print(f'     → 触标时爪子真碰到板面了吗？或 --board-mm 填错'
              f'（当前 {args.board_mm}）？也可能是 TCP 定义变了。')

    ok = rms < 3.0 and zspread < 8.0 and z_ok
    print()
    print(f'  质量              '
          f'{"✅ 可信" if ok else "⚠️ 不通过"}'
          f'   (阈值 RMS<3mm, Z极差<8mm, '
          f'交叉验证<{args.z_tol_mm:.1f}mm)')

    lines = [
        '# 外参（grid → base_link）：多点最小二乘实测',
        f'# 打点数 {len(marks)}   残差 RMS {rms:.3f} mm   最大 '
        f'{float(np.max(residuals))*1000:.3f} mm',
        f'# 板面各点 z 极差 {zspread:.2f} mm',
        f'# TCP 高于固定爪顶端(逐点) {np.round(np.array(dzs)*1000, 2).tolist()} mm',
        f'# 交叉验证：板面高度 {grid_origin_z*1000:+.2f} mm vs 期望 '
        f'{z_expect*1000:+.2f} mm（桌面 {args.table_z*1000:+.2f} + 棋盘纸 '
        f'{args.board_mm:.1f}），偏差 {z_err*1000:+.2f} mm '
        f'{"OK" if z_ok else "FAIL"}',
        f'# 反推格宽 {np.round(implied,3).tolist() if implied else "n/a"} mm '
        f'(标称 {args.cell_cm*10:.1f} mm)',
        f'# 生成时间 {time.strftime("%Y-%m-%d %H:%M:%S")}   质量 '
        f'{"OK" if ok else "FAIL"}',
        f'grid_origin_x={t[0]:.4f}',
        f'grid_origin_y={t[1]:.4f}',
        f'grid_origin_z={grid_origin_z:.4f}',
        f'grid_theta_deg={theta_deg:.2f}',
        f'cell_cm={args.cell_cm}',
        f'quality_ok={1 if ok else 0}',
        f'cross_check_err_mm={z_err*1000:.3f}',
        f'rms_mm={rms:.3f}',
    ]
    with open(args.out, 'w') as fh:
        fh.write('\n'.join(lines) + '\n')
    print(f'\n📄 结果已写入 {args.out}')
    return 0 if ok else 2


if __name__ == '__main__':
    sys.exit(main())
