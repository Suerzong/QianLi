#!/usr/bin/env python3
"""通用物块抓取规划器

策略（用户提的骨架，这里做成参数化、可泛化）
--------------------------------------------
    1. 张爪        开度 = W + 余量      ← 由模型反解夹爪角，不写死
    2. 到物块上空   TCP 垂直对齐物块中心
    3. 姿态对齐    爪口轴 = 物块 yaw，工具轴垂直 → 两个接触面与物块侧面平行
    4. 垂直下降    直线往下，工具轴保持垂直
    5. 合爪        开度 = W − 挤压量     ← 同样由模型反解
    6. 抬起        到安全高度

泛化性体现在：**没有任何"40mm 专用"的常数**。宽度只以参数形式出现，
所有角度都由 `gripper_model` 从网格反解，换任意尺寸的物块都自动成立。

自由度说明（为什么这个任务可解）
--------------------------------
SO-101 是 5 个臂关节。看似要满足 6 个约束（位置 3 + 姿态 3），其实不是：
工具轴竖直时，绕工具轴自转（wrist_roll）不改变 TCP 位置，所以是
    位置(3) + 工具轴方向(2) = 5 个约束 / 5 个关节
而 yaw 由 wrist_roll 单独提供，不占位置约束。所以 6D 目标其实是可解的，
前提是姿态目标本身自洽（工具轴竖直 + 绕它的 yaw）。

安全
----
每个路点都会检查：
  · 关节是否在实测限位内（含余量）
  · 整机最低点 vs 桌面（`lowest_over_all`）
另外链路里还有 safety_gate 兜底。

用法::

    # 只算不执行（默认）
    ~/mj/bin/python grasp_planner.py --x 0.30 --y -0.05 --yaw-deg 12 --width-mm 40
    # 真执行
    ~/mj/bin/python grasp_planner.py ... --execute
"""

from __future__ import annotations

from project_paths import driver_params_path

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gripper_model import GripperModel, JOINTS  # noqa: E402

ARM_JOINTS = JOINTS[:5]
DRIVER_CFG = os.path.expanduser(
    driver_params_path())


def read_joint_limits(path=DRIVER_CFG):
    zero = rmin = rmax = None
    for line in open(path):
        t = line.strip()
        if t.startswith('zero_raw:'):
            zero = [int(v) for v in t.split('[', 1)[1].rstrip(']').split(',')]
        elif t.startswith('raw_min:'):
            rmin = [int(v) for v in t.split('[', 1)[1].rstrip(']').split(',')]
        elif t.startswith('raw_max:'):
            rmax = [int(v) for v in t.split('[', 1)[1].rstrip(']').split(',')]
    if not (zero and rmin and rmax):
        raise RuntimeError(f'读不到限位: {path}')
    out = {}
    for i, n in enumerate(JOINTS):
        a = (rmin[i] - zero[i]) * math.tau / 4096
        b = (rmax[i] - zero[i]) * math.tau / 4096
        out[n] = (min(a, b), max(a, b))
    return out


def quat_from_R(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
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
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def R_from_quat(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rotvec(R_from, R_to):
    R = R_from.T @ R_to
    c = (np.trace(R) - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    ang = math.acos(c)
    if ang < 1e-9:
        return np.zeros(3)
    if abs(math.pi - ang) < 1e-6:
        # 接近 180°：从 R+I 取一个主轴
        A = (R + np.eye(3)) / 2.0
        v = np.sqrt(np.maximum(np.diag(A), 0.0))
        k = int(np.argmax(v))
        axis = A[:, k] / max(v[k], 1e-12)
        axis = axis / np.linalg.norm(axis)
        return axis * ang
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v / (2 * math.sin(ang)) * ang


class Planner:
    def __init__(self, args):
        self.a = args
        self.m = GripperModel(stride=args.stride)
        fails = self.m.selftest()
        if fails:
            print('❌ FK 自检未通过：')
            for f in fails:
                print('   ·', f)
            raise SystemExit(1)
        self.m.prepare_clearance(stride=args.clear_stride)
        self.limits = read_joint_limits()

    # ---------- 正运动学 ----------
    def fk(self, q):
        d = {k: float(v) for k, v in zip(JOINTS, q)}
        T = self.m.solve(d)
        M = T['tcp_link']
        # tcp_link 是 fixed joint，MuJoCo 会合并；ikpy 里它在链末端，有独立矩阵
        return M[:3, 3], M[:3, :3], T

    # ---------- 6D DLS IK（多种子 + 自适应阻尼） ----------
    def _clamp(self, q):
        out = np.array(q, float)
        for i, n in enumerate(JOINTS):
            lo, hi = self.limits[n]
            out[i] = min(max(out[i], lo), hi)
        return out

    def _ik_once(self, q0, p_target, R_target, iters, tol_p, tol_r):
        """Levenberg-Marquardt 风格：失败就加大阻尼，成功就减小。

        固定阻尼在坏初值下会直接冲到关节限位并卡住（实测：肩部顶到
        +105° 上限，位置残差 277mm）。自适应阻尼让它先走小步、稳下来。
        """
        q = self._clamp(q0)
        lam = 1e-2
        eps = 1e-4
        w_r = self.a.rot_weight
        p, R, _ = self.fk(q)
        best = (float(np.linalg.norm(p_target - p)
                      + w_r * np.linalg.norm(rotvec(R, R_target))), q.copy())
        for _ in range(iters):
            p, R, _ = self.fk(q)
            ep = p_target - p
            er = rotvec(R, R_target)
            cost = float(np.linalg.norm(ep) + w_r * np.linalg.norm(er))
            if cost < best[0]:
                best = (cost, q.copy())
            if np.linalg.norm(ep) < tol_p and np.linalg.norm(er) < tol_r:
                return q, True, float(np.linalg.norm(ep)), float(np.linalg.norm(er))
            J = np.zeros((6, 5))
            for i in range(5):
                qq = q.copy()
                qq[i] += eps
                qq = self._clamp(qq)
                if abs(qq[i] - q[i]) < 1e-12:
                    continue
                p2, R2, _ = self.fk(qq)
                J[:3, i] = (p2 - p) / (qq[i] - q[i])
                J[3:, i] = rotvec(R, R2) / (qq[i] - q[i])
            e = np.concatenate([ep, w_r * er])
            try:
                dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(6), e)
            except np.linalg.LinAlgError:
                lam *= 10
                continue
            step = np.clip(dq, -0.15, 0.15)
            # dq 只有 5 维（臂关节），夹爪角不参与 IK，补 0 对齐到 6 维
            q_new = self._clamp(q + np.concatenate([step, [0.0]]))
            p2, R2, _ = self.fk(q_new)
            cost2 = float(np.linalg.norm(p_target - p2)
                          + w_r * np.linalg.norm(rotvec(R2, R_target)))
            if cost2 < cost:
                q = q_new
                lam = max(lam * 0.5, 1e-6)
            else:
                lam = min(lam * 4.0, 1e3)
        p, R, _ = self.fk(q)
        return (best[1], False, float(np.linalg.norm(p_target - p)),
                float(np.linalg.norm(rotvec(R, R_target))))

    def ik(self, q0, p_target, R_target, iters=400, tol_p=2e-4, tol_r=1e-3):
        """多起点求解：先试给定种子，再试若干随机种子，取最优。

        单种子实测会在坏初值下卡死（关节顶到限位、残差 277mm），
        多种子是最省事也最有效的补救。
        """
        rng = np.random.default_rng(self.a.ik_rng_seed)
        seeds = [np.array(q0, float)]
        lo = np.array([self.limits[n][0] for n in JOINTS])
        hi = np.array([self.limits[n][1] for n in JOINTS])
        # 只随机化 5 个臂关节；**夹爪角必须原样保留** ——
        # 随机化夹爪会让输出带着一个乱来的开度（实测出现过 -0.127，
        # 紧贴下限，被误判成"触限位"，而且接近过程中爪子会乱动）。
        g_keep = float(q0[5])
        for _ in range(max(0, self.a.ik_seeds - 1)):
            arm = lo[:5] + rng.random(5) * (hi[:5] - lo[:5])
            seeds.append(np.concatenate([arm, [g_keep]]))
        best = None
        for i, s in enumerate(seeds):
            q, conv, ep, er = self._ik_once(s, p_target, R_target,
                                            iters, tol_p, tol_r)
            q = np.array(q, float)
            q[5] = g_keep
            score = ep + self.a.rot_weight * er
            if best is None or score < best[0]:
                best = (score, q, conv, ep, er, i)
            if conv:
                break
        return best[1], best[2], best[3], best[4]

    def clearance(self, q):
        d = {k: float(v) for k, v in zip(JOINTS, q)}
        low, link = self.m.lowest_over_all(d)
        return low[2] - self.a.table_z, link, low

    def check_limits(self, q, margin_deg=2.0):
        bad = []
        for i, n in enumerate(JOINTS):
            lo, hi = self.limits[n]
            mg = math.radians(margin_deg)
            if q[i] < lo + mg or q[i] > hi - mg:
                bad.append(f'{n}={math.degrees(q[i]):+.1f}° '
                           f'(限位 [{math.degrees(lo):+.1f},'
                           f'{math.degrees(hi):+.1f}])')
        return bad

    def path_check(self, q_from, q_to, n=60):
        """沿关节空间直线插值采样，返回 (最小净空, 受限部件, 最危险处比例)。

        为什么必须查：路点各自安全**不代表中间安全**。实测 wp1→wp2 的
        shoulder_pan 要转 -60°，两点都合法，但中途夹爪可能扫过桌面。
        直线插值是最坏情况下的近似（真实控制器也不会走得更激进）。
        """
        worst, wlink, wt = None, None, 0.0
        for k in range(n + 1):
            t = k / n
            q = np.array(q_from, float) + (np.array(q_to, float)
                                           - np.array(q_from, float)) * t
            clr, link, _ = self.clearance(q)
            if worst is None or clr < worst:
                worst, wlink, wt = clr, link, t
        return worst, wlink, wt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, required=True, help='物块中心 X (m)')
    ap.add_argument('--y', type=float, required=True, help='物块中心 Y (m)')
    ap.add_argument('--z', type=float, default=None,
                    help='物块中心 Z (m)；默认 = 桌面 + 物块半高')
    ap.add_argument('--yaw-deg', type=float, default=0.0, help='物块朝向')
    ap.add_argument('--width-mm', type=float, default=40.0, help='物块边长')
    ap.add_argument('--height-mm', type=float, default=40.0, help='物块高度')
    ap.add_argument('--table-z', type=float, default=-0.06909)
    ap.add_argument('--approach-mm', type=float, default=70.0,
                    help='接近时 TCP 停在物块中心上方多高')
    ap.add_argument('--lift-mm', type=float, default=90.0)
    ap.add_argument('--open-extra-mm', type=float, default=20.0,
                    help='张爪余量：开度 = W + 这个值。'
                         '固定爪基准下只要开度 > W + 贴面余量就够，'
                         '开大一点物块更容易进到两爪之间。')
    ap.add_argument('--face-clearance-mm', type=float, default=3.0,
                    help='下降时固定爪内侧面与物块那个面留的间隙。'
                         '合爪时活动爪会把物块推过这段距离、顶死在固定爪上'
                         '（自定心）。太小会刮到物块，太大会把物块推很远。')
    ap.add_argument('--squeeze-mm', type=float, default=1.5,
                    help='挤压量：合爪开度 = W - 这个值')
    ap.add_argument('--min-clearance-mm', type=float, default=5.0)
    ap.add_argument('--stride', type=int, default=6)
    ap.add_argument('--clear-stride', type=int, default=40)
    ap.add_argument('--rot-weight', type=float, default=1.0)
    ap.add_argument('--ik-seeds', type=int, default=14,
                    help='IK 多起点个数（含给定种子）')
    ap.add_argument('--ik-rng-seed', type=int, default=0)
    ap.add_argument('--sweep-yaw', action='store_true',
                    help='扫描 yaw 0..350°，看哪些朝向可达（诊断用）')
    ap.add_argument('--auto-yaw', action='store_true', default=True,
                    help='在 yaw + k*90° 里自动挑可达的那个（方形物块等价）')
    ap.add_argument('--no-auto-yaw', dest='auto_yaw', action='store_false')
    ap.add_argument('--execute', action='store_true',
                    help='真的发指令（默认只规划）')

    ap.add_argument('--seed', nargs=6, type=float, default=None,
                    help='IK 种子关节角；默认读 /joint_states')
    args = ap.parse_args()

    P = Planner(args)
    W = args.width_mm / 1000.0
    z_block = args.z if args.z is not None else args.table_z + args.height_mm / 2000.0

    print('=' * 74)
    print('  通用抓取规划')
    print('=' * 74)
    print(f'  物块中心   ({args.x*1000:+.1f}, {args.y*1000:+.1f}, '
          f'{z_block*1000:+.1f}) mm')
    print(f'  边长/高    {args.width_mm:.1f} / {args.height_mm:.1f} mm   '
          f'yaw {args.yaw_deg:+.1f}°')
    print(f'  桌面 z     {args.table_z*1000:+.2f} mm')

    # ---- 由模型反解开度对应的夹爪角 ----
    open_w = W + args.open_extra_mm / 1000.0
    grip_w = W - args.squeeze_mm / 1000.0
    a_open, c1 = P.m.angle_for_opening(open_w)
    a_grip, c2 = P.m.angle_for_opening(grip_w)
    print()
    print('  开度 ↔ 夹爪角（由网格反解，无硬编码）')
    print(f'    张爪 开度 {open_w*1000:5.1f}mm → {math.degrees(a_open):+6.2f}°'
          f'{"  ⚠️限幅" if c1 else ""}')
    print(f'    合爪 开度 {grip_w*1000:5.1f}mm → {math.degrees(a_grip):+6.2f}°'
          f'{"  ⚠️限幅" if c2 else ""}')

    # ---- 目标姿态：工具轴竖直朝下 + 爪口轴 = 物块 yaw ----
    yaw = math.radians(args.yaw_deg)
    X = np.array([math.cos(yaw), math.sin(yaw), 0.0])   # 爪口轴
    Z = np.array([0.0, 0.0, -1.0])                      # 工具轴朝下
    Y = np.cross(Z, X)
    R_t = np.column_stack([X, Y, Z])
    print()
    print('  目标姿态（工具轴竖直朝下，爪口轴对准物块 yaw）')
    print(f'    爪口轴 X = ({X[0]:+.3f}, {X[1]:+.3f}, {X[2]:+.3f})')
    print(f'    工具轴 Z = ({Z[0]:+.3f}, {Z[1]:+.3f}, {Z[2]:+.3f})')
    print(f'    det(R) = {np.linalg.det(R_t):+.4f}   '
          f'四元数 = {np.round(quat_from_R(R_t), 4).tolist()}')

    # ---- 种子 ----
    if args.seed:
        q_seed = list(args.seed)
    else:
        import rclpy
        import time
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        rclpy.init()
        node = Node('grasp_plan_seed')
        got = {}
        node.create_subscription(
            JointState, '/joint_states',
            lambda m: got.update(dict(zip(m.name, m.position)))
            if len(m.name) == len(m.position) else None, 10)
        t0 = time.time()
        while rclpy.ok() and len(got) < 6 and time.time() - t0 < 8:
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(got) < 6:
            print('❌ 读不到 /joint_states，且没给 --seed')
            return 1
        q_seed = [float(got[k]) for k in JOINTS]
        print(f'\n  种子（真机当前）= '
              f'{[round(v,3) for v in q_seed]}')

    z_block = args.z if args.z is not None else \
        args.table_z + args.height_mm / 2000.0
    z_app = z_block + args.approach_mm / 1000.0

    def pose_for(yaw_rad):
        X = np.array([math.cos(yaw_rad), math.sin(yaw_rad), 0.0])
        Z = np.array([0.0, 0.0, -1.0])
        Y = np.cross(Z, X)
        return np.column_stack([X, Y, Z])

    # ---- yaw 扫描（诊断）----
    if args.sweep_yaw:
        print()
        print('  yaw 可达性扫描（目标：接近点，位置 3 + 姿态 3）')
        print(f'  {"yaw(°)":>8}{"位置残差":>12}{"姿态残差":>12}  判定')
        print('  ' + '-' * 48)
        best = None
        for yd in range(0, 360, 15):
            R = pose_for(math.radians(yd))
            q, conv, ep, er = P.ik(list(q_seed),
                                   np.array([args.x, args.y, z_app], float), R)
            good = conv and ep < 2e-3 and er < math.radians(3)
            print(f'  {yd:>8}{ep*1000:>11.2f}mm{math.degrees(er):>11.2f}°  '
                  f'{"✅" if good else "❌"}')
            if good and (best is None or ep < best[0]):
                best = (ep, yd)
        if best:
            print(f'\n  → 最可达的 yaw ≈ {best[1]}°（残差 {best[0]*1000:.2f}mm）')
        else:
            print('\n  → 没有可达的 yaw。物块可能超出工作空间。')
        return 0

    # ---- 自动挑 yaw：方形物块在 yaw + k*90° 下等价 ----
    chosen_yaw = args.yaw_deg
    if args.auto_yaw:
        print()
        print('  自动挑朝向（方形物块在 yaw + k*90° 下等价）')
        candidates = [args.yaw_deg + 90.0 * k for k in range(4)]
        best = None
        for yd in candidates:
            R = pose_for(math.radians(yd))
            q, conv, ep, er = P.ik(list(q_seed),
                                   np.array([args.x, args.y, z_app], float), R)
            good = conv and ep < 2e-3 and er < math.radians(3)
            print(f'    yaw {yd:+7.1f}°  位置残差 {ep*1000:7.2f}mm  '
                  f'姿态残差 {math.degrees(er):6.2f}°  '
                  f'{"✅ 可用" if good else "❌"}')
            if good and (best is None or ep < best[0]):
                best = (ep, yd)
        if best:
            chosen_yaw = best[1]
            print(f'    → 选用 yaw = {chosen_yaw:+.1f}°')
        else:
            print('    → 四个朝向都不可达，仍用给定的 yaw 继续（会报错）')

    # ---- 用最终选定的 yaw 重建姿态 ----
    R_t = pose_for(math.radians(chosen_yaw))
    print()
    print(f'  最终目标姿态（工具轴竖直朝下，爪口轴 yaw = {chosen_yaw:+.1f}°）')
    print(f'    爪口轴 X = ({R_t[0,0]:+.3f}, {R_t[1,0]:+.3f}, {R_t[2,0]:+.3f})')
    print(f'    工具轴 Z = ({R_t[0,2]:+.3f}, {R_t[1,2]:+.3f}, {R_t[2,2]:+.3f})')
    print(f'    det(R) = {np.linalg.det(R_t):+.4f}   '
          f'四元数 = {np.round(quat_from_R(R_t), 4).tolist()}')

    # ---- 路点 ----
    #
    # 基准改成了**固定爪内侧面中心**（tcp_link 的 frame X = 0），所以
    # 目标不再是"物块中心"，而是"把固定爪内侧面送到物块那个面的位置"：
    #
    #     目标 = 物块中心 + (W/2 + 贴面余量) * x_hat
    #
    # x_hat 是爪口轴（frame X）在世界下的方向 = 物块 yaw 方向。
    # 下降时留 `face_clearance` 的间隙；合爪时活动爪把物块推过这段距离，
    # **顶死在固定爪这个硬基准上** —— 自定心，不受摩擦和初始偏差影响。
    # 这就是"固定爪贴一个垂直面、另一个爪再合上"的实现。
    off = W / 2.0 + args.face_clearance_mm / 1000.0
    yaw_r = math.radians(chosen_yaw)
    tx = args.x + off * math.cos(yaw_r)
    ty = args.y + off * math.sin(yaw_r)
    print()
    print(f'  固定爪基准：目标 = 物块中心 + ({W/2*1000:.1f} + '
          f'{args.face_clearance_mm:.1f})mm · x̂')
    print(f'    物块中心 ({args.x*1000:+.1f}, {args.y*1000:+.1f}) mm')
    print(f'    TCP 目标 ({tx*1000:+.1f}, {ty*1000:+.1f}) mm '
          f'（偏移 {off*1000:.1f}mm 沿 yaw {chosen_yaw:+.1f}°）')

    z_lift = z_block + args.lift_mm / 1000.0

    # **必须先抬离桌面再张爪。**
    # 实测踩到：当前姿态爪子正贴着板面（起点净空 -2.75mm），这时直接张爪，
    # 活动爪会往下甩（大开度下活动爪比固定爪低约 6mm），直接扎进桌子。
    # 所以插一个"0.抬离"路点：保持当前开度、把 TCP 竖直抬起来，再开爪。
    q_now = list(P._clamp(q_seed))
    p_now, _, _ = P.fk(q_now)
    wps = [
        ('0.抬离',      (p_now[0], p_now[1], p_now[2]
                         + args.lift_mm / 1000.0),                      None),
        ('1.张爪',      None,                                          a_open),
        ('2.到上空',    (tx, ty, z_app),                                None),
        ('3.垂直下降',  (tx, ty, z_block),                              None),
        ('4.合爪',      (tx, ty, z_block),                              a_grip),
        ('5.抬起',      (tx, ty, z_lift),                               None),
    ]

    print()
    print('-' * 74)
    print(f'  {"路点":<12}{"位置残差":>10}{"姿态残差":>10}{"净空":>10}  '
          f'{"受限部件":<26} 判定')
    print('-' * 74)

    q = list(P._clamp(q_seed))   # 种子必须先夹进限位：
    # 机械臂失力下垂时关节会停到软限位之外（实测 shoulder_lift=-1.812
    # 而限位是 [-1.746,+1.746]），拿这种种子当"当前姿态"会一路误报触限位。
    plan = []
    ok_all = True
    for name, p_t, gj in wps:
        if p_t is not None:
            q, conv, ep, er = P.ik(q, np.array(p_t, float), R_t)
        else:
            conv, ep, er = True, 0.0, 0.0
        if gj is not None:
            q = list(q)
            q[5] = float(gj)
        clr, link, low = P.clearance(q)
        bad = P.check_limits(q)
        good = (conv and ep < 2e-3 and er < math.radians(3.0)
                and clr >= args.min_clearance_mm / 1000.0 and not bad)
        ok_all &= good
        verdict = '✅' if good else '❌'
        if not conv:
            verdict += ' 未收敛'
        if bad:
            verdict += ' 触限位'
        if clr < args.min_clearance_mm / 1000.0:
            verdict += ' 净空不足'
        print(f'  {name:<12}{ep*1000:>9.2f}mm{math.degrees(er):>9.2f}°'
              f'{clr*1000:>9.2f}mm  {link:<26} {verdict}')
        plan.append({'name': name, 'q': [float(v) for v in q],
                     'pos': p_t, 'gripper': gj,
                     'ep_mm': ep * 1000, 'er_deg': math.degrees(er),
                     'clearance_mm': clr * 1000, 'limits': bad,
                     'target_gripper': gj})

    print('-' * 74)
    print()
    print('  路径净空检查（相邻路点之间的关节空间插值，取最危险处）')
    print(f'  {"区间":<22}{"最小净空":>12}{"位置":>8}  受限部件')
    print('  ' + '-' * 68)
    path_ok = True
    for w0, w1 in zip(plan, plan[1:]):
        mn, lk, t = P.path_check(w0['q'], w1['q'])
        good = mn >= args.min_clearance_mm / 1000.0
        path_ok &= good
        print(f'  {w0["name"]+" → "+w1["name"]:<22}{mn*1000:>11.2f}mm'
              f'{t*100:>7.0f}%  {lk}')
    if not path_ok:
        print()
        print('  ⚠️ 有区间中途净空不足 —— 两点都合法但**中间会刮到桌子**。')
        print('     这是关节空间直插的固有问题：需要先退到一个安全中间位姿。')
    ok_all &= path_ok
    print()
    print('  关节轨迹（rad）：')
    for w in plan:
        print(f'    {w["name"]:<12}'
              f'{["%+.3f" % v for v in w["q"]]}')

    if not ok_all:
        print()
        print('  ⚠️ 有路点未通过检查 —— 不要执行。')
        print('     常见原因：物块太远/太低，或该 yaw 下臂够不到。')
        print('     可以试：调 --yaw-deg、把物块挪近、或减小 --approach-mm。')

    if args.execute:
        if not ok_all:
            print('\n❌ 检查未通过，拒绝执行')
            return 1
        print('\n=== 执行（--execute）===')
        print('   注意：需要驱动处于 allow_motion=true 且已使能扭矩。')
        return run_plan(plan, args)
    print('\n  （仅规划。加 --execute 才会真动。）')
    return 0 if ok_all else 1


def run_plan(plan, args):
    """按关节轨迹逐路点执行，每步之间用限速插值。"""
    import time

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_srvs.srv import SetBool

    rclpy.init()
    node = Node('grasp_exec')
    pub = node.create_publisher(JointState, '/joint_commands_raw', 10)
    cur = {}

    def on_js(m):
        if len(m.name) == len(m.position):
            cur.clear()
            cur.update(dict(zip(m.name, m.position)))

    node.create_subscription(JointState, '/joint_states', on_js, 10)
    cli = node.create_client(SetBool, '/arm/enable')
    if not cli.wait_for_service(timeout_sec=5.0):
        print('❌ /arm/enable 不可用')
        return 1
    req = SetBool.Request()
    req.data = True
    fut = cli.call_async(req)
    t0 = time.time()
    while rclpy.ok() and not fut.done() and time.time() - t0 < 5:
        rclpy.spin_once(node, timeout_sec=0.05)
    print('  使能:', fut.result().message if fut.done() else '(无响应)')

    def spin(sec):
        e = time.time() + sec
        while rclpy.ok() and time.time() < e:
            rclpy.spin_once(node, timeout_sec=0.02)

    spin(1.0)
    if len(cur) < 6:
        print('❌ 读不到关节反馈')
        return 1
    q_cur = np.array([cur[k] for k in JOINTS], float)

    rate = 30.0
    for w in plan:
        q_t = np.array(w['q'], float)
        n = max(1, int(np.linalg.norm(q_t - q_cur) / 0.01))
        print(f'  → {w["name"]}  ({n} 步)')
        for k in range(1, n + 1):
            q_i = q_cur + (q_t - q_cur) * (k / n)
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = list(JOINTS)
            msg.position = [float(v) for v in q_i]
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.005)
            time.sleep(1.0 / rate)
        q_cur = q_t
        spin(0.6)

    print('  轨迹执行完毕。松开扭矩前请确认已夹稳。')
    try:
        node.destroy_node()
    except Exception:  # noqa: BLE001
        pass
    if rclpy.ok():
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
