#!/usr/bin/env python3
"""自动抓取（不硬编码任何示教位姿）。

姿态自己算
----------
方块与棋盘对齐，所以夹爪的**合爪方向**取棋盘轴 ey（朝机械臂一侧），
**工具轴竖直向下**。R = [x=ey | y=z×x | z=向下]。这两条已用示范位姿交叉
验证过（工具z偏竖直 5.2°、工具x与ey差 2.4°）。

夹持力闭环
----------
合爪不再固定角度：**边合边读载荷**，达到目标载荷（默认 15%）即停；
若到极限角度仍不到目标载荷 -> 判定没夹住，张开、报告失败。
抬升时持续监控载荷，掉到 3% 以下立即停止（避免甩飞）。

收尾
----
无论成败：张开爪子（若没夹住）或**保持夹持角**（若夹住）-> 自动回折叠位。
经用户示教的中间位 READY，路径更可控。
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
from scipy.optimize import least_squares
import yaml
from pathlib import Path

sys.path.insert(0, '/home/ros/legacy/arm/arm-final/ros2_ws/src/so101_bringup')
sys.path.insert(0, os.path.expanduser(
    '~/QianLi/qianli_ws/src/qianli_vision/scripts'))
from so101_bringup.servo_protocol import FeetechSerialBus
from gripper_model import GripperModel, JOINTS, FLANGE_LINK

CONFIG = os.path.expanduser(
    '~/legacy/arm/arm-final/ros2_ws/install/so101_bringup/share/so101_bringup'
    '/config/driver_params.yaml')
CFG = os.path.expanduser('~/QianLi/qianli_ws/config')
TABLE_Z = -0.06485
BOARD_MM = 0.5
DOWN = np.array([0.0, 0.0, -1.0])


class SkipGrasp(Exception):
    """--return-only：跳过抓取，只跑返回流程。"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, required=False, default=0.2)
    ap.add_argument('--y', type=float, required=False, default=-0.1)
    ap.add_argument('--cube-mm', type=float, default=40.0, help='方块边长')
    ap.add_argument('--gap-mm', type=float, default=1.5,
                    help='固定爪与方块侧面的间隙（示范位实测约 0.6mm）')
    ap.add_argument('--grasp-depth-mm', type=float, default=-50.0,
                    help='爪尖抓取高度（方块中心约 -44.9mm）')
    ap.add_argument('--target-load', type=float, default=15.0)
    ap.add_argument('--extra-squeeze-rad', type=float, default=0.02,
                    help='达到目标载荷后再按命令角多压的弧度（越大夹越紧）')
    ap.add_argument('--y-offset-mm', type=float, default=0.0,
                    help='沿棋盘 ey 方向的额外偏置(mm)。负值=固定爪往 −y 收')
    ap.add_argument('--min-load', type=float, default=6.0)
    ap.add_argument('--max-squeeze-rad', type=float, default=0.20)
    ap.add_argument('--lift-mm', type=float, default=40.0)
    ap.add_argument('--hover-mm', type=float, default=50.0)
    ap.add_argument('--place-x', type=float, default=None,
                    help='放置点 base x（默认=棋盘 (0,0) 原点）')
    ap.add_argument('--place-y', type=float, default=None)
    ap.add_argument('--place-z-mm', type=float, default=-52.0,
                    help='放置时爪尖高度')
    ap.add_argument('--place-above-mm', type=float, default=45.0,
                    help='放置点上方多少 mm 处先水平过去')
    ap.add_argument('--return-only', action='store_true',
                    help='只跑返回流程：回中间位 -> 松手 -> 腕部摆正 -> 回折叠位')
    ap.add_argument('--report', default='/tmp/grasp_auto.json')
    a = ap.parse_args()

    cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
    zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
    lo = (np.array(cfg['raw_min']) - zero) * direction * 2 * math.pi / 4096
    hi = (np.array(cfg['raw_max']) - zero) * direction * 2 * math.pi / 4096
    lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)

    fr = json.load(open(os.path.join(CFG, 'board_frame.json')))
    A = np.array(fr['affine'])          # 3x2: 前两行是 2x2 矩阵，第三行是平移
    ey = A[:2, 1] / np.linalg.norm(A[:2, 1])
    # 放置点默认 = 棋盘 (0,0) 原点（affine 的平移项就是 grid(0,0) 的 base 坐标）
    if a.place_x is None:
        a.place_x = float(A[2][0])
    if a.place_y is None:
        a.place_y = float(A[2][1])
    ey3 = np.array([ey[0], ey[1], 0.0])
    xax_w = ey3.copy()                       # 合爪方向 = 棋盘 ey（朝臂侧）
    zax_w = DOWN.copy()                      # 工具轴竖直向下
    yax_w = np.cross(zax_w, xax_w)
    R_des = np.column_stack([xax_w, yax_w, zax_w])

    half = a.cube_mm / 2000.0
    cube_xy = np.array([a.x, a.y])
    tcp_xy = cube_xy + (half + a.gap_mm / 1000.0) * ey
    tip_z = a.grasp_depth_mm / 1000.0
    # 尖端在工具系 z=+6.3mm 处（工具朝下时即在 TCP 下方 6.3mm）
    tcp_z = tip_z + 0.0063
    tgt = np.array([tcp_xy[0], tcp_xy[1], tcp_z])
    hover = np.array([tcp_xy[0], tcp_xy[1],
                      tcp_z + a.hover_mm / 1000.0])

    bus = FeetechSerialBus('/dev/ttyACM0', timeout_s=0.15)
    model = GripperModel(stride=8)
    T0 = model.solve(dict(zip(JOINTS, np.zeros(6))))
    F0, G0 = T0['gripper_frame_link'], T0[FLANGE_LINK]
    w0 = (G0[:3, :3] @ model.parts[FLANGE_LINK].T).T + G0[:3, 3]
    pf = (F0[:3, :3].T @ (w0 - F0[:3, 3]).T).T
    inner = pf[np.abs(pf[:, 0]) < 0.004]
    p_fix = inner[int(np.argmax(inner[:, 2]))]

    out = {'cube_xy': cube_xy.tolist(), 'tcp_target': tgt.tolist(),
           'grasp_dir_ey': ey.tolist(), 'p_fix_mm': (p_fix * 1000).tolist()}

    def rd(fn, n=8, tag=''):
        last = None
        for _ in range(n):
            try:
                return fn()
            except Exception as exc:
                last = exc
                time.sleep(0.2)
        raise RuntimeError(f'{tag} 连续失败: {last}')

    def read():
        return (rd(bus.read_positions, tag='读位置') - zero) \
            * direction * 2 * math.pi / 4096

    def write(q):
        q = np.clip(np.asarray(q, float), lo, hi)
        raw = np.rint(zero + q * direction * 4096 / (2 * math.pi)).astype(int)
        rd(lambda: bus.write_positions(raw.tolist()), tag='写位置')

    def load():
        try:
            return rd(bus.read_gripper_load, tag='读载荷')[0]
        except Exception:
            return 0.0

    def fk(q):
        return model.solve(dict(zip(JOINTS, q)))

    def tip_of(q):
        F = fk(q)['gripper_frame_link']
        return F[:3, 3] + F[:3, :3] @ p_fix

    def lowest(q):
        return model.lowest_over_all(dict(zip(JOINTS, q)))

    def jaw_clearances(q, cube_c, half_m):
        """用 mesh 算固定爪/活动爪 到方块表面 的穿透与间隙。"""
        T = fk(q)
        res = {}
        for link in ('gripper_link', 'moving_jaw_so101_v1_link'):
            G = T[link]
            W = (G[:3, :3] @ model.parts[link].T).T + G[:3, 3]
            d = W - cube_c
            inside = np.all(np.abs(d) <= half_m, axis=1)
            pen = 0.0
            if inside.any():
                pen = float(np.max(np.min(half_m - np.abs(d[inside]), axis=1)))
            out = ~inside
            gap = float(np.min(np.linalg.norm(
                np.maximum(np.abs(d[out]) - half_m, 0), axis=1))) \
                if out.any() else 0.0
            res[link] = (pen, gap)
        return res

    def solve(target, grip, ref, w_axis=0.6):
        def residual(arm):
            q = np.r_[arm, grip]
            F = fk(q)['gripper_frame_link']
            return np.r_[F[:3, 3] - target,
                         w_axis * (F[:3, :3] - R_des).ravel()]
        best, bc = None, None
        rng = np.random.default_rng(0)
        seeds = [ref] + [lo[:5] + rng.random(5) * (hi[:5] - lo[:5])
                         for _ in range(8)]
        for s in seeds:
            ss = np.clip(s[:5], lo[:5] + 1e-6, hi[:5] - 1e-6)
            sol = least_squares(residual, ss, bounds=(lo[:5], hi[:5]),
                                max_nfev=400)
            c = np.r_[sol.x, grip]
            e = float(np.linalg.norm(fk(c)['gripper_frame_link'][:3, 3] - target))
            if bc is None or e < bc:
                best, bc = c, e
        return best, bc

    def move(goal, segs=10, tol=0.05, wait=0.8):
        start = read()
        for i in range(1, segs + 1):
            seg = start + (goal - start) * i / segs
            t0 = time.monotonic()
            while time.monotonic() - t0 < wait:
                write(seg)
                time.sleep(0.08)
                if np.max(np.abs(read() - seg)) < tol:
                    break

    held = False
    # ---- 用 mesh 自动定 y 偏置：让方块在爪口里居中（不靠手调）----
    cube_c = np.array([a.x, a.y, TABLE_Z + half])
    q_probe, e_probe = solve(tgt, 0.58, np.zeros(6))
    cl0 = jaw_clearances(q_probe, cube_c, half)
    gf = cl0['gripper_link'][1]
    gm = cl0['moving_jaw_so101_v1_link'][1]
    pen0 = max(cl0['gripper_link'][0], cl0['moving_jaw_so101_v1_link'][0])
    delta = (gm - gf) / 2.0
    print(f'几何探测(偏置 {a.y_offset_mm:+.1f}mm): 固定爪 穿透 '
          f'{cl0["gripper_link"][0]*1000:.2f}mm/离面 {gf*1000:.2f}mm  '
          f'活动爪 离面 {gm*1000:.2f}mm')
    # 自动居中(让固定爪/活动爪到方块等距) + 用户额外偏置(负=往 -y 收)
    shift = delta + a.y_offset_mm / 1000.0
    if abs(shift) > 0.0002:
        print(f'  偏置 = 自动居中 {delta*1000:+.2f}mm '
              f'+ 额外 {a.y_offset_mm:+.1f}mm = {shift*1000:+.2f}mm')
        tgt = tgt + np.array([shift * ey[0], shift * ey[1], 0.0])
        hover = hover + np.array([shift * ey[0], shift * ey[1], 0.0])
    q_chk, _ = solve(tgt, 0.58, np.zeros(6))
    cl2 = jaw_clearances(q_chk, cube_c, half)
    print(f'  ✅ 最终几何: 固定爪 穿透 {cl2["gripper_link"][0]*1000:.2f}mm / '
          f'离面 {cl2["gripper_link"][1]*1000:.2f}mm   活动爪 穿透 '
          f'{cl2["moving_jaw_so101_v1_link"][0]*1000:.2f}mm / 离面 '
          f'{cl2["moving_jaw_so101_v1_link"][1]*1000:.2f}mm')
    out['jaw_geometry'] = {k: {'pen_mm': v[0] * 1000, 'gap_mm': v[1] * 1000}
                           for k, v in cl2.items()}
    out['auto_offset_mm'] = float(delta * 1000)
    out['total_offset_mm'] = float(shift * 1000)
    out['probe_pen_mm'] = float(pen0 * 1000)

    # ---- 快速/流畅动作原语 ----
    def open_jaws():
        """平滑开爪：时间基准余弦剖面（与 smooth_move 同款）。"""
        q = read()
        g0 = q[5]
        g1 = 0.58
        if abs(g1 - g0) < 0.005:
            return
        dur = 1.2

        def ramp_oj(s):
            # 单边加速-减速的简单正弦：s in [0,1] -> 位移
            return 0.5 * (1 - np.cos(np.pi * s))

        t0 = time.monotonic()
        while True:
            s = (time.monotonic() - t0) / dur
            if s >= 1.0:
                break
            qq = q.copy()
            qq[5] = g0 + (g1 - g0) * float(ramp_oj(s))
            write(qq)
            time.sleep(0.03)
        qq = q.copy()
        qq[5] = g1
        write(qq)

    def smooth_move(goal, check=False, tag=''):
        """梯形速度剖面**时间基准**流式下发。

        丝滑三要素：
          1. 梯形剖面（余弦加速/减速，f=0.13 平缓起步无蹬脚）
          2. 目标按**真实流逝时间**插值（不是拍号）：
             某拍被调度拖慢/抢CPU时，下一拍按时间重算 -> 无位置跳变，
             轨迹平滑伸展，免疫"值守时一卡一卡"（旧实现按拍号会跳变）
          3. 结束前补拍直至到位
        """
        start = read()
        if check:
            worst = None
            for i in range(1, 15):
                q = start + (goal - start) * i / 14
                low, lk = lowest(q)
                if worst is None or low[2] < worst[0]:
                    worst = (low[2], lk)
            print(f'  [{tag}] 路径最低 {worst[1]} z={worst[0]*1000:+.2f}mm')
            if worst[0] < TABLE_Z - 0.003:
                print(f'  [{tag}] 路径低于桌面，跳过')
                return False
        dist = float(np.max(np.abs(goal - start)))
        if dist < 0.002:
            write(goal)
            return True
        duration = np.clip(dist / 0.11, 1.1, 5.5)   # 巡航 ~0.11rad/s
        f = 0.13                                    # 加速/减速段占比
        NSTREAM = 0.012                             # 流式检查周期(秒)

        def ramp(s):
            """归一化时间 s -> [0,1] 位移（梯形余弦速度剖面的积分，已归一化）。

            速度: [0,f) 余弦爬升0->1; [f,1-f] 巡航1; (1-f,1] 余弦降1->0
            位移 = 积分 / (1-f)，保证 ramp(1)=1、处处连续。"""
            if s < f:
                return (s / 2 - f / (2 * np.pi) * np.sin(np.pi * s / f)) \
                    / (1 - f)
            if s <= 1 - f:
                return (s - f / 2) / (1 - f)
            u = s - (1 - f)
            return (1 - 1.5 * f + u / 2
                    + f / (2 * np.pi) * np.sin(np.pi * u / f)) / (1 - f)

        t0 = time.monotonic()
        while True:
            s = (time.monotonic() - t0) / duration
            if s >= 1.0:
                break
            write(start + (goal - start) * float(ramp(s)))
            time.sleep(NSTREAM)
        for _ in range(20):
            if np.max(np.abs(read() - goal)) < 0.05:
                break
            write(goal)
            time.sleep(0.03)
        return True

    released_here = False
    try:
        print('力矩:', rd(lambda: (bus.set_torque(True),
                                   bus.read_torque_states())[1], tag='使能'))
        if a.return_only:
            raise SkipGrasp()
        time.sleep(0.5)
        q_now = read()

        # 途经 READY 中间位
        legs = []
        rp = os.path.join(CFG, 'ready_pose.json')
        if os.path.exists(rp):
            rr = json.load(open(rp))
            legs.append(('READY', np.clip((np.array(rr['raw_exec']) - zero)
                                          * direction * 2 * math.pi / 4096,
                                          lo, hi)))
        q_hi, e_hi = solve(hover, 0.58, q_now)
        print(f'悬停位 IK 误差 {e_hi*1000:.1f}mm  '
              f'TCP 目标 {np.round(hover,4).tolist()}')
        legs.append(('HOVER(目标上方)', q_hi))
        cur = q_now
        for name, q_leg in legs:
            # 用时间基准平滑流式（与摆动测试一致的丝滑），含净空检查
            ok = smooth_move(q_leg, check=True, tag=name)
            if not ok:
                out['error'] = f'path below table ({name})'
                print(f'❌ [{name}] 路径低于桌面，中止')
                return
            cur = q_leg
            if name == 'READY':
                # 出勤时在**中间位**就把爪子打开（用户要求）
                open_jaws()
                print('  [READY] 已开爪')
        # 张开爪子到最大
        q = read()
        for g in np.linspace(q[5], 0.58, 12):
            qq = q.copy()
            qq[5] = g
            write(qq)
            time.sleep(0.08)

        # 竖直下探到抓取位（保持姿态）
        q_lo, e_lo = solve(tgt, 0.58, read())
        print(f'抓取位 IK 误差 {e_lo*1000:.1f}mm  '
              f'TCP 目标 {np.round(tgt,4).tolist()}  工具系尖端 '
              f'{np.round(p_fix*1000,1).tolist()}mm')
        out['ik_hover_mm'] = float(e_hi * 1000)
        out['ik_grasp_mm'] = float(e_lo * 1000)
        if e_lo > 0.006:
            out['error'] = 'grasp IK unreachable'
            print('❌ 抓取位 IK 够不到')
            return
        # 平滑下探到抓取位（时间基准正弦剖面，每段查净空）
        q_from = read()
        t0 = time.monotonic()
        dur = 2.4
        while True:
            s = (time.monotonic() - t0) / dur
            if s >= 1.0:
                break
            si = 0.5 * (1 - np.cos(np.pi * s))   # 快-慢：接近底部变缓
            seg = q_from + (q_lo - q_from) * float(si)
            low, lk = lowest(seg)
            if low[2] < TABLE_Z - 0.002:
                print(f'  下探最低 {lk} z={low[2]*1000:+.2f}mm，停')
                break
            write(seg)
            time.sleep(0.03)
        p = tip_of(read())
        print(f'落到底 爪尖 ({p[0]:.4f},{p[1]:.4f},{p[2]*1000:+.1f}mm)  '
              f'离桌面 {(p[2]-TABLE_Z)*1000:.1f}mm')
        out['tip_bottom_m'] = p.tolist()
        # 下探到位拍照（供核对固定爪相对方块的位置）
        try:
            import cv2
            cap = cv2.VideoCapture(0)
            img = None
            for _ in range(8):
                ok2, f2 = cap.read()
                if ok2:
                    img = f2
            cap.release()
            if img is not None:
                cv2.imwrite('/tmp/auto_descended.jpg', img)
                print('  📷 /tmp/auto_descended.jpg')
        except Exception as exc:
            print('  拍照失败:', exc)

        # ---- 合爪：载荷闭环（时间基准慢速平滑收拢 + 每轮查载荷）----
        q0 = read()
        load0 = load()
        g_open = q0[5]
        g_end = max(a.max_squeeze_rad, lo[5])
        pct = load0
        dur_close = 2.5
        seed = q0.copy()
        sq = q0.copy()
        t0 = time.monotonic()
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            s = (time.monotonic() - t0) / dur_close
            if s >= 1.0:
                break
            # 先快后缓的正弦剖面：靠近方块时变慢，温柔接触
            si = 0.5 * (1 - np.cos(np.pi * s))
            sq = seed.copy()
            sq[5] = g_open + (g_end - g_open) * float(si)
            low, lk = lowest(sq)
            if low[2] < TABLE_Z - 0.002:
                print(f'  合爪触底({lk})，停')
                break
            write(sq)
            seed = sq
            time.sleep(0.04)
            pct = load()
            if pct >= a.target_load:
                break
        out.update(contact_load_pct=float(pct), squeeze_rad=float(sq[5]))
        print(f'合爪: 载荷 {pct:.1f}% (目标 {a.target_load}%)  '
              f'角度 {sq[5]:.4f}rad')
        held = pct >= a.min_load
        if not held:
            print(f'⚠ 载荷 {pct:.1f}% < 最小 {a.min_load}% —— 判定没夹住')
        else:
            # 关键：用**命令角**继续压，不能用实测角。
            # 方块会把爪子弹开，实测角比命令角大（这正是"夹住了"的表现）；
            # 若按实测角再减，反而会把爪子张开、松掉方块。
            sq[5] = max(lo[5], sq[5] - a.extra_squeeze_rad)
            write(sq)
            time.sleep(0.5)
            pct2 = load()
            print(f'  继续压到命令角 {sq[5]:.4f}rad，载荷 {pct2:.1f}%'
                  f'（实测角 {read()[5]:.4f}rad）')
            if pct2 < a.min_load:
                print(f'⚠ 加压后载荷只有 {pct2:.1f}%，判定没夹住')
                held = False
            else:
                held = True

        # ---- 抬升：载荷监控（时间基准流式 + 每轮查载荷）----
        if held:
            F = fk(read())['gripper_frame_link']
            p0 = F[:3, 3].copy()
            lifted = 0.0
            dur = 3.0                                   # 总抬升时长
            seed = read()
            t0 = time.monotonic()
            while True:
                s = (time.monotonic() - t0) / dur
                if s >= 1.0:
                    break
                # 正弦剖面：先快后缓，避免末端惯性
                si = 0.5 * (1 - np.cos(np.pi * s))
                t2 = p0 + np.array([0, 0, a.lift_mm / 1000 * si])
                q_c, e = solve(t2, sq[5], seed)
                if e > 0.006:
                    break
                write(q_c)
                seed = q_c
                time.sleep(0.06)
                cur_load = load()
                lifted = float((fk(read())['gripper_frame_link'][2, 3]
                                - p0[2]) * 1000)
                print(f'  抬升 {lifted:5.1f}mm 载荷 {cur_load:.1f}%')
                if cur_load < 3.0:
                    print('  载荷掉了，停止抬升')
                    break
            out['lift_mm'] = lifted
            out['load_after_lift'] = load()
            held = out['load_after_lift'] >= 3.0
            print(f'抬升 {lifted:.1f}mm 保持载荷 {out["load_after_lift"]:.1f}%'
                  f'  {"✅ 夹住" if held else "❌ 滑脱"}')

        # ---- 放置：统一放到棋盘 (0,0) 位置，然后归位 ----
        if held:
            t_place = time.monotonic()
            gp = float(sq[5])
            # 先到放置点上方（净空检查）
            ph = np.array([a.place_x, a.place_y,
                           a.place_z_mm / 1000.0 + 0.0063
                           + a.place_above_mm / 1000.0])
            q_ph, e1 = solve(ph, gp, read())
            print(f'  [放置] 去 (0,0) 上方 IK 误差 {e1*1000:.1f}mm  '
                  f'TCP {np.round(ph,4).tolist()}')
            smooth_move(q_ph, check=True, tag='去放置点')
            # 下降到放置高度
            pl = np.array([a.place_x, a.place_y,
                           a.place_z_mm / 1000.0 + 0.0063])
            q_pl, e2 = solve(pl, gp, read())
            print(f'  [放置] 下降 IK 误差 {e2*1000:.1f}mm')
            smooth_move(q_pl)
            open_jaws()
            released_here = True
            out['placed_at_00'] = True
            print(f'  ✅ 已放到棋盘 (0,0) 位置并松爪 '
                  f'({time.monotonic()-t_place:.1f}s)')
    except SkipGrasp:
        print('=== 跳过抓取，直接跑返回流程 ===')
    except BaseException as exc:
        out['error'] = str(exc)
        print(f'STOP: {exc}')
    finally:
        out['held'] = bool(held)
        try:
            hm = json.load(open(os.path.join(CFG, 'home_pose.json')))
            q_home = np.clip((np.array(hm['raw_exec']) - zero) * direction
                             * 2 * math.pi / 4096, lo, hi)
            rp = os.path.join(CFG, 'ready_pose.json')
            q_ready = None
            if os.path.exists(rp):
                q_ready = np.clip((np.array(json.load(open(rp))['raw_exec'])
                                   - zero) * direction * 2 * math.pi / 4096,
                                  lo, hi)

            def safe_move(goal, tag):
                # 用时间基准平滑流式 + 净空检查（与出勤大摆动一致）
                return smooth_move(goal, check=True, tag=tag)

            def open_jaws_local():
                open_jaws()

            # ① 先回中间位（若还夹着方块则夹着过去）
            if q_ready is not None:
                safe_move(q_ready, '回中间位')
            # ② 松手：若已在 (0,0) 放过，这里不再放
            if not released_here:
                open_jaws_local()
                print('  已在中间位松手（张开爪子放下方块）')
                out['released_at_ready'] = True
            else:
                print('  （方块已在 (0,0) 位置放下，这里只做腕部摆正）')
            # ③ 腕部摆正：TCP 位置不变，把工具轴转成竖直向下
            cur = read()
            p_now = fk(cur)['gripper_frame_link'][:3, 3].copy()

            def res_ori(arm):
                F = fk(np.r_[arm, cur[5]])['gripper_frame_link']
                return np.r_[F[:3, 3] - p_now,
                             0.8 * (F[:3, :3] - R_des).ravel()]
            sol = least_squares(res_ori,
                                np.clip(cur[:5], lo[:5] + 1e-6, hi[:5] - 1e-6),
                                bounds=(lo[:5], hi[:5]), max_nfev=300)
            q_str = np.r_[sol.x, cur[5]]
            Fs = fk(q_str)['gripper_frame_link']
            zt = Fs[:3, :3] @ np.array([0.0, 0.0, 1.0])
            tilt = math.degrees(math.acos(float(np.clip(-zt[2], -1, 1))))
            print(f'  腕部摆正: 工具轴偏离竖直 {tilt:.1f}°  '
                  f'位置偏差 {np.linalg.norm(Fs[:3,3]-p_now)*1000:.1f}mm')
            if np.linalg.norm(cur[:5] - q_str[:5]) > 0.02:
                safe_move(q_str, '腕部摆正')
            out['wrist_straight_deg'] = float(tilt)
            # ④ 回折叠位（爪子保持张开）
            q_ret = q_home.copy()
            q_ret[5] = 0.58
            safe_move(q_ret, '回折叠位')
            e = read()
            print(f'已自动归位 raw {np.round(e,4).tolist()}  '
                  f'夹爪 {e[5]:.4f}rad 载荷 {load():.1f}%')
            out['returned_home'] = True
            out['load_at_home'] = load()
        except BaseException as exc:
            print('归位失败:', exc)
            out['returned_home'] = False
        Path(a.report).write_text(json.dumps(out, indent=2, ensure_ascii=False))
        bus.close()
        print(json.dumps(out, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
