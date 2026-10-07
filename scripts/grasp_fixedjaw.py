#!/usr/bin/env python3
"""解耦抓取：直接控制"固定爪内侧尖端"位置。

步骤（按用户要求解耦）：
  1) 把固定爪内侧尖端移到"方块右侧面正上方"
  2) 打开爪子
  3) 数值竖直落下到抓取高度
  4) 合上爪子 -> 找接触
  5) 抬起

为什么这样更稳：固定爪是刚性基准，合爪时活动爪从另一侧把方块推向固定爪，
对姿态误差不敏感（不需要把 TCP 精确对准方块中心）。
"""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np
from scipy.optimize import least_squares
import yaml

sys.path.insert(0, '/home/ros/legacy/arm/arm-final/ros2_ws/src/so101_bringup')
sys.path.insert(0, '/home/ros/QianLi/qianli_ws/src/qianli_vision/scripts')
from so101_bringup.servo_protocol import FeetechSerialBus
from gripper_model import GripperModel, JOINTS, FLANGE_LINK

CONFIG = ('/home/ros/legacy/arm/arm-final/ros2_ws/install/so101_bringup/share/'
          'so101_bringup/config/driver_params.yaml')
TABLE_Z = -0.06909
DOWN = np.array([0.0, 0.0, -1.0])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--x', type=float, required=True, help='方块中心 x')
    ap.add_argument('--y', type=float, required=True)
    ap.add_argument('--right', default=None,
                    help='方块"右侧"方向 (ax,ay)，缺省用棋盘 x 轴')
    ap.add_argument('--half-mm', type=float, default=20.0, help='方块半宽')
    ap.add_argument('--tip-z-mm', type=float, default=-58.0, help='抓取时爪尖高度')
    ap.add_argument('--hover-mm', type=float, default=50.0)
    ap.add_argument('--open-rad', type=float, default=0.58)
    ap.add_argument('--close-min-rad', type=float, default=0.16)
    ap.add_argument('--load-pct', type=float, default=8.0)
    ap.add_argument('--lift-mm', type=float, default=40.0)
    ap.add_argument('--execute', action='store_true')
    ap.add_argument('--report', default='/tmp/grasp_fixedjaw.json')
    a = ap.parse_args()

    # 方块"右侧"方向：棋盘 x 轴在 base 下的方向
    if a.right:
        rd = np.array([float(v) for v in a.right.split(',')])
    else:
        rd = np.array([0.111, -0.994])
    rd = rd / np.linalg.norm(rd)

    cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
    zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
    lo = (np.array(cfg['raw_min']) - zero) * direction * 2 * math.pi / 4096
    hi = (np.array(cfg['raw_max']) - zero) * direction * 2 * math.pi / 4096
    lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)

    # 与舵机自身 EEPROM 限位取交集（driver 限位有几处超出舵机自身限位，
    # 打进去会触发 0x02 角度限位错误并锁存 + 切断力矩）
    SAFE = os.path.expanduser('~/QianLi/qianli_ws/config/safe_limits.json')
    if os.path.exists(SAFE):
        sl = json.load(open(SAFE))
        slo, shi = np.array(sl['rad_lo']), np.array(sl['rad_hi'])
        n_before = int(np.sum(lo < slo - 1e-9) + np.sum(hi > shi + 1e-9))
        lo, hi = np.maximum(lo, slo), np.minimum(hi, shi)
        print(f'已套用舵机安全限位（收紧了 {n_before} 处）')

    bus = FeetechSerialBus('/dev/ttyACM0', timeout_s=0.08)
    model = GripperModel(stride=8)
    out = {'mode': 'execute' if a.execute else 'plan'}

    def read():
        return (np.array(bus.read_positions()) - zero) * direction * 2 * math.pi / 4096

    def write(q):
        if not np.all(np.isfinite(q)):
            raise ValueError('command not finite')
        # 实测标定下限略保守（shoulder_lift 实测可到 raw 822，下限按 842 算），
        # 所以小幅越界直接钳位，只有大幅越界才拒绝。
        over = float(np.max(np.maximum(lo - np.asarray(q), np.asarray(q) - hi)))
        if over > 0.25:
            raise ValueError(f'command out of measured limits (超出 {over:.3f} rad)')
        qc = np.clip(q, lo, hi)
        bus.write_positions(np.rint(zero + qc * direction * 4096 /
                                    (2 * math.pi)).astype(int).tolist())

    def fk(q):
        return model.solve(dict(zip(JOINTS, q)))

    # ---- 固定爪"内侧尖端"在 gripper_frame 中的位置 ----
    T0 = fk(np.zeros(6))
    F0, G0 = T0['gripper_frame_link'], T0[FLANGE_LINK]
    fixed_pts = model.parts[FLANGE_LINK]
    w0 = (G0[:3, :3] @ fixed_pts.T).T + G0[:3, 3]
    pf = (F0[:3, :3].T @ (w0 - F0[:3, 3]).T).T       # 固定爪在工具系
    print(f'固定爪在工具系: x {pf[:,0].min()*1000:+.1f}..{pf[:,0].max()*1000:+.1f}  '
          f'z {pf[:,2].min()*1000:+.1f}..{pf[:,2].max()*1000:+.1f} mm')
    # 内侧面(x≈0) 且最深(z 最小) 的点 = 内侧尖端
    inner = pf[np.abs(pf[:, 0]) < 0.004]
    if len(inner) == 0:
        inner = pf[np.argsort(np.abs(pf[:, 0]))[:max(20, len(pf)//50)]]
    p_fix = inner[int(np.argmin(inner[:, 2]))]
    print(f'固定爪内侧尖端(工具系) = {np.round(p_fix*1000,1).tolist()} mm')
    out['p_fixed_local_mm'] = (p_fix * 1000).tolist()

    def fixed_tip(q):
        F = fk(q)['gripper_frame_link']
        return F[:3, 3] + F[:3, :3] @ p_fix

    # 目标：方块中心 + 半宽*右侧方向，高度 = 爪尖高度
    rd3 = np.array([rd[0], rd[1], 0.0])
    target = np.array([a.x, a.y, 0.0]) + a.half_mm / 1000.0 * rd3
    tip_z = a.tip_z_mm / 1000.0
    target_hi = np.array([target[0], target[1], tip_z + a.hover_mm / 1000.0])
    target_lo = np.array([target[0], target[1], tip_z])
    print(f'方块中心 ({a.x:.4f},{a.y:.4f})  右侧方向 {np.round(rd,3).tolist()}')
    print(f'固定爪目标: 上方 ({target_hi[0]:.4f},{target_hi[1]:.4f},'
          f'{target_hi[2]*1000:+.1f}mm) -> 抓取 z {tip_z*1000:+.1f}mm')

    def solve(target_p, seed_q, grip, w_axis=0.15, w_x=0.15):
        seed = np.clip(seed_q[:5], lo[:5] + 1e-6, hi[:5] - 1e-6)

        def residual(arm):
            q = np.r_[arm, grip]
            F = fk(q)['gripper_frame_link']
            p = F[:3, 3] + F[:3, :3] @ p_fix
            zax = F[:3, :3] @ np.array([0.0, 0.0, 1.0])
            xax = F[:3, :3] @ np.array([1.0, 0.0, 0.0])
            return np.r_[p - target_p, w_axis * (zax - DOWN), w_x * (xax - rd3)]
        sol = least_squares(residual, seed, bounds=(lo[:5], hi[:5]), max_nfev=400)
        return np.r_[sol.x, grip]

    rng = np.random.default_rng(3)
    seeds = [read()]
    seeds += [lo[:5] + rng.random(5) * (hi[:5] - lo[:5]) for _ in range(10)]

    def best(target_p, grip):
        b, bc = None, None
        for s in seeds:
            c = solve(target_p, s, grip)
            e = float(np.linalg.norm(fixed_tip(c) - target_p))
            if bc is None or e < bc:
                b, bc = c, e
        return b, bc

    q_now = read()
    grip_open = a.open_rad
    q_hi, e_hi = best(target_hi, grip_open)
    print(f'IK 上方: 固定爪误差 {e_hi*1000:.1f}mm', flush=True)
    if e_hi > 0.005:
        print('IK 够不到，放弃', flush=True)
        bus.close()
        return
    out.update(ik_hi_err_mm=float(e_hi * 1000), q_hi=q_hi.tolist(),
               target_hi_m=target_hi.tolist(), target_lo_m=target_lo.tolist())
    if not a.execute:
        print('PLAN ONLY', flush=True)
        Path(a.report).write_text(json.dumps(out, indent=2))
        bus.close()
        return

    def move(goal, segs=8, tol=0.05, wait=3.0):
        start = read()
        for i in range(1, segs + 1):
            seg = start + (goal - start) * i / segs
            t0 = time.monotonic()
            while time.monotonic() - t0 < wait:
                write(seg)
                time.sleep(0.2)
                if np.max(np.abs(read() - seg)) < tol:
                    break

    # 先张开爪子（安全），再移到方块右上方
    q = read()
    q[5] = grip_open
    for g in np.linspace(q_now[5], grip_open, 12):
        qq = q_now.copy()
        qq[5] = g
        write(qq)
        time.sleep(0.08)
    move(q_hi)
    p_hi = fixed_tip(read())
    print(f'到上方: 固定爪 ({p_hi[0]:.4f},{p_hi[1]:.4f},{p_hi[2]*1000:+.1f}mm)',
          flush=True)

    # 数值竖直落下
    for i in range(1, 13):
        want = target_hi + (target_lo - target_hi) * i / 12
        q_c, err = best(want, read(), grip_open)
        if err > 0.006:
            print(f'下落第{i}步 IK 误差 {err*1000:.1f}mm，停止', flush=True)
            break
        write(q_c)
        time.sleep(0.35)
    p_lo = fixed_tip(read())
    print(f'落到底: 固定爪 ({p_lo[0]:.4f},{p_lo[1]:.4f},{p_lo[2]*1000:+.1f}mm)',
          flush=True)
    out['fixed_tip_bottom_m'] = p_lo.tolist()
    lw, lk = model.lowest_over_all(dict(zip(JOINTS, read())))
    print(f'  此时整臂最低点 {lk} z={lw[2]*1000:+.2f}mm '
          f'(桌面 {TABLE_Z*1000:+.2f})', flush=True)

    # 合爪找接触
    q0 = read()
    load0 = bus.read_gripper_load()[0]
    close = q0.copy()
    hits, pct = 0, load0
    deadline = time.monotonic() + 20
    while close[5] > a.close_min_rad and time.monotonic() < deadline:
        close[5] = max(a.close_min_rad, close[5] - 0.006)
        lw, lk = model.lowest_over_all(dict(zip(JOINTS, close)))
        if lw[2] < TABLE_Z + 0.0008:
            raise RuntimeError(f'合爪会碰桌面({lk})')
        write(close)
        time.sleep(0.09)
        pct = bus.read_gripper_load()[0]
        hits = hits + 1 if pct >= max(a.load_pct, load0 + 6) else 0
        if hits >= 3:
            break
    out.update(contact_load_pct=float(pct), hold_rad=float(close[5]), hits=hits)
    if hits < 3:
        print(f'未检到接触（载荷 {pct:.1f}%）；方块可能不在固定爪左侧', flush=True)
        Path(a.report).write_text(json.dumps(out, indent=2))
        bus.close()
        return
    print(f'夹持成立: 载荷 {pct:.1f}%  夹爪 {close[5]:.4f}rad', flush=True)
    m = read()
    close[5] = max(lo[5], m[5] - 0.008)
    write(close)
    time.sleep(0.4)

    # 抬起
    p0 = fixed_tip(read())
    for i in range(1, 11):
        want = p0 + np.array([0, 0, a.lift_mm / 1000 * i / 10])
        q_c, err = best(want, read(), close[5])
        if err > 0.006:
            break
        write(q_c)
        time.sleep(0.3)
    p1 = fixed_tip(read())
    out['lift_mm'] = float((p1[2] - p0[2]) * 1000)
    out['end_load_pct'] = float(bus.read_gripper_load()[0])
    out['end_fixed_tip_m'] = p1.tolist()
    ok = out['lift_mm'] >= a.lift_mm * 0.5 and out['end_load_pct'] >= 3.0
    out['completed'] = bool(ok)
    print(f'抬起 {out["lift_mm"]:.1f}mm  保持载荷 {out["end_load_pct"]:.1f}%', flush=True)
    print('GRASP OK: 方块已抓起' if ok else 'WARNING: 抬升/载荷不足', flush=True)
    Path(a.report).write_text(json.dumps(out, indent=2))
    bus.close()


if __name__ == '__main__':
    main()
