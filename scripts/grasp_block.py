#!/usr/bin/env python3
"""上方抓取：以"成功抓过的示教位姿"为模板，抓指定 (x,y) 的物块。

模板（quick_taught_grasp.json 实测成功）：
  TCP(gripper_frame) = (0.27146, 0.03502, -0.05662)
  q               = [-0.1166, 0.9480, -0.4587, 1.2364, -1.5509, 0.3099]
即该位姿下夹爪正环住一个立在棋盘上的 4cm 方块。

流程：抬到物块上方 50mm -> 下探到同一抓取高度 -> 合爪找接触 -> 抬升 -> 校验。
"""
import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
from scipy.optimize import least_squares
import yaml

sys.path.insert(0, '/home/ros/legacy/arm/arm-final/ros2_ws/src/so101_bringup')
sys.path.insert(0, '/home/ros/QianLi/qianli_ws/src/qianli_vision/scripts')
from so101_bringup.servo_protocol import FeetechSerialBus
from gripper_model import GripperModel, JOINTS

CONFIG = ('/home/ros/legacy/arm/arm-final/ros2_ws/install/so101_bringup/share/'
          'so101_bringup/config/driver_params.yaml')
TABLE_Z = -0.06909
TEACH_Q = np.array([-0.11658253987930872, 0.9480001269133262,
                    -0.4586602555778067, 1.2363885150358267,
                    -1.5508545765523831, 0.3098641191528995])
TEACH_TCP = np.array([0.27145880900300084, 0.03502343513643272,
                      -0.05662023769794756])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--x', type=float, required=True)
    ap.add_argument('--y', type=float, required=True)
    ap.add_argument('--grasp-tcp-z-mm', type=float, default=-54.0)
    ap.add_argument('--hover-mm', type=float, default=50.0)
    ap.add_argument('--lift-mm', type=float, default=40.0)
    ap.add_argument('--open-rad', type=float, default=0.55)
    ap.add_argument('--close-min-rad', type=float, default=0.18)
    ap.add_argument('--load-pct', type=float, default=8.0)
    ap.add_argument('--execute', action='store_true')
    ap.add_argument('--axis-weight', type=float, default=0.6,
                    help='工具轴约束权重；0=只求位置')
    ap.add_argument('--ik-tol-mm', type=float, default=4.0)
    ap.add_argument('--report', default='/tmp/grasp_block.json')
    a = ap.parse_args()

    cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
    zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
    lo = (np.array(cfg['raw_min']) - zero) * direction * 2 * math.pi / 4096
    hi = (np.array(cfg['raw_max']) - zero) * direction * 2 * math.pi / 4096
    lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)

    bus = FeetechSerialBus('/dev/ttyACM0', timeout_s=0.08)
    model = GripperModel(stride=12)
    out = {'mode': 'execute' if a.execute else 'plan'}

    def read():
        return (np.array(bus.read_positions()) - zero) * direction * 2 * math.pi / 4096

    def write(q):
        if np.any(q < lo - 1e-6) or np.any(q > hi + 1e-6) or not np.all(np.isfinite(q)):
            raise ValueError('command out of measured limits')
        bus.write_positions(np.rint(zero + q * direction * 4096 / (2 * math.pi)).astype(int).tolist())

    def tcp(q):
        return model.solve(dict(zip(JOINTS, q)))['gripper_frame_link']

    # 模板的工具轴方向
    Rt = tcp(TEACH_Q)[:3, :3]
    zax_ref = Rt @ np.array([0.0, 0.0, 1.0])
    print(f'模板 TCP {np.round(TEACH_TCP,4).tolist()}  工具z轴 '
          f'{np.round(zax_ref,3).tolist()}', flush=True)

    grasp_tcp = np.array([a.x, a.y, a.grasp_tcp_z_mm / 1000.0])
    hover_tcp = grasp_tcp + np.array([0, 0, a.hover_mm / 1000.0])

    def solve(target, seed_q, grip):
        seed = np.clip(seed_q[:5], lo[:5] + 1e-6, hi[:5] - 1e-6)

        def residual(arm):
            q = np.r_[arm, grip]
            f = tcp(q)
            zax = f[:3, :3] @ np.array([0.0, 0.0, 1.0])
            return np.r_[f[:3, 3] - target, a.axis_weight * (zax - zax_ref)]
        sol = least_squares(residual, seed, bounds=(lo[:5], hi[:5]), max_nfev=400)
        return np.r_[sol.x, grip]

    rng = np.random.default_rng(1)
    seeds = [TEACH_Q, read()]
    for _ in range(8):
        seeds.append(lo[:5] + rng.random(5) * (hi[:5] - lo[:5]))

    def best_solve(target, grip):
        best, bc = None, None
        for s in seeds:
            c = solve(target, s, grip)
            e = float(np.linalg.norm(tcp(c)[:3, 3] - target))
            if bc is None or e < bc:
                best, bc = c, e
        return best, bc

    grip_open = a.open_rad
    hover_q, e1 = best_solve(hover_tcp, grip_open)
    grasp_q, e2 = best_solve(grasp_tcp, grip_open)
    for tag, cand in (('悬停', hover_q), ('抓取', grasp_q)):
        zax = tcp(cand)[:3, :3] @ np.array([0.0, 0.0, 1.0])
        tilt = math.degrees(math.acos(float(np.clip(zax @ zax_ref, -1, 1))))
        print(f'  {tag} 工具轴偏离模板 {tilt:.1f}°', flush=True)
    print(f'IK 悬停: 误差 {e1*1000:.1f}mm   抓取: 误差 {e2*1000:.1f}mm', flush=True)
    out.update(hover_tcp=hover_tcp.tolist(), grasp_tcp=grasp_tcp.tolist(),
               ik_hover_err_mm=float(e1 * 1000), ik_grasp_err_mm=float(e2 * 1000),
               hover_q=hover_q.tolist(), grasp_q=grasp_q.tolist())
    if e1 > a.ik_tol_mm / 1000 or e2 > a.ik_tol_mm / 1000:
        print(f'IK 精度不足（容差 {a.ik_tol_mm}mm），放弃', flush=True)
        Path(a.report).write_text(json.dumps(out, indent=2))
        bus.close()
        return

    # 路径净空
    q_now = read()
    path = [q_now + (hover_q - q_now) * i / 10 for i in range(1, 11)]
    path += [hover_q + (grasp_q - hover_q) * i / 8 for i in range(1, 9)]
    worst = None
    for s in path:
        lw, lk = model.lowest_over_all(dict(zip(JOINTS, s)))
        if worst is None or lw[2] < worst[0]:
            worst = (lw[2], lk)
    print(f'路径最低点: {worst[1]} z={worst[0]*1000:+.2f}mm '
          f'(桌面 {TABLE_Z*1000:+.2f}mm)', flush=True)
    out['path_lowest_mm'] = float(worst[0] * 1000)
    out['path_lowest_link'] = worst[1]
    if worst[0] < TABLE_Z + 0.004:
        print('路径会刮桌面，放弃', flush=True)
        Path(a.report).write_text(json.dumps(out, indent=2))
        bus.close()
        return
    if not a.execute:
        print('PLAN ONLY: 不写寄存器', flush=True)
        Path(a.report).write_text(json.dumps(out, indent=2))
        bus.close()
        return

    def move(goal, segs=10, tol=0.05):
        start = read()
        for i in range(1, segs + 1):
            seg = start + (goal - start) * i / segs
            t0 = time.monotonic()
            while time.monotonic() - t0 < 3.0:
                write(seg)
                time.sleep(0.2)
                if np.max(np.abs(read() - seg)) < tol:
                    break

    move(hover_q)
    fs = tcp(read())[:3, 3]
    print(f'悬停到位 TCP ({fs[0]:.4f},{fs[1]:.4f},{fs[2]*1000:+.1f}mm)', flush=True)
    time.sleep(0.5)
    move(grasp_q)
    fg = tcp(read())[:3, 3]
    print(f'下探到位 TCP ({fg[0]:.4f},{fg[1]:.4f},{fg[2]*1000:+.1f}mm)', flush=True)

    # 合爪找接触
    q0 = read()
    load0 = bus.read_gripper_load()[0]
    close = q0.copy()
    hits, pct = 0, load0
    deadline = time.monotonic() + 20
    while close[5] > a.close_min_rad and time.monotonic() < deadline:
        close[5] = max(a.close_min_rad, close[5] - 0.006)
        lw, lk = model.lowest_over_all(dict(zip(JOINTS, close)))
        if lw[2] < TABLE_Z + 0.001:
            raise RuntimeError(f'合爪会碰桌面({lk})')
        write(close)
        time.sleep(0.09)
        pct = bus.read_gripper_load()[0]
        hits = hits + 1 if pct >= max(a.load_pct, load0 + 6) else 0
        if hits >= 3:
            break
    out.update(contact_load_pct=float(pct), hold_gripper_rad=float(close[5]),
               hits=hits)
    if hits < 3:
        print(f'未检到接触（载荷 {pct:.1f}%），物块可能不在夹口之间', flush=True)
        Path(a.report).write_text(json.dumps(out, indent=2))
        bus.close()
        return
    print(f'夹持成立: 载荷 {pct:.1f}%  夹爪 {close[5]:.4f}rad', flush=True)
    m = read()
    close[5] = max(lo[5], m[5] - 0.008)
    write(close)
    time.sleep(0.4)

    # 抬升
    p0 = tcp(read())[:3, 3].copy()
    for i in range(1, 11):
        target = p0 + np.array([0, 0, a.lift_mm / 1000.0 * i / 10])
        goal, err = best_solve(target, read(), close[5])
        if err > 0.005:
            print(f'抬升点 {i} IK 误差 {err*1000:.1f}mm，停止抬升', flush=True)
            break
        move(goal, segs=3, tol=0.06)
    end = read()
    out['measured_lift_mm'] = float((tcp(end)[2, 3] - p0[2]) * 1000)
    out['end_load_pct'] = float(bus.read_gripper_load()[0])
    out['end_tcp_m'] = tcp(end)[:3, 3].tolist()
    need = a.lift_mm * 0.5
    ok = out['measured_lift_mm'] >= need and out['end_load_pct'] >= 3.0
    out['completed'] = bool(ok)
    print(f'抬升实测 {out["measured_lift_mm"]:.1f}mm（验收≥{need:.0f}）'
          f'  保持载荷 {out["end_load_pct"]:.1f}%', flush=True)
    print('GRASP OK: 物块已抓起' if ok else 'WARNING: 抬升或保持载荷不足', flush=True)
    Path(a.report).write_text(json.dumps(out, indent=2))
    bus.close()


if __name__ == '__main__':
    main()
