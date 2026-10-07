#!/usr/bin/env python3
"""把夹爪"爪尖"移动到指定位置，工具轴竖直向下。收敛式下发。

注意：当前 shoulder_lift 实测可能略超出标定范围（raw 822 < raw_min 842），
直接拿来做 least_squares 的初值会报 "Initial guess is outside of bounds"，
所以先 clamp 到界内。
"""

from project_paths import default_arm_port

from project_paths import arm_source_path, driver_params_path, project_path
import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
from scipy.optimize import least_squares
import yaml

sys.path.insert(0, arm_source_path())
sys.path.insert(0, project_path('qianli_ws/src/qianli_vision/scripts'))
from so101_bringup.servo_protocol import FeetechSerialBus
from gripper_model import GripperModel, JOINTS

CONFIG = (driver_params_path())
TABLE_Z = -0.06909
DOWN = np.array([0.0, 0.0, -1.0])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--x', type=float, required=True)
    ap.add_argument('--y', type=float, required=True)
    ap.add_argument('--tip-z-mm', type=float, required=True)
    ap.add_argument('--grip-rad', type=float, default=None)
    ap.add_argument('--execute', action='store_true')
    ap.add_argument('--report', default='/tmp/move_tip.json')
    a = ap.parse_args()

    cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
    zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
    lo = (np.array(cfg['raw_min']) - zero) * direction * 2 * math.pi / 4096
    hi = (np.array(cfg['raw_max']) - zero) * direction * 2 * math.pi / 4096
    lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)

    bus = FeetechSerialBus(default_arm_port(), timeout_s=0.08)
    model = GripperModel(stride=12)
    out = {'mode': 'execute' if a.execute else 'plan'}

    def read():
        return (np.array(bus.read_positions()) - zero) * direction * 2 * math.pi / 4096

    def write(q):
        if np.any(q < lo - 1e-6) or np.any(q > hi + 1e-6) or not np.all(np.isfinite(q)):
            raise ValueError('command out of measured limits')
        bus.write_positions(np.rint(zero + q * direction * 4096 / (2 * math.pi)).astype(int).tolist())

    def frame(q):
        return model.solve(dict(zip(JOINTS, q)))['gripper_frame_link']

    def tip(q):
        f = frame(q)
        c = model._tcp_offset_local if hasattr(model, '_tcp_offset_local') else None
        return f

    q_now = read()
    grip = a.grip_rad if a.grip_rad is not None else float(q_now[5])
    T_now = model.solve(dict(zip(JOINTS, q_now)))
    F_now = T_now['gripper_frame_link']
    M_gr = T_now['gripper_link']
    pts_gr = model.parts['gripper_link']
    w_gr = (M_gr[:3, :3] @ pts_gr.T).T + M_gr[:3, 3]
    k = int(np.argmin(w_gr[:, 2]))
    tip_world = w_gr[k]
    # 固定爪顶端在 gripper_frame 中的常向量（标定结论：TCP 高于爪尖 ~11.6mm）
    c_local = F_now[:3, :3].T @ (tip_world - F_now[:3, 3])
    low_now, link_now = model.lowest_over_all(dict(zip(JOINTS, q_now)))
    out.update(start_q=q_now.tolist(), start_lowest_mm=float(low_now[2] * 1000),
               start_link=link_now,
               start_tip_world_m=tip_world.tolist(),
               start_tcp_world_m=F_now[:3, 3].tolist(),
               c_local_mm=(c_local * 1000).tolist(),
               target={'x': a.x, 'y': a.y, 'tip_z_mm': a.tip_z_mm,
                       'grip_rad': grip})
    print(f'起点: 固定爪顶端=({tip_world[0]:.4f},{tip_world[1]:.4f},'
          f'{tip_world[2]*1000:+.1f}mm)  TCP z={F_now[2,3]*1000:+.1f}mm  '
          f'(TCP 高于爪尖 {(F_now[2,3]-tip_world[2])*1000:+.1f}mm)', flush=True)
    print(f'爪尖常向量(工具系) = {np.round(c_local*1000,1).tolist()} mm', flush=True)

    target_tip = np.array([a.x, a.y, a.tip_z_mm / 1000.0])
    UP = np.array([0.0, 0.0, 1.0])

    def solve(seed_q, iters=300):
        seed = np.clip(seed_q[:5], lo[:5] + 1e-6, hi[:5] - 1e-6)

        def residual(arm):
            q = np.r_[arm, grip]
            f = frame(q)
            t = f[:3, 3] + f[:3, :3] @ c_local
            # 工具轴（gripper_frame 的 z）竖直朝上 => 爪口水平、爪尖朝下
            zax = f[:3, :3] @ UP
            return np.r_[t - target_tip, 0.5 * (zax - UP)]
        sol = least_squares(residual, seed, bounds=(lo[:5], hi[:5]),
                            max_nfev=iters)
        return np.r_[sol.x, grip]

    # 多起点：当前姿态 + 几个"向下够"的启发位姿 + 界内随机
    rng = np.random.default_rng(0)
    seeds = [q_now[:5]]
    for base_seed in ([0.0, 0.6, 0.6, 1.0, -1.4],
                      [0.0, 1.0, -0.4, 1.3, -1.5],
                      [0.0, 0.2, 1.0, 0.8, -1.5]):
        seeds.append(np.array(base_seed))
    for _ in range(6):
        seeds.append(lo[:5] + rng.random(5) * (hi[:5] - lo[:5]))

    best, best_cost = None, None
    for s in seeds:
        cand_s = solve(s)
        f = frame(cand_s)
        t = f[:3, 3] + f[:3, :3] @ c_local
        c = float(np.linalg.norm(t - target_tip))
        if best_cost is None or c < best_cost:
            best, best_cost = cand_s, c
    cand = best
    f = frame(cand)
    t = f[:3, 3] + f[:3, :3] @ c_local
    zax = f[:3, :3] @ UP
    err = float(np.linalg.norm(t - target_tip))
    ang = math.degrees(math.acos(float(np.clip(zax @ UP, -1, 1))))
    print(f'IK: 爪尖误差 {err*1000:.1f}mm, 工具轴偏离竖直 {ang:.1f}°, '
          f'解={np.round(cand,4).tolist()}', flush=True)
    out.update(ik_err_mm=float(err * 1000), axis_off_deg=float(ang),
               solved_q=cand.tolist())
    if err > 0.005:
        print('爪尖无法到达目标，放弃', flush=True)
        Path(a.report).write_text(json.dumps(out, indent=2))
        bus.close()
        return
    if not a.execute:
        print('PLAN ONLY: no writes', flush=True)
        Path(a.report).write_text(json.dumps(out, indent=2))
        bus.close()
        return

    # 分 8 段插值过去，每段收敛下发；先检查整条路径的净空
    q_start = read()
    segs = [q_start + (cand - q_start) * i / 8 for i in range(1, 9)]
    worst = None
    for s in segs:
        lw, lk = model.lowest_over_all(dict(zip(JOINTS, s)))
        if worst is None or lw[2] < worst[0]:
            worst = (lw[2], lk, s)
    print(f'路径最低点: {worst[1]} z={worst[0]*1000:+.2f}mm '
          f'(桌面 {TABLE_Z*1000:+.2f}mm)', flush=True)
    out['path_lowest_mm'] = float(worst[0] * 1000)
    out['path_lowest_link'] = worst[1]
    if worst[0] < TABLE_Z + 0.006:
        print('路径会刮到桌面，放弃', flush=True)
        Path(a.report).write_text(json.dumps(out, indent=2))
        bus.close()
        return

    for seg in segs:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            write(seg)
            time.sleep(0.22)
            cur = read()
            if np.max(np.abs(cur[:5] - seg[:5])) < 0.05:
                break
    q_end = read()
    f = frame(q_end)
    t = f[:3, 3] + f[:3, :3] @ c_local
    low, link = model.lowest_over_all(dict(zip(JOINTS, q_end)))
    out.update(end_q=q_end.tolist(), end_tip_m=t.tolist(),
               end_lowest_mm=float(low[2] * 1000), end_link=link,
               end_tip_err_mm=float(np.linalg.norm(t - target_tip) * 1000))
    print(f'到位: 爪尖=({t[0]:.4f},{t[1]:.4f},{t[2]*1000:+.1f}mm) '
          f'误差 {out["end_tip_err_mm"]:.1f}mm  最低点 {link} '
          f'z={low[2]*1000:+.2f}mm', flush=True)
    Path(a.report).write_text(json.dumps(out, indent=2))
    bus.close()


if __name__ == '__main__':
    main()
