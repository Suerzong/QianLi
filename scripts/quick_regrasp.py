#!/usr/bin/env python3
"""就地重抓：夹爪已张开并跨在物块两侧，合爪找接触 → 抬升。

阈值按实际能力放宽（抬升验收按 60% 折算），失败即安全停下、不抬。
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


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute', action='store_true')
    ap.add_argument('--lift-mm', type=float, default=40.0)
    ap.add_argument('--load-pct', type=float, default=8.0)
    ap.add_argument('--report', default='/tmp/quick_regrasp.json')
    a = ap.parse_args()

    cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
    zero = np.array(cfg['zero_raw'])
    direction = np.array(cfg['direction'])
    lo = (np.array(cfg['raw_min']) - zero) * direction * 2 * math.pi / 4096
    hi = (np.array(cfg['raw_max']) - zero) * direction * 2 * math.pi / 4096
    lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)

    bus = FeetechSerialBus(default_arm_port(), timeout_s=0.08)
    model = GripperModel(stride=12)
    result = {'mode': 'execute' if a.execute else 'plan', 'completed': False}

    def read():
        return (np.array(bus.read_positions()) - zero) * direction * 2 * math.pi / 4096

    def write(q):
        if not np.all(np.isfinite(q)) or np.any(q < lo - 1e-6) or np.any(q > hi + 1e-6):
            raise ValueError('command exceeds measured limits')
        bus.write_positions(np.rint(zero + q * direction * 4096 / (2 * math.pi)).astype(int).tolist())

    def frame(q):
        return model.solve(dict(zip(JOINTS, q)))['gripper_frame_link']

    def lowest(q):
        return model.lowest_over_all(dict(zip(JOINTS, q)))

    try:
        q0 = read()
        torque = bus.read_torque_states()
        load0 = bus.read_gripper_load()[0]
        low0, link0 = lowest(q0)
        result.update(start_gripper_rad=float(q0[5]), start_load_pct=load0,
                      start_lowest_mm=float(low0[2] * 1000), start_lowest_link=link0,
                      start_tcp_m=frame(q0)[:3, 3].tolist(), torques=torque)
        print(f'起始: 夹爪 {q0[5]:.4f}rad  载荷 {load0:.1f}%  最低点 '
              f'{link0} z={low0[2]*1000:+.2f}mm', flush=True)
        if torque != [1] * 6:
            raise ValueError('arm is not torque-held')
        if not a.execute:
            print('PLAN ONLY: no writes', flush=True)
            return
        if load0 > 3.0:
            raise ValueError(f'jaws already loaded ({load0:.1f}%); not a free grasp')

        # ---- 合爪找接触 ----
        close = q0.copy()
        hits = 0
        pct = load0
        deadline = time.monotonic() + 20
        while close[5] > lo[5] + 0.008 and time.monotonic() < deadline:
            close[5] = max(lo[5], close[5] - 0.006)
            lw, _ = lowest(close)
            if lw[2] < TABLE_Z + 0.0005:
                raise RuntimeError('jaw closure would touch table')
            write(close)
            time.sleep(0.09)
            pct = bus.read_gripper_load()[0]
            hits = hits + 1 if pct >= max(a.load_pct, load0 + 6) else 0
            if hits >= 3:
                break
        if hits < 3:
            result['contact_load_pct'] = pct
            print(f'未检到夹持接触（最终载荷 {pct:.1f}%），物块可能不在夹爪之间', flush=True)
            raise RuntimeError('no jaw contact detected; not lifting')
        measured = read()
        close[5] = max(lo[5], measured[5] - 0.008)
        write(close)
        time.sleep(0.4)
        result.update(contact_load_pct=pct, held_gripper_rad=float(close[5]))
        print(f'接触成立: 载荷 {pct:.1f}%，夹爪停在 {close[5]:.4f}rad', flush=True)

        # ---- 抬升 ----
        q_hold = read()
        p0 = frame(q_hold)[:3, 3].copy()
        axis = frame(q_hold)[:3, 2].copy()
        seed = q_hold[:5].copy()
        waypoints = []
        for z in np.linspace(0, a.lift_mm / 1000, 9)[1:]:
            target = p0 + [0, 0, z]
            def residual(arm):
                f = frame(np.r_[arm, q_hold[5]])
                return np.r_[f[:3, 3] - target, 0.05 * (f[:3, 2] - axis)]
            sol = least_squares(residual, seed, bounds=(lo[:5], hi[:5]), max_nfev=100)
            cand = np.r_[sol.x, q_hold[5]]
            err = float(np.linalg.norm(frame(cand)[:3, 3] - target))
            lw, lk = lowest(cand)
            if err > 0.003 or lw[2] < TABLE_Z + 0.0005:
                print(f'抬升点不可达 (IK {err*1000:.1f}mm / 最低 {lw[2]*1000:+.1f}mm)',
                      flush=True)
                break
            waypoints.append(cand.tolist())
            seed = sol.x
        if not waypoints:
            raise RuntimeError('no lift waypoint available')
        result['lift_waypoints'] = waypoints

        for target in waypoints:
            tgt = np.array(target)
            tgt[5] = close[5]
            for _ in range(4):
                cur = read()
                cur[5] = close[5]
                err = float(np.linalg.norm(frame(cur)[:3, 3] - frame(tgt)[:3, 3]))
                if err < 0.010:
                    break
                write(tgt)
                time.sleep(0.25)
        actual = read()
        result['measured_lift_mm'] = float((frame(actual)[2, 3] - p0[2]) * 1000)
        result['end_gripper_rad'] = float(actual[5])
        result['end_load_pct'] = bus.read_gripper_load()[0]
        need = a.lift_mm * 0.6
        print(f'抬升实测 {result["measured_lift_mm"]:.1f}mm '
              f'(指令 {a.lift_mm:.0f}mm, 验收 ≥{need:.0f}mm)，'
              f'保持载荷 {result["end_load_pct"]:.1f}%', flush=True)
        if result['measured_lift_mm'] >= need and result['end_load_pct'] >= 3.0:
            result['completed'] = True
            print('GRASP OK: 物块已被抓起并保持在夹爪中', flush=True)
        else:
            print('WARNING: 抬升或保持载荷不足，物块可能滑落', flush=True)
        result['end_tcp_m'] = frame(actual)[:3, 3].tolist()
    except BaseException as exc:
        result['error'] = str(exc)
        print(f'STOP: {exc}', flush=True)
    finally:
        Path(a.report).write_text(json.dumps(result, indent=2))
        print(json.dumps({k: v for k, v in result.items() if k != 'lift_waypoints'},
                         indent=2), flush=True)
        bus.close()


if __name__ == '__main__':
    main()
