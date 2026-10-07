#!/usr/bin/env python3
"""收敛式退避：把夹爪抬到距桌面指定净空。只向上运动。

为什么不用"一步发一次指令"：位置舵机在重力负载下上行会明显滞后，
单次下发实测只走一小部分（示教抓取里"指令 50mm 只抬到 37.8mm"就是这个）。
这里对每个小目标**重复下发直到实测收敛**，再进入下一个小目标。
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
    ap.add_argument('--clearance-mm', type=float, default=30.0,
                    help='目标净空（夹爪最低点距桌面）')
    ap.add_argument('--step-mm', type=float, default=4.0)
    ap.add_argument('--report', default='/tmp/quick_retreat.json')
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

    goal_low = TABLE_Z + a.clearance_mm / 1000

    try:
        q0 = read()
        low0, link0 = model.lowest_over_all(dict(zip(JOINTS, q0)))
        out.update(start_lowest_mm=float(low0[2] * 1000), start_link=link0,
                   start_tcp_m=frame(q0)[:3, 3].tolist(),
                   torque=bus.read_torque_states())
        print(f'起点: 最低点 {link0} z={low0[2]*1000:+.2f}mm '
              f'(离桌面 {(low0[2]-TABLE_Z)*1000:+.2f}mm)', flush=True)
        if not a.execute:
            print('PLAN ONLY', flush=True)
            return

        trace = []
        for outer in range(20):
            cur = read()
            low, link = model.lowest_over_all(dict(zip(JOINTS, cur)))
            if low[2] >= goal_low:
                print(f'达到净空目标: 最低点 {low[2]*1000:+.2f}mm '
                      f'(离桌面 {(low[2]-TABLE_Z)*1000:+.2f}mm)', flush=True)
                break
            axis = frame(cur)[:3, 2].copy()
            start_p = frame(cur)[:3, 3].copy()
            target = start_p + np.array([0, 0, a.step_mm / 1000])
            seed = cur[:5].copy()

            def residual(arm):
                f = frame(np.r_[arm, cur[5]])
                return np.r_[f[:3, 3] - target, 0.05 * (f[:3, 2] - axis)]

            sol = least_squares(residual, seed, bounds=(lo[:5], hi[:5]), max_nfev=120)
            cand = np.r_[sol.x, cur[5]]
            err = float(np.linalg.norm(frame(cand)[:3, 3] - target))
            if err > 0.003:
                print(f'IK 无法达到目标 (误差 {err*1000:.1f}mm)，停止', flush=True)
                break
            # 收敛下发：重复写同一目标直到实测到位或超时
            t0 = time.monotonic()
            reached = False
            while time.monotonic() - t0 < 3.0:
                write(cand)
                time.sleep(0.25)
                now = read()
                if np.linalg.norm(frame(now)[:3, 3] - target) < 0.002:
                    reached = True
                    break
            low_now, _ = model.lowest_over_all(dict(zip(JOINTS, read())))
            trace.append({'outer': outer, 'reached': reached,
                          'lowest_mm': float(low_now[2] * 1000)})
            print(f'  第{outer+1}步: 目标 {target[2]*1000:+.1f}mm '
                  f'{"到位" if reached else "超时(仍有偏差)"} '
                  f'最低点 {low_now[2]*1000:+.2f}mm', flush=True)
            if not reached:
                break
        out['trace'] = trace
        q_end = read()
        low_end, link_end = model.lowest_over_all(dict(zip(JOINTS, q_end)))
        out.update(end_lowest_mm=float(low_end[2] * 1000), end_link=link_end,
                   end_tcp_m=frame(q_end)[:3, 3].tolist(),
                   clearance_mm=float((low_end[2] - TABLE_Z) * 1000))
        print(f'结束: 最低点 {link_end} z={low_end[2]*1000:+.2f}mm '
              f'(离桌面 {(low_end[2]-TABLE_Z)*1000:+.2f}mm)', flush=True)
        out['ok'] = bool(low_end[2] >= goal_low - 0.003)
    except BaseException as exc:
        out['error'] = str(exc)
        print(f'STOP: {exc}', flush=True)
    finally:
        Path(a.report).write_text(json.dumps(out, indent=2))
        bus.close()


if __name__ == '__main__':
    main()
