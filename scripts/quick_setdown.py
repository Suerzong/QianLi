#!/usr/bin/env python3
"""把当前夹持的物块降到桌面并张开夹爪释放（闭环，带堵转保护）。

用实测关节角算"夹爪几何最低点"（≈物块底面），逐步下探到距桌面 ~2mm，
再张开夹爪。全程不写超出实测限位的值。
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


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute', action='store_true')
    # 2026-10-06 实测：模型在这一带的竖直估计比现实乐观 2~3mm（标定 z 判据
    # 容差本身就是 ±2mm）。用 2mm 净空会把爪尖压进桌面（已实际发生），
    # 所以默认留 8mm，并把步长减半以免单步冲过头。
    ap.add_argument('--clearance-mm', type=float, default=8.0)
    ap.add_argument('--step-mm', type=float, default=1.5)
    ap.add_argument('--open-rad', type=float, default=0.70)
    ap.add_argument('--report', default='/tmp/quick_setdown.json')
    a = ap.parse_args()

    cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
    zero = np.array(cfg['zero_raw'])
    direction = np.array(cfg['direction'])
    lo = (np.array(cfg['raw_min']) - zero) * direction * 2 * math.pi / 4096
    hi = (np.array(cfg['raw_max']) - zero) * direction * 2 * math.pi / 4096
    lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)

    bus = FeetechSerialBus('/dev/ttyACM0', timeout_s=0.08)
    result = {'mode': 'execute' if a.execute else 'plan', 'completed': False}
    model = GripperModel(stride=12)

    def read():
        return (np.array(bus.read_positions()) - zero) * direction * 2 * math.pi / 4096

    def write(q):
        if not np.all(np.isfinite(q)) or np.any(q < lo - 1e-6) or np.any(q > hi + 1e-6):
            raise ValueError('command exceeds measured limits')
        raw = np.rint(zero + q * direction * 4096 / (2 * math.pi)).astype(int)
        bus.write_positions(raw.tolist())

    def frame(q):
        return model.solve(dict(zip(JOINTS, q)))['gripper_frame_link']

    def lowest(q):
        return model.lowest_over_all(dict(zip(JOINTS, q)))

    goal_z = TABLE_Z + a.clearance_mm / 1000

    try:
        q0 = read()
        torque = bus.read_torque_states()
        load0 = bus.read_gripper_load()[0]
        low0, link0 = lowest(q0)
        f0 = frame(q0)
        result.update(start_tcp_m=f0[:3, 3].tolist(),
                      start_lowest_mm=low0[2] * 1000,
                      start_lowest_link=link0,
                      start_load_pct=load0,
                      start_gripper_rad=float(q0[5]),
                      torques=torque)
        print(f'起始: TCP z={f0[2,3]*1000:+.1f}mm  最低点 {link0} '
              f'z={low0[2]*1000:+.2f}mm  桌面 {TABLE_Z*1000:+.2f}mm  '
              f'载荷 {load0:.1f}%  夹爪 {q0[5]:.4f}rad', flush=True)
        if torque != [1] * 6:
            raise ValueError('arm is not torque-held; refusing to move')
        if not a.execute:
            print('PLAN ONLY: no writes', flush=True)
            return

        # ---- 逐步下探 ----
        steps = []
        stagnant = 0
        prev_low_z = low0[2]
        for i in range(40):
            q = read()
            low, link = lowest(q)
            if low[2] <= goal_z:
                print(f'到达目标高度: 最低点 {low[2]*1000:+.2f}mm', flush=True)
                break
            if low[2] >= prev_low_z - 1e-5:
                stagnant += 1
                if stagnant >= 2:
                    print('连续两次下探无进展，判定已触底，停止下探', flush=True)
                    break
            else:
                stagnant = 0
            prev_low_z = low[2]
            axis = frame(q)[:3, 2].copy()
            target = frame(q)[:3, 3] - np.array([0, 0, a.step_mm / 1000])
            seed = q[:5].copy()

            def residual(arm):
                f = frame(np.r_[arm, q[5]])
                return np.r_[f[:3, 3] - target, 0.05 * (f[:3, 2] - axis)]

            sol = least_squares(residual, seed, bounds=(lo[:5], hi[:5]),
                                max_nfev=100)
            cand = np.r_[sol.x, q[5]]
            err = float(np.linalg.norm(frame(cand)[:3, 3] - target))
            if err > 0.003:
                print(f'IK 误差 {err*1000:.1f}mm，停止下探', flush=True)
                break
            write(cand)
            time.sleep(0.35)
            steps.append({'lowest_z_mm': float(low[2] * 1000),
                          'tcp_z_mm': float(frame(read())[2, 3] * 1000)})
        result['descent_steps'] = steps
        q_now = read()
        low_now, link_now = lowest(q_now)
        result['final_lowest_mm'] = float(low_now[2] * 1000)
        print(f'下探结束: 最低点 {low_now[2]*1000:+.2f}mm '
              f'(桌面 {TABLE_Z*1000:+.2f}mm, 余量 '
              f'{(low_now[2]-TABLE_Z)*1000:+.2f}mm)', flush=True)

        # ---- 张开夹爪（先确认已足够低，避免高处抛落） ----
        gap_mm = (low_now[2] - TABLE_Z) * 1000
        if gap_mm > 12.0:
            raise ValueError(
                f'下探不足（离桌面还有 {gap_mm:.1f}mm），拒绝张开夹爪；'
                f'物块未释放')
        q_open = q_now.copy()
        for g in np.linspace(q_now[5], a.open_rad, 25):
            q_open = q_now.copy()
            q_open[5] = g
            write(q_open)
            time.sleep(0.08)
        time.sleep(0.6)
        load1 = bus.read_gripper_load()[0]
        result['end_gripper_rad'] = float(q_open[5])
        result['end_load_pct'] = load1
        print(f'夹爪已张开到 {q_open[5]:.3f}rad，载荷 {load0:.1f}% -> {load1:.1f}%',
              flush=True)
        if load1 <= max(3.0, load0 - 5):
            result['released'] = True
            print('RELEASED: 物块已放下', flush=True)
        else:
            print('WARNING: 载荷未明显下降，物块可能仍被夹住/卡住', flush=True)
        # 稍微抬起，离开物块
        for _ in range(6):
            q = read()
            axis = frame(q)[:3, 2].copy()
            target = frame(q)[:3, 3] + np.array([0, 0, 0.006])
            seed = q[:5].copy()

            def residual(arm):
                f = frame(np.r_[arm, q[5]])
                return np.r_[f[:3, 3] - target, 0.05 * (f[:3, 2] - axis)]

            sol = least_squares(residual, seed, bounds=(lo[:5], hi[:5]), max_nfev=80)
            cand = np.r_[sol.x, q[5]]
            write(cand)
            time.sleep(0.25)
        q_end = read()
        result['end_tcp_m'] = frame(q_end)[:3, 3].tolist()
        result['completed'] = True
        print(f'已抬起离开: TCP z={frame(q_end)[2,3]*1000:+.1f}mm', flush=True)
    except BaseException as exc:
        result['error'] = str(exc)
        print(f'STOP: {exc}', flush=True)
        raise
    finally:
        Path(a.report).write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2), flush=True)
        bus.close()


if __name__ == '__main__':
    main()
