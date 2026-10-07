#!/usr/bin/env python3
"""关节空间移动到指定 6 关节位姿（插值 + 收敛下发）。不需要 IK。"""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
import yaml

sys.path.insert(0, '/home/ros/legacy/arm/arm-final/ros2_ws/src/so101_bringup')
from so101_bringup.servo_protocol import FeetechSerialBus

CONFIG = ('/home/ros/legacy/arm/arm-final/ros2_ws/install/so101_bringup/share/'
          'so101_bringup/config/driver_params.yaml')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--q', nargs=6, type=float, required=True)
    ap.add_argument('--execute', action='store_true')
    ap.add_argument('--report', default='/tmp/move_joints.json')
    a = ap.parse_args()

    cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
    zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
    lo = (np.array(cfg['raw_min']) - zero) * direction * 2 * np.pi / 4096
    hi = (np.array(cfg['raw_max']) - zero) * direction * 2 * np.pi / 4096
    lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)

    bus = FeetechSerialBus('/dev/ttyACM0', timeout_s=0.08)
    out = {}
    q_now = (np.array(bus.read_positions()) - zero) * direction * 2 * np.pi / 4096
    tgt = np.array(a.q)
    out['start_q'] = q_now.tolist()
    out['target_q'] = tgt.tolist()
    out['clamped'] = bool(np.any(tgt < lo - 1e-9) or np.any(tgt > hi + 1e-9))
    tgt = np.clip(tgt, lo, hi)
    out['target_q_clamped'] = tgt.tolist()
    print('起点:', np.round(q_now, 4).tolist())
    print('目标:', np.round(tgt, 4).tolist(),
          '(被限幅)' if out['clamped'] else '')
    print('限位: lo', np.round(lo, 3).tolist())
    print('      hi', np.round(hi, 3).tolist())
    if not a.execute:
        print('PLAN ONLY')
        Path(a.report).write_text(json.dumps(out, indent=2))
        bus.close()
        return
    for i in range(1, 13):
        seg = q_now + (tgt - q_now) * i / 12
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            bus.write_positions(np.rint(zero + seg * direction * 4096 /
                                        (2 * np.pi)).astype(int).tolist())
            time.sleep(0.2)
            cur = (np.array(bus.read_positions()) - zero) * direction * 2 * np.pi / 4096
            if np.max(np.abs(cur - seg)) < 0.06:
                break
    end = (np.array(bus.read_positions()) - zero) * direction * 2 * np.pi / 4096
    out['end_q'] = end.tolist()
    out['max_err'] = float(np.max(np.abs(end - tgt)))
    print('到位:', np.round(end, 4).tolist(),
          f'最大误差 {out["max_err"]:.4f} rad')
    Path(a.report).write_text(json.dumps(out, indent=2))
    bus.close()


if __name__ == '__main__':
    main()
