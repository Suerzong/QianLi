#!/usr/bin/env python3
"""带重试的关节移动：串口偶发超时时自动重试，避免中途掉线。"""

from project_paths import default_arm_port

from project_paths import arm_source_path, driver_params_path
import argparse
import math
import os
import sys
import time

import numpy as np
import yaml
from pathlib import Path

sys.path.insert(0, arm_source_path())
from so101_bringup.servo_protocol import FeetechSerialBus

CONFIG = os.path.expanduser(
    driver_params_path())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--q', nargs=6, type=float, required=True)
    ap.add_argument('--segs', type=int, default=14)
    ap.add_argument('--settle', type=float, default=2.5)
    a = ap.parse_args()

    cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
    zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
    lo = (np.array(cfg['raw_min']) - zero) * direction * 2 * math.pi / 4096
    hi = (np.array(cfg['raw_max']) - zero) * direction * 2 * math.pi / 4096
    lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)

    bus = FeetechSerialBus(default_arm_port(), timeout_s=0.15)

    def retry(fn, n=8, tag=''):
        last = None
        for i in range(n):
            try:
                return fn()
            except Exception as exc:
                last = exc
                time.sleep(0.25)
        raise RuntimeError(f'{tag} 连续 {n} 次失败: {last}')

    def read():
        return (np.array(retry(bus.read_positions, tag='读位置')) - zero) \
            * direction * 2 * math.pi / 4096

    def torque_on():
        retry(lambda: bus.set_torque(True), tag='使能力矩')
        return retry(bus.read_torque_states, tag='读力矩')

    print('使能力矩:', torque_on())
    time.sleep(0.5)
    tgt = np.clip(np.array(a.q), lo, hi)
    print('目标:', np.round(tgt, 4).tolist())
    start = read()
    print('起点:', np.round(start, 4).tolist())
    for i in range(1, a.segs + 1):
        seg = start + (tgt - start) * i / a.segs
        raw = np.rint(zero + seg * direction * 4096 / (2 * math.pi)).astype(int)
        retry(lambda: bus.write_positions(raw.tolist()), tag='写位置')
        t0 = time.monotonic()
        while time.monotonic() - t0 < a.settle:
            time.sleep(0.15)
            if np.max(np.abs(read() - seg)) < 0.06:
                break
    end = read()
    print('到位:', np.round(end, 4).tolist(),
          f'最大误差 {np.max(np.abs(end - tgt)):.4f} rad')
    print('力矩:', retry(bus.read_torque_states, tag='读力矩'))
    bus.close()


if __name__ == '__main__':
    main()
