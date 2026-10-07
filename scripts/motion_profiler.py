#!/usr/bin/env python3
"""到位平滑度剖析：下发一条轨迹，到位后高频采样 2s。

指标（每个关节）：
  - 过冲 overshoot：越过目标的最大偏差(mrad)
  - 回摆 ringing：收敛期间速度变号次数
  - 到位抖动 hold_jitter：稳定后的 std(mrad) —— D 增益的直接体现
  - 收敛时间 settle_ms：最后一次下发后到进入 ±10mrad 稳定带
"""

from project_paths import default_arm_port

from project_paths import arm_source_path, driver_params_path, project_path
import json
import math
import os
import sys
import time

import numpy as np
import yaml
from pathlib import Path

sys.path.insert(0, arm_source_path())
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from so101_bringup.servo_protocol import FeetechSerialBus

CONFIG = os.path.expanduser(
    driver_params_path())
CFG = os.path.expanduser(project_path('config'))

cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
lo = (np.array(cfg['raw_min']) - zero) * direction * 2 * math.pi / 4096
hi = (np.array(cfg['raw_max']) - zero) * direction * 2 * math.pi / 4096
lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)


def pose(name):
    p = os.path.join(CFG, name + '_pose.json')
    raw = json.load(open(p))['raw_exec']
    return np.clip((np.array(raw) - zero) * direction * 2 * math.pi / 4096,
                   lo, hi)


def rd(fn, n=8, tag=''):
    last = None
    for _ in range(n):
        try:
            return fn()
        except Exception as exc:
            last = exc
            time.sleep(0.2)
    raise RuntimeError(f'{tag}: {last}')


def raw_to_rad(raw):
    return (np.array(raw) - zero) * direction * 2 * math.pi / 4096


bus = FeetechSerialBus(default_arm_port(), timeout_s=0.05)
try:
    rd(lambda: (bus.set_torque(True), bus.read_torque_states())[1], tag='使能')
    time.sleep(0.4)
    from_ = pose(sys.argv[1]) if len(sys.argv) > 1 else pose('ready')
    to_ = pose(sys.argv[2]) if len(sys.argv) > 2 else pose('home')

    start = raw_to_rad(rd(bus.read_positions, tag='读位置'))
    dist = float(np.max(np.abs(to_ - start)))
    n = int(np.clip(dist / 0.012, 40, 220))
    f = 0.18
    tau = np.linspace(0.0, 1.0, n)
    v = np.zeros(n)
    for i, t in enumerate(tau):
        if t < f:
            v[i] = (1.0 - np.cos(np.pi * t / f)) / 2.0
        elif t <= 1.0 - f:
            v[i] = 1.0
        else:
            v[i] = (1.0 + np.cos(np.pi * (t - (1.0 - f)) / f)) / 2.0
    p = np.cumsum(v)
    p = p / p[-1]

    t0 = time.monotonic()
    for k in range(1, n + 1):
        q = start + (to_ - start) * float(p[k - 1])
        raw = np.rint(zero + q * direction * 4096 / (2 * math.pi)).astype(int)
        rd(lambda raw=raw: bus.write_positions(raw.tolist()), n=3, tag='写')
        time.sleep(0.03)
    t_stream = time.monotonic() - t0
    rd(lambda: bus.write_positions(
        np.rint(zero + to_ * direction * 4096 / (2 * math.pi)).astype(int).tolist()),
        n=3, tag='终写')
    # 到位后高频采样 2s
    hist = []
    while time.monotonic() - t0 < t_stream + 2.5:
        pos = rd(bus.read_positions, n=5, tag='读')
        hist.append(raw_to_rad(pos))
        time.sleep(0.02)
    bus.close()

    arr = np.array(hist)
    names = ['pan', 'lift', 'elbow', 'wrist_f', 'roll', 'gripper']
    print(f'{n} 采样下发 {t_stream:.2f}s -> 到位后采样 {arr.shape[0]} 次')
    for j, nm in enumerate(names):
        qj = arr[:, j]
        g = to_[j]
        err = qj - g
        os_ = float(np.max(err)) if np.max(err) > 0 else 0.0
        # 回摆：误差序列在收敛过程中的变号次数
        d = np.diff(err)
        ring = int(np.sum(np.sign(d[:-1]) != np.sign(d[1:])))
        # 稳定带：连续 5 个采样都在 ±10mrad
        stable = np.where(
            np.array([np.all(np.abs(qj[i:i + 5] - g) < 0.010)
                      for i in range(len(qj) - 4)]))[0]
        settle = float(stable[0] * 0.02) if len(stable) else None
        hold = qj[int(len(qj) * 0.5):]
        stxt = f'{settle*1000:6.0f}ms' if settle is not None else '  --  '
        print(f'  {nm:8s} 过冲 {os_*1000:6.1f}mrad  回摆 {ring:3d}  '
              f'收敛 {stxt}  稳定后抖动 std={hold.std()*1000:5.2f}mrad')
except Exception as exc:
    print('FAIL:', exc)
    raise
