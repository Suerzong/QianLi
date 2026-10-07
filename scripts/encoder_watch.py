#!/usr/bin/env python3
"""编码器示波器：流式下发一条轨迹，密集采样 Present Position。

记录每拍的 (t, 命令raw, 实际raw) 到 /tmp/enc_trace.csv，
并当场输出阶梯感指标：
  - 每拍实际增量分布（max/mean）
  - "到位停拍"次数：实际几乎=该拍命令（说明追上了中间目标 -> 阶梯）
  - 命令 vs 实际滞后
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

DT = float(sys.argv[1]) if len(sys.argv) > 1 else 0.03   # 每拍间隔


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
            time.sleep(0.15)
    raise RuntimeError(f'{tag}: {last}')


def rad_to_raw(q):
    return np.rint(zero + q * direction * 4096 / (2 * math.pi)).astype(int)


bus = FeetechSerialBus(default_arm_port(), timeout_s=0.05)
try:
    rd(lambda: (bus.set_torque(True), bus.read_torque_states())[1], tag='使能')
    time.sleep(0.4)
    from_ = pose('ready')
    to_ = pose('home')

    # 先真正走到起始位（不然"起始"就是当前位置，轨迹=没动，全是假数据）
    cur = (np.array(rd(bus.read_positions, tag='读位置')) - zero) * \
        direction * 2 * math.pi / 4096
    if np.max(np.abs(cur - from_)) > 0.01:
        n0 = int(np.clip(np.max(np.abs(from_ - cur)) / 0.012, 30, 200))
        for k in range(1, n0 + 1):
            q0 = cur + (from_ - cur) * k / n0
            raw0 = rad_to_raw(q0)
            rd(lambda raw=raw0: bus.write_positions(raw.tolist()), n=3, tag='起始走')
            time.sleep(0.03)
        time.sleep(0.5)
    start = (np.array(rd(bus.read_positions, tag='读位置')) - zero) * \
        direction * 2 * math.pi / 4096
    dist = float(np.max(np.abs(to_ - start)))
    F = float(os.environ.get('FACC', '0.18'))       # 加速段占比
    INCR = float(os.environ.get('INCR', '0.012'))   # 每拍覆盖弧度
    n = int(np.clip(dist / INCR, 40, 260))
    f = F
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

    rows = []
    t0 = time.monotonic()
    FLOOR = int(os.environ.get('FLOOR', '0'))      # 每拍最小增量(计数)，0=不量化
    last_sent = start
    for k in range(1, n + 1):
        q = start + (to_ - start) * float(p[k - 1])
        raw_ideal = rad_to_raw(q)
        if FLOOR > 0:
            # 量化到 FLOOR 计数网格（单调轨迹：取整即每拍 0 或 >=FLOOR）
            raw_sent = np.round(raw_ideal / FLOOR) * FLOOR
            # 端点夹紧：不越过目标
            raw_end = rad_to_raw(to_)
            raw_sent = np.minimum(raw_sent, raw_end) \
                if np.all(raw_end >= raw_sent) else np.maximum(raw_sent, raw_end)
            raw_cmd = raw_sent.astype(int)
        else:
            raw_cmd = raw_ideal.astype(int)
        rd(lambda raw=raw_cmd: bus.write_positions(raw.tolist()), n=3, tag='写')
        act = rd(bus.read_positions, n=3, tag='读')
        rows.append((time.monotonic() - t0, raw_cmd.tolist(), act))
        last_sent = raw_cmd
        time.sleep(DT)
    # 终位保持采样
    final = rad_to_raw(to_)
    for _ in range(20):
        rd(lambda: bus.write_positions(final.tolist()), n=3, tag='终写')
        act = rd(bus.read_positions, n=3, tag='读')
        rows.append((time.monotonic() - t0, final.tolist(), act))
        time.sleep(DT)
    bus.close()

    with open('/tmp/enc_trace.csv', 'w') as fh:
        fh.write('t,cmd0..cmd5,act0..act5\n')
        for r in rows:
            fh.write(f'{r[0]:.4f},"{r[1]}","{r[2]}"\n')

    t = np.array([r[0] for r in rows])
    cmd = np.array([r[1] for r in rows], float)
    act = np.array([r[2] for r in rows], float)
    names = ['pan', 'lift', 'elbow', 'wrist_f', 'roll', 'gripper']
    n_stream = n
    print(f'拍间隔 {DT}s 下发 {n_stream} 拍 共 {len(rows)} 个采样点 '
          f'({t[-1]:.2f}s)')
    for j, nm in enumerate(names):
        da = np.abs(np.diff(act[:, j]))
        dcmd = np.diff(cmd[:, j])
        cmd_moving = np.abs(dcmd) >= 2
        act_stuck = np.abs(np.diff(act[:, j])) < 1
        stuck = int(np.sum(cmd_moving & act_stuck))   # 命令在动实际停住=阶梯
        # 响应不均匀度：实际增量序列的标准差（抖动）
        jit = float(da[da > 0].std()) if np.any(da > 0) else 0.0
        lag = act[1:n_stream, j] - cmd[1:n_stream, j]
        print(f'  {nm:8s} 实际每拍Δ max={int(da.max()):4d} '
              f'mean={da.mean():5.1f} 命令动而实际停 {stuck:3d}拍 '
              f'增量抖动std={jit:4.1f} 平均滞后 '
              f'{lag.mean():+5.1f} 计数  max|滞后| {int(np.abs(lag).max()):4d}')
    # 起步曲线：最活跃关节的前 14 拍实际增量（看是否平滑爬升还是猛蹬）
    hj = int(np.argmax(np.abs(to_ - start)))
    dact = np.diff(act[:, hj])
    print(f'\n  最活跃关节 {names[hj]} 起步前14拍实际增量: '
          + ' '.join(f'{int(x):+d}' for x in dact[:14]))
except Exception as exc:
    print('FAIL:', exc)
    raise
