#!/usr/bin/env python3
"""运动审计（数字孪生 + 编码器双通道）。

命令侧(纯计算,数字孪生)：用 gripper_model(MuJoCo FK) 校验指令轨迹
    - 关节速度/加速度是否连续(无突变)
    - TCP 路径是否平滑(速度无跳变)
实际侧(编码器)：实测 Present Position
    - 起步爬升曲线(丝滑=平滑爬升, 卡=死寂后跳)
    - 命令动而实际停拍数(阶梯感)
    - 增量均匀度

用法: motion_audit.py [--f 0.13] [--incr 0.016] [--dt 0.03]
"""
import json
import math
import os
import sys
import time

import numpy as np
import yaml
from pathlib import Path

sys.path.insert(0, '/home/ros/legacy/arm/arm-final/ros2_ws/src/so101_bringup')
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from so101_bringup.servo_protocol import FeetechSerialBus
from gripper_model import GripperModel, JOINTS

CONFIG = os.path.expanduser(
    '~/legacy/arm/arm-final/ros2_ws/install/so101_bringup/share/so101_bringup'
    '/config/driver_params.yaml')
CFG = os.path.expanduser('~/QianLi/qianli_ws/config')

F = float(sys.argv[sys.argv.index('--f') + 1]) if '--f' in sys.argv else 0.13
INCR = float(sys.argv[sys.argv.index('--incr') + 1]) \
    if '--incr' in sys.argv else 0.016
DT = float(sys.argv[sys.argv.index('--dt') + 1]) if '--dt' in sys.argv else 0.03

cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
lo = (np.array(cfg['raw_min']) - zero) * direction * 2 * math.pi / 4096
hi = (np.array(cfg['raw_max']) - zero) * direction * 2 * math.pi / 4096
lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)

model = GripperModel(stride=8)


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


def profile(n, f):
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
    return p / p[-1]


def tcp_xyz(q):
    Fm = model.solve(dict(zip(JOINTS, q)))['gripper_frame_link']
    return Fm[:3, 3]


# ---------- 命令侧(数字孪生) ----------
from_ = pose('ready')
to_ = pose('home')
dist = float(np.max(np.abs(to_ - from_)))
n = int(np.clip(dist / INCR, 36, 180))
p = profile(n, F)
q_traj = np.array([from_ + (to_ - from_) * float(pp) for pp in p])
vel = np.diff(q_traj, axis=0) / DT
acc = np.diff(vel, axis=0) / DT
tcp = np.array([tcp_xyz(q) for q in q_traj])
tcp_spd = np.linalg.norm(np.diff(tcp, axis=0), axis=1) / DT
print(f'=== 命令侧(数字孪生) f={F} incr={INCR} n={n} ===')
names = ['pan', 'lift', 'elbow', 'wrist_f', 'roll', 'gripper']
for j, nm in enumerate(names):
    print(f'  {nm:8s} 最大速度 {np.abs(vel[:, j]).max():6.3f} rad/s  '
          f'最大加速度 {np.abs(acc[:, j]).max():6.2f} rad/s²')
print(f'  TCP     最大线速度 {tcp_spd.max():6.3f} m/s  '
      f'速度突变max {np.abs(np.diff(tcp_spd)).max():6.4f} m/s/拍')

# ---------- 实际侧(编码器) ----------
bus = FeetechSerialBus('/dev/ttyACM0', timeout_s=0.05)
try:
    rd(lambda: (bus.set_torque(True), bus.read_torque_states())[1], tag='使能')
    time.sleep(0.4)
    cur = (np.array(rd(bus.read_positions, tag='读位置')) - zero) * \
        direction * 2 * math.pi / 4096
    if np.max(np.abs(cur - from_)) > 0.01:
        n0 = int(np.clip(np.max(np.abs(from_ - cur)) / 0.016, 30, 180))
        p0 = profile(n0, F)
        for k in range(1, n0 + 1):
            q0 = cur + (from_ - cur) * float(p0[k - 1])
            raw0 = rad_to_raw(q0)
            rd(lambda raw=raw0: bus.write_positions(raw.tolist()), n=3, tag='起始走')
            time.sleep(DT)
        time.sleep(0.5)
    start = (np.array(rd(bus.read_positions, tag='读位置')) - zero) * \
        direction * 2 * math.pi / 4096
    rows = []
    t0 = time.monotonic()
    for k in range(1, n + 1):
        q = start + (to_ - start) * float(p[k - 1])
        raw_cmd = rad_to_raw(q)
        rd(lambda raw=raw_cmd: bus.write_positions(raw.tolist()), n=3, tag='写')
        act = rd(bus.read_positions, n=3, tag='读')
        rows.append((time.monotonic() - t0, raw_cmd.tolist(), act))
        time.sleep(DT)
    final = rad_to_raw(to_)
    for _ in range(15):
        rd(lambda: bus.write_positions(final.tolist()), n=3, tag='终写')
        act = rd(bus.read_positions, n=3, tag='读')
        rows.append((time.monotonic() - t0, final.tolist(), act))
        time.sleep(DT)
    bus.close()

    cmd = np.array([r[1] for r in rows], float)
    act = np.array([r[2] for r in rows], float)
    print(f'\n=== 实际侧(编码器) {len(rows)} 采样 ===')
    for j, nm in enumerate(names):
        da = np.abs(np.diff(act[:, j]))
        dcmd = np.diff(cmd[:, j])
        stuck = int(np.sum((np.abs(dcmd) >= 2) & (np.abs(np.diff(act[:, j])) < 1)))
        jit = float(da[da > 0].std()) if np.any(da > 0) else 0.0
        print(f'  {nm:8s} 冻结拍 {stuck:3d}  增量std {jit:4.1f}  '
              f'maxΔ {int(da.max()):4d}  平均滞后 '
              f'{(act[1:n, j]-cmd[1:n, j]).mean():+5.1f}')
    hj = int(np.argmax(np.abs(to_ - start)))
    dact = np.diff(act[:, hj])
    print(f'\n  最活跃关节 {names[hj]} 起步前14拍: '
          + ' '.join(f'{int(x):+d}' for x in dact[:14]))
except Exception as exc:
    print('FAIL:', exc)
    raise
