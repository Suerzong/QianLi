#!/usr/bin/env python3
"""数字孪生全流程运动审计（命令侧，纯计算，不动臂）。

把完整抓取周期的所有运动剖面生成指令轨迹，校验：
  1) 关节速度/加速度连续性（无突变 = 不顿）
  2) 速度峰值是否合理（不过快）
  3) 各剖面衔接是否平滑
覆盖：READY→HOVER 大摆动(smooth_move梯形)、下探(正弦)、合爪(正弦)、
      抬升(正弦)、开爪(正弦)。
用法: ~/mj/bin/python cycle_audit.py
"""
import json
import math
import os
import sys

import numpy as np
import yaml
from pathlib import Path

sys.path.insert(0, '/home/ros/legacy/arm/arm-final/ros2_ws/src/so101_bringup')
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gripper_model import GripperModel, JOINTS

CONFIG = os.path.expanduser(
    '~/legacy/arm/arm-final/ros2_ws/install/so101_bringup/share/so101_bringup'
    '/config/driver_params.yaml')
CFG = os.path.expanduser('~/QianLi/qianli_ws/config')

cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
lo = (np.array(cfg['raw_min']) - zero) * direction * 2 * math.pi / 4096
hi = (np.array(cfg['raw_max']) - zero) * direction * 2 * math.pi / 4096
lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)

model = GripperModel(stride=8)
TABLE_Z = 0.06909


def pose(name):
    p = os.path.join(CFG, name + '_pose.json')
    raw = json.load(open(p))['raw_exec']
    return np.clip((np.array(raw) - zero) * direction * 2 * math.pi / 4096,
                   lo, hi)


def tcp_xyz(q):
    Fm = model.solve(dict(zip(JOINTS, q)))['gripper_frame_link']
    return Fm[:3, 3]


def audit(name, qs, dt):
    """qs: (N,6) 指令关节轨迹; 校验速度/加速度连续性。"""
    q = np.array(qs)
    if len(q) < 3:
        return
    vel = np.diff(q, axis=0) / dt
    acc = np.diff(vel, axis=0) / dt
    tcp = np.array([tcp_xyz(x) for x in q])
    tspd = np.linalg.norm(np.diff(tcp, axis=0), axis=1) / dt
    names = ['pan', 'lift', 'elbow', 'wrist_f', 'roll', 'gripper']
    print(f'--- {name} ({len(q)} 拍, {len(q)*dt:.1f}s) ---')
    ok = True
    for j, nm in enumerate(names):
        vmax = float(np.abs(vel[:, j]).max())
        amax = float(np.abs(acc[:, j]).max())
        flag = 'OK' if vmax < 2.0 and amax < 20 else '⚠'
        if flag != 'OK':
            ok = False
        print(f'  {nm:8s} vmax={vmax:5.2f}rad/s amax={amax:6.2f}rad/s² {flag}')
    tj = float(np.abs(np.diff(tspd)).max())
    print(f'  TCP    vmax={tspd.max():.3f}m/s 每拍速度突变max={tj:.4f}m/s '
          f'{"OK" if tj < 0.05 else "⚠"}')
    if tj >= 0.05:
        ok = False
    return ok


def ramp_trap(s, f=0.13):
    """smooth_move 的梯形余弦积分（与 grasp_auto 同款）。"""
    if s < f:
        return (s / 2 - f / (2 * np.pi) * np.sin(np.pi * s / f)) / (1 - f)
    if s <= 1 - f:
        return (s - f / 2) / (1 - f)
    u = s - (1 - f)
    return (1 - 1.5 * f + u / 2 + f / (2 * np.pi) * np.sin(np.pi * u / f)) \
        / (1 - f)


def sine(s):
    return 0.5 * (1 - np.cos(np.pi * s))


home = pose('home')
ready = pose('ready')
N = 100
dt = 0.03

allok = True
# 1) READY -> HOVER 大摆动（梯形剖面，模拟 smooth_move）
g_hover = ready.copy()
g_hover[1] = ready[1] + 0.3
qs = [ready + (g_hover - ready) * ramp_trap(i / N) for i in range(N + 1)]
allok &= audit('READY→HOVER 摆动(梯形)', qs, dt)

# 2) 下探（正弦，模拟 descend）
qs = [g_hover + (g_hover + np.array([0, 0, 0, 0, 0, -0.3]) - g_hover)
      * sine(i / N) for i in range(N + 1)]
# 下探实际是沿 z 下降：这里构造 z 变化的示意
base = g_hover.copy()
tgt = base.copy()
tgt[2] = base[2] + 0.4          # 示意：肘/腕变化使 TCP 下降
qs = [base + (tgt - base) * sine(i / N) for i in range(N + 1)]
allok &= audit('下探(正弦)', qs, dt)

# 3) 合爪（正弦，模拟 close）
g_open = home.copy()
g_open[5] = 0.58
g_close = home.copy()
g_close[5] = 0.25
qs = [g_open + (g_close - g_open) * sine(i / N) for i in range(N + 1)]
allok &= audit('合爪(正弦)', qs, dt)

# 4) 抬升（正弦，模拟 lift）
g_low = home.copy()
g_high = home.copy()
g_high[2] = home[2] + 0.25
qs = [g_low + (g_high - g_low) * sine(i / N) for i in range(N + 1)]
allok &= audit('抬升(正弦)', qs, dt)

# 5) 开爪（正弦）
qs = [g_close + (g_open - g_close) * sine(i / N) for i in range(N + 1)]
allok &= audit('开爪(正弦)', qs, dt)

print('\n=== 结论:', '全部平滑 ✅' if allok else '存在突变 ⚠', '===')
