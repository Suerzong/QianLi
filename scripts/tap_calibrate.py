#!/usr/bin/env python3
"""拖动标定：手拖固定爪尖端在桌面上滑动，采样 FK 位置。

用途
----
1. 定出**桌面平面**（拟合 z = a·x + b·y + c），供下探安全高度使用
2. **自检 FK 精度**：贴平桌面滑动时，若模型与现实 1:1，FK 的 z 应恒定，
   拟合残差 RMS 应在毫米级；残差大就说明模型有问题（会打印出来）
3. 记录覆盖的 XY 范围，供判断工作空间

用法：先启动（扭矩必须为 0），然后手拖指尖贴桌面来回滑动，结束后
      touch /tmp/tap_stop 或等 --duration 到点。
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
import yaml
from pathlib import Path

sys.path.insert(0, '/home/ros/legacy/arm/arm-final/ros2_ws/src/so101_bringup')
sys.path.insert(0, os.path.expanduser(
    '~/QianLi/qianli_ws/src/qianli_vision/scripts'))
from so101_bringup.servo_protocol import FeetechSerialBus
from gripper_model import GripperModel, JOINTS

CONFIG = os.path.expanduser(
    '~/legacy/arm/arm-final/ros2_ws/install/so101_bringup/share/so101_bringup'
    '/config/driver_params.yaml')
STOP = '/tmp/tap_stop'


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--duration', type=float, default=120.0)
    ap.add_argument('--start-delay', type=float, default=8.0)
    ap.add_argument('--rate-hz', type=float, default=20.0)
    ap.add_argument('--json', default='/tmp/tap_samples.json')
    a = ap.parse_args()

    cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
    zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
    bus = FeetechSerialBus('/dev/ttyACM0', timeout_s=0.08)
    model = GripperModel(stride=14)
    gpts = model.parts['gripper_link']

    if os.path.exists(STOP):
        os.remove(STOP)

    torque = bus.read_torque_states()
    print(f'扭矩状态 {torque}')
    if any(torque):
        print('❌ 扭矩未全关，手拖不动；请先卸力')
        bus.close()
        return 1

    print('=' * 68)
    print('  拖动标定：请用手抓住机械臂，让**固定爪尖端贴着桌面滑动**')
    print('=' * 68)
    print(f'  {a.start_delay:.0f} 秒后开始记录，持续 {a.duration:.0f} 秒')
    print('  · 在棋盘周围的桌面上左右、前后各划几趟（覆盖你关心的区域）')
    print('  · 尽量让爪尖**一直贴住桌面**，别抬起来')
    print('  · 想提前结束：touch /tmp/tap_stop')
    print('=' * 68, flush=True)

    t0 = time.monotonic() + a.start_delay
    while time.monotonic() < t0:
        print(f'\r  准备中… {t0 - time.monotonic():4.1f}s ', end='', flush=True)
        time.sleep(0.2)
    print('\r  开始记录！        ')

    samples = []
    fails = 0
    dt = 1.0 / a.rate_hz
    t_end = time.monotonic() + a.duration
    t_next = time.monotonic() + 2
    while time.monotonic() < t_end:
        if os.path.exists(STOP):
            break
        try:
            raw = np.array(bus.read_positions())
        except Exception:
            fails += 1
            time.sleep(0.1)
            continue
        q = (raw - zero) * direction * 2 * math.pi / 4096
        T = model.solve(dict(zip(JOINTS, q)))
        M = T['gripper_link']
        w = (M[:3, :3] @ gpts.T).T + M[:3, 3]
        k = int(np.argmin(w[:, 2]))
        samples.append(w[k].tolist())
        now = time.monotonic()
        if now >= t_next:
            t_next = now + 3
            P = np.array(samples)
            print(f'  样本 {len(samples):5d}  剩余 {t_end-now:5.1f}s  '
                  f'读失败 {fails}  z: 均值 {P[:,2].mean()*1000:+.2f} '
                  f'极差 {(P[:,2].max()-P[:,2].min())*1000:5.2f}mm  '
                  f'XY 覆盖 {(P[:,0].max()-P[:,0].min())*1000:.0f}×'
                  f'{(P[:,1].max()-P[:,1].min())*1000:.0f}mm', flush=True)
        time.sleep(dt)
    bus.close()
    if os.path.exists(STOP):
        os.remove(STOP)

    P = np.array(samples)
    print(f'\n共 {len(P)} 样本，读失败 {fails}')
    if len(P) < 30:
        print('样本太少')
        return 1
    # 拟合 z = a x + b y + c
    A = np.hstack([P[:, :2], np.ones((len(P), 1))])
    coef, *_ = np.linalg.lstsq(A, P[:, 2], rcond=None)
    pred = A @ coef
    res = (P[:, 2] - pred) * 1000
    rms = float(np.sqrt(np.mean(res ** 2)))
    print('\n' + '=' * 68)
    print('  桌面平面拟合  z = a·x + b·y + c')
    print('=' * 68)
    print(f'  a = {coef[0]:+.5f}   b = {coef[1]:+.5f}   c = {coef[2]*1000:+.3f} mm')
    tilt = math.degrees(math.atan(math.hypot(coef[0], coef[1])))
    print(f'  平面倾角 {tilt:.3f}°')
    print(f'  残差 RMS {rms:.3f} mm   最大 {np.abs(res).max():.3f} mm')
    print(f'  z 原始极差 {(P[:,2].max()-P[:,2].min())*1000:.2f} mm')
    print(f'  XY 覆盖 x {(P[:,0].min())*1000:+.0f}..{(P[:,0].max())*1000:+.0f}mm  '
          f'y {(P[:,1].min())*1000:+.0f}..{(P[:,1].max())*1000:+.0f}mm')
    cx, cy = P[:, 0].mean(), P[:, 1].mean()
    zc = coef[0] * cx + coef[1] * cy + coef[2]
    print(f'  工作面中心 ({cx:.4f},{cy:.4f}) 处平面高度 z = {zc*1000:+.3f} mm')
    print(f'\n  → 残差 RMS 若在 ~2mm 内，说明 FK 与现实的竖直一致性良好；')
    print(f'    若明显更大，说明模型竖直方向不可信，需要先修模型')
    json.dump({'samples': samples, 'plane': coef.tolist(), 'rms_mm': rms,
               'tilt_deg': tilt, 'z_at_center_mm': float(zc * 1000),
               'xy_extent_mm': [[float(P[:,0].min()), float(P[:,0].max())],
                                [float(P[:,1].min()), float(P[:,1].max())]]},
              open(a.json, 'w'), indent=2)
    print(f'  样本 → {a.json}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
