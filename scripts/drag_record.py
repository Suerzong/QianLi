#!/usr/bin/env python3
"""沿一排方格拖拽记录：量出棋盘边的方向与格边长。

用法：--release 卸力 -> 手拖固定爪尖沿一排方格走 6 格（在每个交点稍停）
      -> --record 记录轨迹 -> 自动分析方向/格距/停留点

分析：
  · PCA 求轨迹主方向 = 该边在机械臂坐标系下的方向
  · 按停留聚类得到各"交点"，相邻间距 = 真实格边长
"""

from project_paths import default_arm_port

from project_paths import arm_source_path, driver_params_path
import argparse
import json
import math
import os
import sys
import time

import numpy as np
import yaml
from pathlib import Path

sys.path.insert(0, arm_source_path())
from so101_bringup.servo_protocol import FeetechSerialBus
from gripper_model import GripperModel, JOINTS, FLANGE_LINK

CONFIG = os.path.expanduser(
    driver_params_path())
STOP = '/tmp/drag_stop'


def make_tip():
    model = GripperModel(stride=8)
    T0 = model.solve(dict(zip(JOINTS, np.zeros(6))))
    F0, G0 = T0['gripper_frame_link'], T0[FLANGE_LINK]
    w0 = (G0[:3, :3] @ model.parts[FLANGE_LINK].T).T + G0[:3, 3]
    pf = (F0[:3, :3].T @ (w0 - F0[:3, 3]).T).T
    inner = pf[np.abs(pf[:, 0]) < 0.004]
    p_fix = inner[int(np.argmax(inner[:, 2]))]

    def tip(q):
        T = model.solve(dict(zip(JOINTS, q)))
        F = T['gripper_frame_link']
        return F[:3, 3] + F[:3, :3] @ p_fix
    return tip


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--release', action='store_true')
    g.add_argument('--record', action='store_true')
    ap.add_argument('--duration', type=float, default=90.0)
    ap.add_argument('--start-delay', type=float, default=8.0)
    ap.add_argument('--json', default='/tmp/drag_line.json')
    a = ap.parse_args()

    cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
    zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
    bus = FeetechSerialBus(default_arm_port(), timeout_s=0.08)

    if a.release:
        bus.set_torque(False)
        time.sleep(0.6)
        print('已卸力，可以手拖。力矩:', bus.read_torque_states())
        bus.close()
        return

    tip_of = make_tip()
    if os.path.exists(STOP):
        os.remove(STOP)
    print(f'{a.start_delay:.0f} 秒后开始记录，持续 {a.duration:.0f} 秒')
    print('请把固定爪尖端贴住棋盘纸，沿**一排方格**从一端走到另一端（6 格），')
    print('在每个交点稍停 ~1 秒。中途可 touch /tmp/drag_stop 结束。', flush=True)
    t0 = time.monotonic() + a.start_delay
    while time.monotonic() < t0:
        print(f'\r  准备中… {t0 - time.monotonic():4.1f}s ', end='', flush=True)
        time.sleep(0.2)
    print('\r  开始记录！      ')

    samples = []
    t_end = time.monotonic() + a.duration
    while time.monotonic() < t_end:
        if os.path.exists(STOP):
            break
        try:
            raw = np.array(bus.read_positions())
        except Exception:
            time.sleep(0.1)
            continue
        q = (raw - zero) * direction * 2 * math.pi / 4096
        samples.append(tip_of(q).tolist())
        time.sleep(0.05)
    bus.close()
    if os.path.exists(STOP):
        os.remove(STOP)
    P = np.array(samples)
    print(f'\n共 {len(P)} 样本')
    if len(P) < 50:
        print('样本太少')
        return
    # PCA 主方向（只用 XY）
    XY = P[:, :2]
    c = XY.mean(0)
    U, S, Vt = np.linalg.svd(XY - c, full_matrices=False)
    d = Vt[0]
    t = (XY - c) @ d
    print(f'主方向(单位向量, base XY) = ({d[0]:+.5f}, {d[1]:+.5f})  '
          f'= {math.degrees(math.atan2(d[1], d[0])):+.2f}°')
    print(f'沿该方向跨度 {t.max()-t.min():.1f} mm')
    print(f'垂直方向残差 RMS {np.sqrt(np.mean(((XY-c)@Vt[1])**2)):.2f} mm '
          f'(直线度)')
    # 停留聚类：按相邻样本距离大/小切分
    step = np.linalg.norm(np.diff(XY, axis=0), axis=1) * 1000
    stops, cur = [], [0]
    for i, s in enumerate(step):
        if s < 0.35:
            cur.append(i + 1)
        else:
            if len(cur) >= 6:
                stops.append(cur)
            cur = [i + 1]
    if len(cur) >= 6:
        stops.append(cur)
    print(f'\n检测到 {len(stops)} 个"停留点"（每个交点停一下）：')
    centers = []
    for k, idx in enumerate(stops):
        p = XY[idx].mean(0)
        centers.append(p)
        print(f'  停留{k+1}: ({p[0]:.4f},{p[1]:.4f})  '
              f'{len(idx)} 样本')
    if len(centers) >= 2:
        C = np.array(centers)
        seg = np.linalg.norm(np.diff(C, axis=0), axis=1) * 1000
        print(f'\n相邻停留点间距 (mm): {np.round(seg,1).tolist()}')
        if len(seg):
            print(f'  中位 {np.median(seg):.2f} mm  → 若相邻为 1 格，'
                  f'格边长 ≈ {np.median(seg):.2f} mm')
        print(f'  闭合总长 {(np.linalg.norm(C[-1]-C[0])*1000):.1f} mm  '
              f'→ /6 = {np.linalg.norm(C[-1]-C[0])*1000/6:.2f} mm/格')
    json.dump({'samples': samples, 'direction_xy': d.tolist(),
               'length_mm': float(t.max() - t.min()),
               'straight_rms_mm': float(np.sqrt(np.mean(((XY-c)@Vt[1])**2))),
               'stops_xy': [p.tolist() for p in centers],
               'stops_mm': [float(x * 1000) for x in
                            (np.linalg.norm(np.diff(np.array(centers), axis=0),
                                            axis=1) if len(centers) > 1 else [])]},
              open(a.json, 'w'), indent=2)
    print(f'\n→ {a.json}')


if __name__ == '__main__':
    main()
