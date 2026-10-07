#!/usr/bin/env python3
"""重做 grid→base 外参（直连串口读关节角，不需要 ROS）。

修正零点后 FK 应已准确，所以这次用**刚性拟合**验收：
  · 刚性 RMS 若降到几毫米、反推格宽≈33mm（且两个方向一致），说明零位修对了
  · 同时给仿射结果对比

固定爪顶端 = gripper_link 网格最低点（与之前标定同一口径）。
每点要求 |爪尖z − 板面z| ≤ z_tol，否则拒收（"贴到板面"才算数）。
"""

from project_paths import default_arm_port

from project_paths import arm_source_path, calibration_path, driver_params_path, project_path
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, arm_source_path())
sys.path.insert(0, os.path.expanduser(
    project_path('qianli_ws/src/qianli_vision/scripts')))
from so101_bringup.servo_protocol import FeetechSerialBus
from gripper_model import GripperModel, JOINTS

CONFIG = os.path.expanduser(
    driver_params_path())
TRIGGER = '/tmp/grid_mark'


def solve_planar(G, B):
    gc, bc = G.mean(axis=0), B.mean(axis=0)
    Gd, Bd = G - gc, B - bc
    num = float(np.sum(Gd[:, 0] * Bd[:, 1] - Gd[:, 1] * Bd[:, 0]))
    den = float(np.sum(Gd[:, 0] * Bd[:, 0] + Gd[:, 1] * Bd[:, 1]))
    th = math.atan2(num, den)
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, -s], [s, c]])
    t = bc - R @ gc
    res = (R @ G.T).T + t - B
    return th, t, res


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--points', default='0,0;9.9,0;0,6.6;9.9,6.6;3.3,3.3')
    # 板面高度：2026-10-06 拖拽标定实测（固定爪尖端贴桌面滑动 694 样本，
    # 平面残差 RMS 0.073mm）→ 桌面 -64.85mm，棋盘纸 +0.5mm。
    # 旧值 -69.09mm 是按"整爪最低点"口径拟合的，与"固定爪顶端"口径差 4.2mm，
    # 会导致 z 判据系统性误拒。
    ap.add_argument('--table-z', type=float, default=-0.06485)
    ap.add_argument('--board-mm', type=float, default=0.5)
    ap.add_argument('--z-tol-mm', type=float, default=3.5)
    ap.add_argument('--resume', action='store_true',
                    help='沿用已记录的点，跳过它们')
    ap.add_argument('--cell-cm', type=float, default=3.3)
    ap.add_argument('--json', default=calibration_path('extrinsic_marks_new.json'))
    ap.add_argument('--out', default=calibration_path('extrinsic_new.txt'))
    a = ap.parse_args()

    pts = []
    for seg in a.points.split(';'):
        x, y = seg.split(',')
        pts.append((float(x), float(y)))

    cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
    zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
    print(f'零点 {zero.tolist()}')
    print(f'限位 raw_min {cfg["raw_min"]}  raw_max {cfg["raw_max"]}')

    bus = FeetechSerialBus(default_arm_port(), timeout_s=0.08)
    model = GripperModel(stride=10)
    board_z = a.table_z + a.board_mm / 1000.0
    marks = []
    done = set()
    if a.resume and os.path.exists(a.json):
        try:
            marks = json.load(open(a.json))
            done = {tuple(m['grid_cm']) for m in marks}
            print(f'续打：已有 {len(marks)} 点 {sorted(done)}')
        except Exception as exc:
            print(f'续打读取失败({exc})，从头开始')
            marks, done = [], set()

    def read_q():
        last = None
        for _ in range(6):
            try:
                raw = np.array(bus.read_positions())
                return (raw - zero) * direction * 2 * math.pi / 4096
            except Exception as exc:          # 串口偶发超时，重试
                last = exc
                time.sleep(0.15)
        raise RuntimeError(f'读关节角失败: {last}')

    def tip_of(q):
        T = model.solve(dict(zip(JOINTS, q)))
        M = T['gripper_link']
        pts_g = model.parts['gripper_link']
        w = (M[:3, :3] @ pts_g.T).T + M[:3, 3]
        k = int(np.argmin(w[:, 2]))
        return w[k]

    if os.path.exists(TRIGGER):
        os.remove(TRIGGER)
    print()
    print('=' * 70)
    print(f'  重打外参：共 {len(pts)} 点  板面 z = {board_z*1000:+.2f} mm')
    print('  每点：把**固定爪顶端**压到对应格点的板面上，然后我触发记录')
    print('=' * 70)
    for i, (gx, gy) in enumerate(pts, 1):
        if (gx, gy) in done:
            print(f'\n▶ 第 {i}/{len(pts)} 点：grid ({gx}, {gy}) cm —— 已有记录，跳过',
                  flush=True)
            continue
        print(f'\n▶ 第 {i}/{len(pts)} 点：grid ({gx}, {gy}) cm', flush=True)
        while not os.path.exists(TRIGGER):
            time.sleep(0.1)
        os.remove(TRIGGER)
        time.sleep(0.15)
        q = read_q()
        tip = tip_of(q)
        zerr = (tip[2] - board_z) * 1000
        if abs(zerr) > a.z_tol_mm:
            print(f'  ⛔ 爪尖 z={tip[2]*1000:+.2f}mm，板面 {board_z*1000:+.2f}mm，'
                  f'差 {zerr:+.2f}mm（容差 ±{a.z_tol_mm}）—— 没贴到板面，不记录',
                  flush=True)
            continue
        marks.append({'grid_cm': [gx, gy], 'contact_m': tip.tolist(),
                      'joints': dict(zip(JOINTS, q.tolist())),
                      'z_err_mm': float(zerr)})
        Path(a.json).write_text(json.dumps(marks, indent=2))
        print(f'  ✅ 固定爪顶端 ({tip[0]:.4f}, {tip[1]:.4f}, {tip[2]*1000:+.2f}mm)'
              f'  z-板面 {zerr:+.2f}mm', flush=True)
    bus.close()

    if len(marks) < 3:
        print(f'\n点数不足（{len(marks)}），无法拟合')
        return 1

    G = np.array([m['grid_cm'] for m in marks], float) / 100.0
    B = np.array([m['contact_m'][:2] for m in marks], float)
    th, t, res = solve_planar(G, B)
    rms_r = float(np.sqrt(np.mean(np.sum(res ** 2, axis=1)))) * 1000
    print('\n' + '=' * 70)
    print('  刚性拟合（4DOF）—— 零位修对的话这里应该很小')
    print('=' * 70)
    print(f'  θ = {math.degrees(th):+.3f}°   origin = ({t[0]:.4f}, {t[1]:.4f})')
    print(f'  残差 RMS = {rms_r:.2f} mm')
    for m, r in zip(marks, res):
        g = str(m['grid_cm'])
        print(f'    grid {g:>12}: ({r[0]*1000:+7.2f}, {r[1]*1000:+7.2f}) mm')

    # 反推格宽（沿两条边）
    d = {}
    for m in marks:
        d[tuple(m['grid_cm'])] = np.array(m['contact_m'][:2])
    print('\n  反推格宽:')
    keys = list(d)
    for g1 in keys:
        for g2 in keys:
            if g1 == g2:
                continue
            dg = np.hypot(g2[0] - g1[0], g2[1] - g1[1]) / a.cell_cm
            if dg < 0.9:
                continue
            dist = np.linalg.norm(d[g2] - d[g1]) * 1000
            print(f'    {g1} -> {g2}: {dist:6.1f}mm / {dg:.1f}格 '
                  f'= {dist/dg:.2f} mm/格')

    X = np.hstack([G, np.ones((len(G), 1))])
    coef, *_ = np.linalg.lstsq(X, B, rcond=None)
    res_a = X @ coef - B
    rms_a = float(np.sqrt(np.mean(np.sum(res_a ** 2, axis=1)))) * 1000
    A = coef[:2].T
    ang = math.degrees(math.acos(np.clip(
        A[:, 0] @ A[:, 1] / (np.linalg.norm(A[:, 0]) * np.linalg.norm(A[:, 1])),
        -1, 1)))
    print(f'\n  仿射拟合（6DOF）: RMS = {rms_a:.2f} mm')
    print(f'    |x列|={np.linalg.norm(A[:,0]):.4f}  |y列|={np.linalg.norm(A[:,1]):.4f}'
          f'  夹角={ang:.2f}° (理想 90)')
    print(f'    → 列长若≈1.0 且夹角≈90°，说明棋盘格在 x/y 上等宽，'
          f'之前的各向异性是零位误差造成的假象')

    Path(a.out).write_text(
        f'# grid→base 外参（零点修正后重打，{time.strftime("%Y-%m-%d %H:%M:%S")}）\n'
        f'# 点数 {len(marks)}  刚性RMS {rms_r:.3f}mm  仿射RMS {rms_a:.3f}mm\n'
        f'grid_origin_x={t[0]:.6f}\ngrid_origin_y={t[1]:.6f}\n'
        f'grid_theta_deg={math.degrees(th):.6f}\n'
        f'cell_cm={a.cell_cm}\nrms_mm={rms_r:.3f}\n'
        f'affine_A={json.dumps(A.tolist())}\naffine_b={json.dumps(coef[2].tolist())}\n')
    print(f'\n  结果 → {a.out}')
    print(f'  打点 → {a.json}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
