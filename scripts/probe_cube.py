#!/usr/bin/env python3
"""触觉测方块宽度：固定爪面停在方块中心，合爪，接触时爪口开度×2 = 方块宽度。

不依赖任何尺寸假设（只依赖视觉给的方块中心 + 爪口开度模型）。
"""
import json
import math
import os
import sys
import time

import numpy as np
from scipy.optimize import least_squares
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
CFG = os.path.expanduser('~/QianLi/qianli_ws/config')
TABLE_Z = -0.06485
DOWN = np.array([0.0, 0.0, -1.0])

ap = __import__('argparse').ArgumentParser()
ap.add_argument('--x', type=float, required=True)
ap.add_argument('--y', type=float, required=True)
ap.add_argument('--depth-mm', type=float, default=-55.0)
ap.add_argument('--step-rad', type=float, default=0.004)
ap.add_argument('--rise-pct', type=float, default=6.0)
a = ap.parse_args()

cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
lo = (np.array(cfg['raw_min']) - zero) * direction * 2 * math.pi / 4096
hi = (np.array(cfg['raw_max']) - zero) * direction * 2 * math.pi / 4096
lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)
fr = json.load(open(os.path.join(CFG, 'board_frame.json')))
A = np.array(fr['affine'])
ey = np.array([A[0, 1], A[1, 1]])
ey = ey / np.linalg.norm(ey)
xax_w = np.array([ey[0], ey[1], 0.0])
yax_w = np.cross(DOWN, xax_w)
R_des = np.column_stack([xax_w, yax_w, DOWN])

bus = FeetechSerialBus('/dev/ttyACM0', timeout_s=0.15)
model = GripperModel(stride=8)


def rd(fn, n=8, tag=''):
    last = None
    for _ in range(n):
        try:
            return fn()
        except Exception as exc:
            last = exc
            time.sleep(0.2)
    raise RuntimeError(f'{tag} {last}')


def read():
    return (rd(bus.read_positions, tag='读位置') - zero) * direction \
        * 2 * math.pi / 4096


def write(q):
    q = np.clip(np.asarray(q, float), lo, hi)
    rd(lambda: bus.write_positions(
        np.rint(zero + q * direction * 4096 / (2 * math.pi)).astype(int).tolist()),
       tag='写位置')


def load():
    try:
        return rd(bus.read_gripper_load, tag='载荷')[0]
    except Exception:
        return 0.0


def fk(q):
    return model.solve(dict(zip(JOINTS, q)))


def solve(target, grip, w=0.6):
    def residual(arm):
        F = fk(np.r_[arm, grip])['gripper_frame_link']
        return np.r_[F[:3, 3] - target, w * (F[:3, :3] - R_des).ravel()]
    best, bc = None, None
    rng = np.random.default_rng(0)
    for s in [np.zeros(5)] + [lo[:5] + rng.random(5) * (hi[:5] - lo[:5])
                              for _ in range(9)]:
        sol = least_squares(residual, np.clip(s, lo[:5] + 1e-6, hi[:5] - 1e-6),
                            bounds=(lo[:5], hi[:5]), max_nfev=400)
        c = np.r_[sol.x, grip]
        e = float(np.linalg.norm(fk(c)['gripper_frame_link'][:3, 3] - target))
        if bc is None or e < bc:
            best, bc = c, e
    return best, bc


print('力矩:', rd(lambda: (bus.set_torque(True), bus.read_torque_states())[1],
                  tag='使能'))
time.sleep(0.4)
# 固定爪面停在方块中心：TCP 正好落在视觉给的方块中心上方
tgt = np.array([a.x, a.y, a.depth_mm / 1000.0 + 0.0063])
q, e = solve(tgt, 0.58)
print(f'定位到方块中心上方 IK 误差 {e*1000:.1f}mm  TCP {np.round(tgt,4).tolist()}')
cur = read()
for i in range(1, 13):
    write(cur + (q - cur) * i / 12)
    time.sleep(0.12)
time.sleep(0.6)
load0 = load()
print(f'起始载荷 {load0:.1f}%   开度 '
      f'{model.jaw_opening(float(read()[5]))*1000:.1f}mm')
sq = read()
hit = None
deadline = time.monotonic() + 25
while sq[5] > 0.16 and time.monotonic() < deadline:
    sq[5] = max(0.16, sq[5] - a.step_rad)
    write(sq)
    time.sleep(0.09)
    pct = load()
    op = model.jaw_opening(float(sq[5]))
    if pct >= load0 + a.rise_pct:
        hit = (float(sq[5]), op, pct)
        break
if hit is None:
    print(f'❌ 未检测到接触（最终载荷 {load():.1f}%，开度 '
          f'{model.jaw_opening(float(read()[5]))*1000:.1f}mm）')
else:
    ang, op, pct = hit
    print(f'✅ 接触! 角度 {ang:.4f}rad  爪口开度 {op*1000:.2f}mm  '
          f'载荷 {pct:.1f}%')
    print(f'   → 方块宽度 ≈ 2 × 开度 = {op*2000:.1f} mm')
    json.dump({'width_mm': op * 2000, 'jaw_opening_mm': op * 1000,
               'angle_rad': ang, 'load_pct': pct,
               'cube_xy': [a.x, a.y], 'depth_mm': a.depth_mm},
              open('/tmp/cube_width.json', 'w'), indent=2)
    print('   → /tmp/cube_width.json')
# 张开并归位
q = read()
for g in np.linspace(q[5], 0.58, 10):
    qq = q.copy()
    qq[5] = g
    write(qq)
    time.sleep(0.06)
hm = json.load(open(os.path.join(CFG, 'home_pose.json')))
q_home = np.clip((np.array(hm['raw_exec']) - zero) * direction
                 * 2 * math.pi / 4096, lo, hi)
cur = read()
for i in range(1, 17):
    write(cur + (q_home - cur) * i / 16)
    time.sleep(0.1)
print('已归位')
bus.close()
