#!/usr/bin/env python3
"""按用户示范位姿抓取：GOTO_TAUGHT -> 合爪加力 -> 抬起 -> 自动归位。

为什么用示范位姿：模型算出的"固定爪位置"与实物对位始终有共模偏差
（前端也看不出来，因为红十字用同一套 FK）。用户手拖示范把这个偏差一次性
消掉，之后全自动。

无论成败，收尾都：张开爪子（若没夹住）-> 自动回到折叠位。
"""

from project_paths import default_arm_port

from project_paths import arm_source_path, driver_params_path, project_path
import argparse
import json
import math
import os
import sys
import time

import numpy as np
from scipy.optimize import least_squares
import yaml
from pathlib import Path

sys.path.insert(0, arm_source_path())
sys.path.insert(0, os.path.expanduser(
    project_path('qianli_ws/src/qianli_vision/scripts')))
from so101_bringup.servo_protocol import FeetechSerialBus
from gripper_model import GripperModel, JOINTS, FLANGE_LINK

CONFIG = os.path.expanduser(
    driver_params_path())
CFG = os.path.expanduser(project_path('config'))
TABLE_Z = -0.06485


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--squeeze-rad', type=float, default=0.29,
                    help='合爪目标角度（越小越紧）')
    ap.add_argument('--load-pct', type=float, default=8.0)
    ap.add_argument('--lift-mm', type=float, default=40.0)
    ap.add_argument('--report', default='/tmp/grasp_taught.json')
    a = ap.parse_args()

    cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
    zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
    lo = (np.array(cfg['raw_min']) - zero) * direction * 2 * math.pi / 4096
    hi = (np.array(cfg['raw_max']) - zero) * direction * 2 * math.pi / 4096
    lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)

    tg = json.load(open(os.path.join(CFG, 'taught_grasp.json')))
    q_t = np.array(tg['q_rad'])
    hm = json.load(open(os.path.join(CFG, 'home_pose.json')))
    q_home = ((np.array(hm['raw_exec']) - zero) * direction
              * 2 * math.pi / 4096)
    q_home = np.clip(q_home, lo, hi)

    bus = FeetechSerialBus(default_arm_port(), timeout_s=0.15)
    model = GripperModel(stride=8)
    out = {'taught_raw': tg['raw']}

    def rd(fn, n=8, tag=''):
        last = None
        for _ in range(n):
            try:
                return fn()
            except Exception as exc:
                last = exc
                time.sleep(0.2)
        raise RuntimeError(f'{tag} 连续失败: {last}')

    def read():
        return (rd(bus.read_positions, tag='读位置') - zero) \
            * direction * 2 * math.pi / 4096

    def write(q):
        q = np.clip(np.asarray(q, float), lo, hi)
        raw = np.rint(zero + q * direction * 4096 / (2 * math.pi)).astype(int)
        rd(lambda: bus.write_positions(raw.tolist()), tag='写位置')

    def load():
        try:
            return rd(bus.read_gripper_load, tag='读载荷')[0]
        except Exception:
            return 0.0

    def lowest(q):
        return model.lowest_over_all(dict(zip(JOINTS, q)))

    def move(goal, segs=12, tol=0.05, wait=2.0):
        start = read()
        for i in range(1, segs + 1):
            seg = start + (goal - start) * i / segs
            t0 = time.monotonic()
            while time.monotonic() - t0 < wait:
                write(seg)
                time.sleep(0.2)
                if np.max(np.abs(read() - seg)) < tol:
                    break

    try:
        print('力矩:', rd(lambda: (bus.set_torque(True),
                                   bus.read_torque_states())[1], tag='使能'))
        time.sleep(0.5)
        q_now = read()
        print('起点 raw', np.round(q_now, 4).tolist())

        # 途经"示教中间位"（READY）：把 home->抓取位 拆成两段，路径更可控
        rdy_path = os.path.join(CFG, 'ready_pose.json')
        legs = []
        if os.path.exists(rdy_path):
            rp = json.load(open(rdy_path))
            q_ready = np.clip(((np.array(rp['raw_exec']) - zero) * direction
                               * 2 * math.pi / 4096), lo, hi)
            legs.append(('READY(示教中间位)', q_ready))
            print(f'READY 中间位 raw {rp["raw_exec"]}')
        else:
            print('（未示教中间位，直接去抓取位）')
        legs.append(('TAUGHT(示教抓取位)', q_t))

        cur = q_now
        for leg_name, q_leg in legs:
            n = 20
            worst = None
            for i in range(1, n + 1):
                q = cur + (q_leg - cur) * i / n
                low, lk = lowest(q)
                if worst is None or low[2] < worst[0]:
                    worst = (low[2], lk)
            print(f'[{leg_name}] 路径最低 {worst[1]} z={worst[0]*1000:+.2f}mm '
                  f'(桌面 {TABLE_Z*1000:+.2f})')
            if worst[0] < TABLE_Z - 0.003:
                print(f'❌ 到 {leg_name} 的路径低于桌面超过 3mm，中止（不动）')
                out['error'] = f'path below table ({leg_name})'
                return
            move(q_leg)
            print(f'  → 到位 raw {np.round(read(), 4).tolist()}')
            cur = q_leg

        # 合爪加力
        q0 = read()
        load0 = load()
        sq = q0.copy()
        hits, pct = 0, load0
        deadline = time.monotonic() + 15
        while sq[5] > a.squeeze_rad and time.monotonic() < deadline:
            sq[5] = max(a.squeeze_rad, sq[5] - 0.005)
            low, lk = lowest(sq)
            # 网格是抽样近似（stride=8），最低点估计偏保守；
            # 允许到桌面下 2mm 才停，否则夹不住方块
            if low[2] < TABLE_Z - 0.002:
                print(f'  合爪会碰桌面({lk} z={low[2]*1000:+.2f}mm)，停')
                break
            write(sq)
            time.sleep(0.1)
            pct = load()
            hits = hits + 1 if pct >= max(a.load_pct, load0 + 6) else 0
            if hits >= 3:
                break
        out.update(load_pct=float(pct), squeeze_rad=float(sq[5]), hits=hits)
        print(f'合爪: 载荷 {pct:.1f}%  角度 {sq[5]:.4f}rad  hits={hits}')
        held = hits >= 3
        if not held:
            print('⚠ 未检到夹持力（可能没夹住）')
        m = read()
        sq[5] = max(lo[5], m[5] - 0.008)
        write(sq)
        time.sleep(0.4)

        # 抬起：慢速、逐步，载荷掉了就立即停（避免把方块甩飞）
        if held:
            T0 = model.solve(dict(zip(JOINTS, read())))
            p0 = T0['gripper_frame_link'][:3, 3].copy()
            R0 = T0['gripper_frame_link'][:3, :3].copy()
            lifted = 0.0
            for i in range(1, 13):
                tgt = p0 + np.array([0, 0, a.lift_mm / 1000 * i / 12])
                seed = np.clip(read()[:5], lo[:5] + 1e-6, hi[:5] - 1e-6)

                def residual(arm):
                    T = model.solve(dict(zip(JOINTS, np.r_[arm, sq[5]])))
                    F = T['gripper_frame_link']
                    return np.r_[F[:3, 3] - tgt,
                                 0.5 * (F[:3, :3] - R0).ravel()]
                sol = least_squares(residual, seed,
                                    bounds=(lo[:5], hi[:5]), max_nfev=200)
                write(np.r_[sol.x, sq[5]])
                time.sleep(0.5)
                cur = load()
                T1 = model.solve(dict(zip(JOINTS, read())))
                lifted = float((T1['gripper_frame_link'][2, 3] - p0[2]) * 1000)
                print(f'  抬起第{i}步 {lifted:5.1f}mm 载荷 {cur:.1f}%')
                if cur < 3.0:
                    print('  载荷掉了，停止抬升')
                    break
            out['lift_mm'] = lifted
            out['load_after_lift'] = load()
            out['held'] = bool(out['load_after_lift'] >= 3.0)
            print(f'抬起 {lifted:.1f}mm  保持载荷 {out["load_after_lift"]:.1f}%')
        else:
            out['held'] = False
    except BaseException as exc:
        out['error'] = str(exc)
        print(f'STOP: {exc}')
    finally:
        # 收尾：没夹住就张开爪子，然后自动归位
        try:
            if not out.get('held'):
                q = read()
                for g in np.linspace(q[5], 0.58, 15):
                    qq = q.copy()
                    qq[5] = g
                    write(qq)
                    time.sleep(0.08)
                print('已张开爪子')
        except BaseException as exc:
            print('张开失败:', exc)
        try:
            q_now = read()
            q_ret = q_home.copy()
            # 夹住东西时，归位必须**保持夹持角**，否则会把方块松开丢掉
            if out.get('held'):
                q_ret[5] = q_now[5]
                print(f'归位时保持夹持角 {q_now[5]:.4f}rad（不松开）')
            n = 16
            worst = None
            for i in range(1, n + 1):
                q = q_now + (q_ret - q_now) * i / n
                low, lk = lowest(q)
                if worst is None or low[2] < worst[0]:
                    worst = (low[2], lk)
            print(f'归位路径最低 {worst[1]} z={worst[0]*1000:+.2f}mm')
            if worst[0] < TABLE_Z - 0.003:
                print('⚠ 归位路径贴桌（仍在网格误差内），继续慢速归位')
            move(q_ret, segs=16, tol=0.06, wait=2.5)
            end = read()
            print('已自动归位 raw', np.round(end, 4).tolist(),
                  f' 夹爪 {end[5]:.4f}rad 载荷 {load():.1f}%')
            out['returned_home'] = True
            out['load_at_home'] = load()
        except BaseException as exc:
            print('归位失败:', exc)
            out['returned_home'] = False
        Path(a.report).write_text(json.dumps(out, indent=2, ensure_ascii=False))
        bus.close()
        print(json.dumps({k: v for k, v in out.items() if k != 'taught_raw'},
                         indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
