#!/usr/bin/env python3
"""三维距离扫描：找出"固定爪刚好贴住物块面、活动爪最近"的抓取位置

为什么用距离而不是接触：
  · 接触只在"已接触/穿插"时出现，分不清"刚好贴上"和"插进去 5mm"
  · mj_geomDistance 给出精确距离 → 可以精确找"贴住但不穿插"的位置
  · 之前失败的根因就是：按闭合状态算的偏移，在张开状态下固定爪已经插进物块，
    一动就把物块顶跑了

判据：
  d_fix = 物块到"固定指尖区"的最小距离   → 目标 0.3~1.0mm（刚好贴上）
  d_mov = 物块到"活动指尖区"的最小距离   → 越小越好（活动爪一合就碰到）
  在 d_fix 达标的候选里，选 d_mov 最小的

用法：
  ~/mj/bin/python sim_scan_touch.py --obj-size 0.020
  ~/mj/bin/python sim_scan_touch.py --obj-size 0.020 --grasp   # 直接试抓最优解
"""

import argparse
import sys

import numpy as np

import mujoco

import sim_grasp as S
import sim_fixed_jaw as FJ
import sim_mesh_gripper as MG
import sim_ik_dls as IK


def tip_geoms(model, data, body_id, band=0.035):
    lo = 1e9
    for i in range(model.ngeom):
        if model.geom_bodyid[i] != body_id:
            continue
        R = data.geom_xmat[i].reshape(3, 3)
        half = np.abs(R) @ model.geom_size[i][:3]
        lo = min(lo, data.geom_xpos[i][2] - half[2])
    ids = []
    for i in range(model.ngeom):
        if model.geom_bodyid[i] != body_id:
            continue
        R = data.geom_xmat[i].reshape(3, 3)
        half = np.abs(R) @ model.geom_size[i][:3]
        if data.geom_xpos[i][2] - half[2] < lo + band:
            ids.append(i)
    return ids


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--obj-size', type=float, default=0.020)
    ap.add_argument('--angle', type=float, default=0.6)
    ap.add_argument('--span', type=float, default=24.0)
    ap.add_argument('--step', type=float, default=2.0)
    ap.add_argument('--grasp', action='store_true')
    a = ap.parse_args()

    model = MG.build(a.obj_size)
    data = mujoco.MjData(model)
    qadr = {}
    for i in range(model.njnt):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if n:
            qadr[n] = model.jnt_qposadr[i]
    gl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper_link')
    mjb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                            'moving_jaw_so101_v1_link')
    cg = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    cq = S.attach_handles(model)[3]
    op = S.obj_world_pos()

    IK.ik_dls(model, data, qadr, op)
    data.qpos[qadr['gripper']] = a.angle
    data.qpos[cq:cq + 3] = op
    data.qpos[cq + 3:cq + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    fix_tip = tip_geoms(model, data, gl)
    mov_tip = tip_geoms(model, data, mjb)
    print(f'张开角 {a.angle}：固定指尖 {len(fix_tip)} 块，活动指尖 {len(mov_tip)} 块')
    print(f'扫描 ±{a.span}mm 步长 {a.step}mm（TCP 不动，挪物块 = 等价挪爪）')

    vals = np.arange(-a.span, a.span + 1e-9, a.step)
    rows = []
    for dx in vals:
        for dy in vals:
            for dz in vals:
                data.qpos[cq:cq + 3] = op + np.array([dx, dy, dz]) / 1000.0
                mujoco.mj_forward(model, data)
                df = min(mujoco.mj_geomDistance(model, data, cg, p, 0.5, None)
                         for p in fix_tip)
                dm = min(mujoco.mj_geomDistance(model, data, cg, p, 0.5, None)
                         for p in mov_tip)
                rows.append((dx, dy, dz, df, dm))
    good = [r for r in rows if -0.0005 <= r[3] <= 0.0012]
    print(f'\n"固定爪刚好贴上"的候选: {len(good)} 个')
    good.sort(key=lambda r: r[4])
    print('  dx(mm)  dy(mm)  dz(mm)   固定爪距(mm)  活动爪距(mm)')
    for r in good[:12]:
        print(f'  {r[0]:+6.1f} {r[1]:+6.1f} {r[2]:+6.1f}   {r[3]*1000:10.2f}  '
              f'{r[4]*1000:10.2f}')
    if not good:
        print('  没有"刚好贴上"的候选，放宽范围看最小固定爪距:')
        rows.sort(key=lambda r: abs(r[3]))
        for r in rows[:6]:
            print(f'  {r[0]:+6.1f} {r[1]:+6.1f} {r[2]:+6.1f}   '
                  f'{r[3]*1000:10.2f}  {r[4]*1000:10.2f}')
        return
    best = good[0]
    tcp = op - np.array(best[:3]) / 1000.0
    print(f'\n最优: 物块偏移({best[0]:+.1f},{best[1]:+.1f},{best[2]:+.1f})mm '
          f'→ TCP 目标 = {np.round(tcp, 4)}')
    np.save('/tmp/touch_offset.npy', np.array(best[:3]))

    if a.grasp:
        print('\n直接试抓该位置:')
        for ang in (0.6, 0.9, 1.2, 0.4):
            m2 = MG.build(a.obj_size)
            d2 = mujoco.MjData(m2)
            q2, a2 = {}, {}
            for i in range(m2.njnt):
                n = mujoco.mj_id2name(m2, mujoco.mjtObj.mjOBJ_JOINT, i)
                if n:
                    q2[n] = m2.jnt_qposadr[i]
            for i in range(m2.nu):
                a2[mujoco.mj_id2name(m2, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                     i).replace('servo_', '')] = i
            oz, oz0, ga = FJ.run(m2, d2, q2, a2, tcp, a.obj_size,
                                 approach_angle=ang)
            print(f'  接近角 {ang}: {"✅ 抓起" if oz > oz0 + 0.005 else "❌ 没抓起"}'
                  f'  终角 {ga:+.3f}  升高 {(oz-oz0)*1000:+.1f}mm', flush=True)


if __name__ == '__main__':
    main()
