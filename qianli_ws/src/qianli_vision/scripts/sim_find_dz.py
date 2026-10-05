#!/usr/bin/env python3
"""用引擎精确找出"物块正好在两片爪之间"的抓取高度 dz

方法：
  对一系列 dz（TCP 相对物块中心的下移量）：
    · 把臂摆到 TCP = 物块 + dz，夹爪张开
    · 用 mj_geomDistance 分别求：
        d_fixed = 固定侧（gripper_link 的爪零件）到物块的最小距离
        d_move  = 活动侧（moving_jaw 的爪零件）到物块的最小距离
    · 两侧都最小时 → 物块正好卡在两片爪中间 = 正确抓取高度

用法：
  ~/mj/bin/python sim_find_dz.py --obj-size 0.020
"""

import argparse
import sys

import numpy as np

import mujoco

import sim_grasp as S
import sim_mesh_gripper as MG


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--obj-size', type=float, default=0.020)
    ap.add_argument('--lo', type=float, default=-0.080)
    ap.add_argument('--hi', type=float, default=0.000)
    ap.add_argument('--step', type=float, default=0.005)
    ap.add_argument('--roll', type=float, default=0.0)
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

    fix_parts = [i for i in range(model.ngeom)
                 if model.geom_bodyid[i] == gl
                 and (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
                      or '').startswith('pg')]
    mov_parts = [i for i in range(model.ngeom)
                 if model.geom_bodyid[i] == mjb
                 and (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
                      or '').startswith('pg')]
    print(f'物块 {a.obj_size*1000:.0f}mm；固定侧零件 {len(fix_parts)} 块，'
          f'活动侧 {len(mov_parts)} 块')

    ch, names = S.make_chain()
    op = S.obj_world_pos()
    seed = np.zeros(len(ch.links))
    print('\n   dz(mm)   固定侧距离(mm)  活动侧距离(mm)  两侧之和  判定')
    rows = []
    dz = a.lo
    while dz <= a.hi + 1e-9:
        sol, _ = S.solve_ik(ch, names, [op[0], op[1], op[2] + dz], -90.0, seed)
        sol[names.index('wrist_roll')] = a.roll
        for j in S.ARM_JOINTS:
            data.qpos[qadr[j]] = sol[names.index(j)]
        data.qpos[qadr['gripper']] = 1.2
        data.qpos[qadr['gripper']] = 1.2
        mujoco.mj_forward(model, data)
        df = min(mujoco.mj_geomDistance(model, data, cg, p, 1.0, None)
                 for p in fix_parts)
        dm = min(mujoco.mj_geomDistance(model, data, cg, p, 1.0, None)
                 for p in mov_parts)
        tot = df + dm
        tag = ''
        if df < 0.002 and dm < 0.002:
            tag = '★ 两侧都进 2mm'
        elif df < 0.015 and dm < 0.015:
            tag = '两侧都在 15mm 内'
        rows.append((dz, df, dm, tot, tag))
        print(f'  {dz*1000:+7.1f}   {df*1000:14.1f}  {dm*1000:14.1f}  '
              f'{tot*1000:8.1f}  {tag}', flush=True)
        dz += a.step
    best = min(rows, key=lambda r: r[3])
    print(f'\n最佳 dz = {best[0]*1000:+.1f}mm '
          f'(固定侧 {best[1]*1000:.1f}mm, 活动侧 {best[2]*1000:.1f}mm)')


if __name__ == '__main__':
    main()
