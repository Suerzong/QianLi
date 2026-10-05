#!/usr/bin/env python3
"""在孪生里搜索抓取偏移：扫描 TCP 目标相对物块的水平偏移，找能夹住的组合

背景：
  诊断发现物块在夹爪坐标系里 X=-20.7mm（开合方向）—— TCP 原点不在爪口中心，
  所以夹爪从旁边擦过。与其解析复杂网格，不如让仿真直接扫出正确偏移。

扫描量：沿 base 的 X/Y 两个方向的偏移（±30mm），其余参数固定。
输出：每个偏移下 是否夹住 / 爪-物块接触数 / 物块位移

用法：
  ~/mj/bin/python sim_sweep_offset.py --obj-size 0.014
  ~/mj/bin/python sim_sweep_offset.py --axis y --span 30 --step 5
"""

import argparse
import math
import sys

import numpy as np

import mujoco

sys.argv = [sys.argv[0]]
import sim_grasp as S   # noqa: E402


def trial(model, data, ch, names, dx, dy, dz, obj_size, fast=True):
    qadr, aadr, cube, cq = S.attach_handles(model)
    op = S.obj_world_pos()
    z = S.TABLE_Z + 0.003 + obj_size / 2
    gl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper_link')
    cg = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    arm_ids = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n) for n in
               ['shoulder_link', 'upper_arm_link', 'lower_arm_link',
                'wrist_link', 'gripper_link', 'moving_jaw_so101_v1_link']}
    tx, ty, tz = op[0] + dx, op[1] + dy, z + dz

    def arm_contacts():
        n = 0
        for c in range(data.ncon):
            cn = data.contact[c]
            if cg in (cn.geom1, cn.geom2):
                o = cn.geom2 if cn.geom1 == cg else cn.geom1
                if model.geom_bodyid[o] in arm_ids:
                    n += 1
        return n

    mujoco.mj_resetData(model, data)
    data.qpos[cq:cq + 3] = op
    data.qpos[cq + 3:cq + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    seed = np.zeros(len(ch.links))
    s1, _ = S.solve_ik(ch, names, [tx, ty, tz + 0.06], -90.0, seed)
    for j in S.ARM_JOINTS:
        data.ctrl[aadr[j]] = s1[names.index(j)]
    data.ctrl[aadr['gripper']] = 1.2
    for _ in range(500):
        mujoco.mj_step(model, data)
    s2, _ = S.solve_ik(ch, names, [tx, ty, tz], -90.0, s1)
    for j in S.ARM_JOINTS:
        data.ctrl[aadr[j]] = s2[names.index(j)]
    for _ in range(700):
        mujoco.mj_step(model, data)
    nc_before = arm_contacts()
    for k in range(50):
        data.ctrl[aadr['gripper']] = 1.2 * (1 - k / 49.0)
        for _ in range(8):
            mujoco.mj_step(model, data)
    nc_close = arm_contacts()
    grip_ang = data.qpos[qadr['gripper']]
    s3, _ = S.solve_ik(ch, names, [tx, ty, tz + 0.10], -90.0, s2)
    for j in S.ARM_JOINTS:
        data.ctrl[aadr[j]] = s3[names.index(j)]
    for _ in range(800):
        mujoco.mj_step(model, data)
    oz = data.xpos[cube][2]
    return (oz > op[2] + 0.005, max(nc_before, nc_close),
            np.linalg.norm(data.xpos[cube] - op) * 1000, grip_ang)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--obj-size', type=float, default=0.014)
    ap.add_argument('--axis', default='both', choices=['x', 'y', 'both'])
    ap.add_argument('--span', type=float, default=30.0, help='±mm')
    ap.add_argument('--step', type=float, default=6.0)
    a = ap.parse_args()
    S._OBJ_SIZE_OVERRIDE[0] = a.obj_size

    model = S.build_model()
    data = mujoco.MjData(model)
    ch, names = S.make_chain()
    op = S.obj_world_pos()
    print(f'物块 {a.obj_size*1000:.0f}mm @ ({op[0]:.4f}, {op[1]:.4f}, '
          f'{op[2]:.4f})')

    offs = [i * a.step for i in range(-int(a.span // a.step),
                                     int(a.span // a.step) + 1)]
    axes = ['x', 'y'] if a.axis == 'both' else [a.axis]
    best = []
    for ax in axes:
        print(f'\n=== 扫描 base {ax.upper()} 方向偏移 ===')
        print('  偏移(mm)  是否夹住   爪-物块接触  物块位移  夹爪终角')
        for o in offs:
            dx, dy = (o / 1000.0, 0.0) if ax == 'x' else (0.0, o / 1000.0)
            ok, nc, moved, ga = trial(model, data, ch, names, dx, dy, 0.0,
                                      a.obj_size)
            print(f'  {o:+7.1f}   {"✅ 夹住" if ok else "❌ 没夹住"}    '
                  f'{nc:2d}         {moved:6.1f}mm   {ga:+.3f}', flush=True)
            if ok:
                best.append((ax, o, moved, nc))
    print()
    if best:
        print('🎉 找到可用偏移:')
        for ax, o, moved, nc in best:
            print(f'   base {ax.upper()} {o:+.1f}mm  接触{nc}  位移{moved:.1f}mm')
    else:
        print('本次扫描无成功组合 —— 需要同时调高度/滚转，或换更小的物块')


if __name__ == '__main__':
    main()
