#!/usr/bin/env python3
"""闭合过程接触追踪（修掉 body/geom id 混用的 bug）

之前误报"0 接触"是因为拿 body id 去比 contact.geom1/geom2（那是 geom id）。
本脚本正确地用 geom id 过滤，并打印：
  · 接触点相对物块中心的偏移
  · 接触法向（看是从哪一面推物块）
  · 参与接触的几何属于哪个 body

用法：
  ~/mj/bin/python sim_close_trace.py --obj-size 0.020
  ~/mj/bin/python sim_close_trace.py --obj-size 0.020 --close-steps 300
"""

import argparse
import sys

import numpy as np

import mujoco

import sim_grasp as S
import sim_fixed_jaw as FJ
import sim_mesh_gripper as MG
import sim_ik_dls as IK


def body_of(model, gid):
    b = model.geom_bodyid[gid]
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or 'world'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--obj-size', type=float, default=0.020)
    ap.add_argument('--clearance', type=float, default=1.0)
    ap.add_argument('--approach-angle', type=float, default=0.6)
    ap.add_argument('--close-steps', type=int, default=120)
    ap.add_argument('--substeps', type=int, default=10)
    a = ap.parse_args()

    model = MG.build(a.obj_size)
    data = mujoco.MjData(model)
    qadr, aadr = {}, {}
    for i in range(model.njnt):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if n:
            qadr[n] = model.jnt_qposadr[i]
    for i in range(model.nu):
        aadr[mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                               i).replace('servo_', '')] = i

    parts = FJ.load_parts()
    tgt, info = FJ.compute_target_iter(model, data, qadr, parts, a.obj_size,
                                       a.clearance)
    cg = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')   # geom!
    cb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'object')  # body!
    cq = S.attach_handles(model)[3]
    op = S.obj_world_pos()
    print(f'目标 TCP = {np.round(tgt, 4)}')
    print(f'物块到固定爪面 = '
          f'{np.linalg.norm(op - info["v_fix"])*1000:.1f}mm (目标 '
          f'{a.obj_size/2*1000 + a.clearance:.1f}mm)')

    IK.ik_dls(model, data, qadr, tgt)
    data.qpos[qadr['gripper']] = a.approach_angle
    data.qpos[cq:cq + 3] = op
    data.qpos[cq + 3:cq + 7] = [1, 0, 0, 0]
    for j in S.ARM_JOINTS:
        data.ctrl[aadr[j]] = data.qpos[qadr[j]]
    data.ctrl[aadr['gripper']] = a.approach_angle
    mujoco.mj_forward(model, data)

    print(f'\n闭合追踪（b = 与物块的接触数，法向 = 接触力方向）:')
    print('  步   夹爪角   物块z      b   接触点相对物块(mm)        法向       对方')
    for k in range(a.close_steps + 1):
        data.ctrl[aadr['gripper']] = a.approach_angle * (
            1 - k / float(a.close_steps))
        for _ in range(a.substeps):
            mujoco.mj_step(model, data)
        if k % max(1, a.close_steps // 8) and k != a.close_steps:
            continue
        rows = [c for c in range(data.ncon)
                if cg in (data.contact[c].geom1, data.contact[c].geom2)]
        desc = []
        for c in rows[:2]:
            cn = data.contact[c]
            off = (cn.pos - data.xpos[cb]) * 1000
            nrm = np.round(np.array(cn.frame).reshape(3, 3)[:3, 0], 2)
            other = cn.geom2 if cn.geom1 == cg else cn.geom1
            desc.append(f'({off[0]:+5.1f},{off[1]:+5.1f},{off[2]:+5.1f}) '
                        f'{nrm} {body_of(model, other)[:12]}')
        print(f'  {k:3d}  {data.qpos[qadr["gripper"]]:+.3f}  '
              f'{data.xpos[cb][2]:+.4f}  '
              f'{len(rows):2d}   {" | ".join(desc) if desc else "-"}',
              flush=True)
    print(f'总位移 = '
          f'{np.linalg.norm(data.xpos[cb] - op)*1000:.1f}mm')


if __name__ == '__main__':
    main()
