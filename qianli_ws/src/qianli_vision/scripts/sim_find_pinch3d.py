#!/usr/bin/env python3
"""三维扫描找出"物块被两片爪同时夹住"的位置（用真实接触，不用顶点近似）

方法：
  1. 把臂摆到 TCP = 物块名义位置，夹爪**闭合**
  2. 在物块周围三维扫描它的位置（只做 mj_forward，不做动力学 → 很快）
  3. 记录每个位置上"与固定侧/活动侧的接触数"
  4. 两侧都 > 0 的位置 = 物块正好被夹在中间 → 那就是正确抓取位姿

比之前"最近顶点对/缝方向"可靠：那是几何近似，这里是引擎真实接触。

用法：
  ~/mj/bin/python sim_find_pinch3d.py --obj-size 0.020
"""

import argparse
import sys

import numpy as np

import mujoco

import sim_grasp as S
import sim_mesh_gripper as MG
import sim_ik_dls as IK


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--obj-size', type=float, default=0.020)
    ap.add_argument('--span', type=float, default=40.0, help='±mm')
    ap.add_argument('--step', type=float, default=4.0)
    ap.add_argument('--angle', type=float, default=0.0,
                    help='扫描时用的夹爪角（0=闭合）')
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
    cq = S.attach_handles(model)[3]
    op = S.obj_world_pos()

    # 摆位
    IK.ik_dls(model, data, qadr, op)
    data.qpos[qadr['gripper']] = a.angle
    data.qpos[cq:cq + 3] = op
    data.qpos[cq + 3:cq + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)

    # ★ 只统计"指尖区域"的零件：否则腕部外壳碰到物块顶面会被误判成夹住
    def tip_geoms(body_id, band=0.035):
        ids = []
        lo = 1e9
        for i in range(model.ngeom):
            if model.geom_bodyid[i] != body_id:
                continue
            p = data.geom_xpos[i]
            R = data.geom_xmat[i].reshape(3, 3)
            half = np.abs(R) @ model.geom_size[i][:3]
            lo = min(lo, p[2] - half[2])
        for i in range(model.ngeom):
            if model.geom_bodyid[i] != body_id:
                continue
            p = data.geom_xpos[i]
            R = data.geom_xmat[i].reshape(3, 3)
            half = np.abs(R) @ model.geom_size[i][:3]
            if p[2] - half[2] < lo + band:
                ids.append(i)
        return set(ids), lo

    fix_tip, lo_fix = tip_geoms(gl)
    mov_tip, lo_mov = tip_geoms(mjb)
    print(f'名义位姿：TCP 已对准物块，夹爪角 {a.angle}')
    print(f'指尖区零件：固定侧 {len(fix_tip)} 个（最低 {lo_fix:+.4f}），'
          f'活动侧 {len(mov_tip)} 个（最低 {lo_mov:+.4f}）')
    print(f'高度差 = {(lo_mov-lo_fix)*1000:+.1f}mm')
    print(f'扫描范围 ±{a.span}mm，步长 {a.step}mm，'
          f'{int(2*a.span/a.step+1)**3} 个位置')

    hits = []
    vals = np.arange(-a.span, a.span + 1e-9, a.step)
    for dx in vals:
        for dy in vals:
            for dz in vals:
                data.qpos[cq:cq + 3] = op + np.array([dx, dy, dz]) / 1000.0
                mujoco.mj_forward(model, data)
                nf = nm = 0
                for c in range(data.ncon):
                    cn = data.contact[c]
                    if cn.geom1 in fix_tip or cn.geom2 in fix_tip:
                        nf += 1
                    if cn.geom1 in mov_tip or cn.geom2 in mov_tip:
                        nm += 1
                if nf > 0 and nm > 0:
                    hits.append((dx, dy, dz, nf, nm))
    print(f'\n两侧同时接触的位置: {len(hits)} 个')
    if hits:
        arr = np.array([(h[0], h[1], h[2]) for h in hits])
        c = arr.mean(axis=0)
        print(f'  有效区域中心偏移 = ({c[0]:+.1f}, {c[1]:+.1f}, {c[2]:+.1f}) mm')
        print(f'  范围: x[{arr[:,0].min():+.0f},{arr[:,0].max():+.0f}] '
              f'y[{arr[:,1].min():+.0f},{arr[:,1].max():+.0f}] '
              f'z[{arr[:,2].min():+.0f},{arr[:,2].max():+.0f}] mm')
        print('  样例:')
        for h in hits[:8]:
            print(f'    偏移({h[0]:+5.1f},{h[1]:+5.1f},{h[2]:+5.1f})mm  '
                  f'固定侧{h[3]} 活动侧{h[4]} 接触')
        np.save('/tmp/pinch_offset.npy', c)
        idx = np.argmin(np.linalg.norm(arr, axis=1))
        print(f'  离原位最近的可夹位置 = {np.round(arr[idx], 1)} mm')
        np.save('/tmp/pinch_offset.npy', arr[idx])
    else:
        print('  没有两侧同时接触的位置 —— 需要换夹爪角或调整臂的姿态')


if __name__ == '__main__':
    main()
