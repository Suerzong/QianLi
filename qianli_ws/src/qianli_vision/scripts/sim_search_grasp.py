#!/usr/bin/env python3
"""黑盒搜索可用抓取参数（模型只构建一次，反复试抓）

思路：
  不再靠几何推算（顶点/距离/接触反复出错），直接把"仿真里抓起来了没有"
  当目标函数，系统扫描 TCP 目标偏移 + 接近角。

优化要点：模型只 build 一次（68 个网格零件，每次重建要 15s 是瓶颈），
         每次试抓只 resetData。

用法：
  ~/mj/bin/python sim_search_grasp.py --obj-size 0.020 --phase coarse
  ~/mj/bin/python sim_search_grasp.py --obj-size 0.020 --phase fine --center 8 -4
结果同时写入 /tmp/grasp_search.log
"""

import argparse
import itertools
import sys
import time

import numpy as np

import mujoco

import sim_grasp as S
import sim_mesh_gripper as MG
import sim_ik_dls as IK

LOG = '/tmp/grasp_search.log'


def log(msg):
    print(msg, flush=True)
    with open(LOG, 'a') as fh:
        fh.write(msg + '\n')


def trial(model, qadr, aadr, cq, op, cg, cb, dx, dy, dz, ang,
          close_steps=60, sub=8, descend=500, lift=50, settle=400):
    """一次试抓：摆位 → 闭合 → **等爪子合拢** → 抬起。

    settle 阶段是关键：不加它的话，闭合斜坡跑完时伺服还滞后（爪停在
    0.05 rad，口宽还有 20mm+）就开始抬 → 必然夹不住。
    """
    data = mujoco.MjData(model)
    tgt = op + np.array([dx, dy, dz]) / 1000.0
    IK.ik_dls(model, data, qadr, tgt)
    data.qpos[qadr['gripper']] = ang
    data.qpos[cq:cq + 3] = op
    data.qpos[cq + 3:cq + 7] = [1, 0, 0, 0]
    for j in S.ARM_JOINTS:
        data.ctrl[aadr[j]] = data.qpos[qadr[j]]
    data.ctrl[aadr['gripper']] = ang
    mujoco.mj_forward(model, data)
    for _ in range(descend):
        mujoco.mj_step(model, data)
    maxc = 0
    for k in range(close_steps + 1):
        data.ctrl[aadr['gripper']] = ang * (1 - k / float(close_steps))
        for _ in range(sub):
            mujoco.mj_step(model, data)
        n = sum(1 for c in range(data.ncon)
                if cg in (data.contact[c].geom1, data.contact[c].geom2))
        maxc = max(maxc, n)
    # ★ 等待爪子真正合拢（被物块挡住就会停在中途）
    data.ctrl[aadr['gripper']] = 0.0
    for _ in range(settle):
        mujoco.mj_step(model, data)
        n = sum(1 for c in range(data.ncon)
                if cg in (data.contact[c].geom1, data.contact[c].geom2))
        maxc = max(maxc, n)
    ga = data.qpos[qadr['gripper']]
    for k in range(lift + 1):                     # 抬起
        t = tgt + np.array([0, 0, 0.10 * k / float(lift)])
        IK.ik_dls(model, data, qadr, t)
        for j in S.ARM_JOINTS:
            data.ctrl[aadr[j]] = data.qpos[qadr[j]]
        data.ctrl[aadr['gripper']] = 0.0
        for _ in range(6):
            mujoco.mj_step(model, data)
    dz_up = (data.xpos[cb][2] - op[2]) * 1000
    return dz_up > 5.0, ga, dz_up, maxc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--obj-size', type=float, default=0.020)
    ap.add_argument('--phase', default='coarse')
    ap.add_argument('--center', nargs=2, type=float, default=[0.0, 0.0])
    a = ap.parse_args()

    t0 = time.time()
    model = MG.build(a.obj_size)
    qadr, aadr = {}, {}
    for i in range(model.njnt):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if n:
            qadr[n] = model.jnt_qposadr[i]
    for i in range(model.nu):
        aadr[mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                               i).replace('servo_', '')] = i
    cq = S.attach_handles(model)[3]
    cg = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    cb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'object')
    op = S.obj_world_pos()
    log(f'[build {time.time()-t0:.1f}s] 物块 {a.obj_size*1000:.0f}mm，'
        f'phase={a.phase}')

    if a.phase == 'coarse':
        xs = [-16, -8, 0, 8, 16]
        ys = [-16, -8, 0, 8, 16]
        zs = [0.0, -10.0, 10.0]
        angs = [0.6, 1.2]
    else:
        cx, cy = a.center
        xs = [cx + d for d in (-6, -3, 0, 3, 6)]
        ys = [cy + d for d in (-6, -3, 0, 3, 6)]
        zs = [0.0, -6.0, 6.0]
        angs = [0.4, 0.6, 0.9]

    combos = list(itertools.product(xs, ys, zs, angs))
    log(f'共 {len(combos)} 组参数')
    log('  dx(mm) dy(mm) dz(mm)  角   结果      终角    升高(mm) 最多接触')
    best = []
    for i, (dx, dy, dz, ang) in enumerate(combos, 1):
        ok, ga, dzu, mc = trial(model, qadr, aadr, cq, op, cg, cb,
                                dx, dy, dz, ang)
        log(f'  {dx:+6.1f} {dy:+6.1f} {dz:+6.1f}  {ang:.2f}  '
            f'{"✅ 抓起" if ok else "❌ 没抓起"}  {ga:+.3f}  {dzu:+8.1f}  {mc:3d}'
            f'   [{i}/{len(combos)}]')
        if ok:
            best.append((dx, dy, dz, ang, dzu))
    log('')
    if best:
        log(f'🎉 找到 {len(best)} 组可用参数：')
        for b in sorted(best, key=lambda x: -x[4])[:10]:
            log(f'   dx={b[0]:+.1f} dy={b[1]:+.1f} dz={b[2]:+.1f} '
                f'角={b[3]:.2f} 升高={b[4]:.1f}mm')
    else:
        log('本批无可用参数')


if __name__ == '__main__':
    main()
