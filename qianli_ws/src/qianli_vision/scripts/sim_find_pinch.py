#!/usr/bin/env python3
"""三维搜索"物块正好卡在两爪之间"的抓取位姿偏移

判据（比"距离和最小"可靠）：
  · 固定侧爪、活动侧爪各自到物块的最小距离 d_fix / d_mov
  · 两侧的最近点→物块中心 的方向要**相反**（点积 < 0）= 物块被夹在中间
  · 在此前提下，d_fix + d_mov 最小 = 最佳夹取位姿
  （只用"距离和最小"会被"穿透"骗到：穿得越深距离和越小）

技巧：爪子是刚体，所以"TCP 目标偏移 local offset"等价于
      "物块反向偏移 R_gl @ offset" → 摆一次臂就能扫完整个网格，免去上千次 IK

用法：
  ~/mj/bin/python sim_find_pinch.py --obj-size 0.020
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
    ap.add_argument('--x', nargs=3, type=float, default=[-0.045, 0.045, 0.005])
    ap.add_argument('--y', nargs=3, type=float, default=[-0.045, 0.045, 0.005])
    ap.add_argument('--z', nargs=3, type=float, default=[-0.080, 0.010, 0.005])
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
    cb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'object')
    qadr_handles = S.attach_handles(model)
    cq = qadr_handles[3]
    fix_parts = [i for i in range(model.ngeom)
                 if model.geom_bodyid[i] == gl
                 and (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
                      or '').startswith('pg')]
    mov_parts = [i for i in range(model.ngeom)
                 if model.geom_bodyid[i] == mjb
                 and (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
                      or '').startswith('pg')]
    op = S.obj_world_pos()
    ch, names = S.make_chain()
    # 摆到名义抓取姿态（TCP 对准物块）
    sol, _ = S.solve_ik(ch, names, list(op), -90.0, np.zeros(len(ch.links)))
    for j in S.ARM_JOINTS:
        data.qpos[qadr[j]] = sol[names.index(j)]
    data.qpos[qadr['gripper']] = 1.2
    data.qpos[cq:cq + 3] = op
    data.qpos[cq + 3:cq + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    R_gl = data.xmat[gl].reshape(3, 3).copy()

    def closest(parts):
        best = (1e9, None)
        for p in parts:
            d = mujoco.mj_geomDistance(model, data, cg, p, 0.5, None)
            if d < best[0]:
                best = (d, p)
        return best

    xs = np.arange(a.x[0], a.x[1] + 1e-9, a.x[2])
    ys = np.arange(a.y[0], a.y[1] + 1e-9, a.y[2])
    zs = np.arange(a.z[0], a.z[1] + 1e-9, a.z[2])
    print(f'网格 {len(xs)}×{len(ys)}×{len(zs)} = {len(xs)*len(ys)*len(zs)} 个位姿')
    results = []
    for dx in xs:
        for dy in ys:
            for dz in zs:
                off_local = np.array([dx, dy, dz])
                # 物块反向偏移（等价于爪子正向偏移）
                data.qpos[cq:cq + 3] = op - R_gl @ off_local
                mujoco.mj_forward(model, data)
                df, pf = closest(fix_parts)
                dm, pm = closest(mov_parts)
                if pf is None or pm is None:
                    continue
                cube_c = data.xpos[cb]
                # 两侧最近点 → 物块中心 的方向
                vf = cube_c - data.geom_xpos[pf]
                vm = cube_c - data.geom_xpos[pm]
                nf = vf / (np.linalg.norm(vf) + 1e-9)
                nm = vm / (np.linalg.norm(vm) + 1e-9)
                dot = float(np.dot(nf, nm))
                if dot < -0.2 and max(df, dm) < 0.006:
                    results.append((df + dm, dx, dy, dz, df, dm, dot))
    results.sort(key=lambda r: r[0])
    print(f'\n满足"两侧相反方向且都 <6mm"的位姿: {len(results)} 个')
    print('  排名  dx(mm)  dy(mm)  dz(mm)  固定侧(mm) 活动侧(mm) 方向点积')
    for k, r in enumerate(results[:12], 1):
        print(f'  {k:3d}  {r[1]*1000:+6.1f} {r[2]*1000:+6.1f} {r[3]*1000:+6.1f}  '
              f'{r[4]*1000:9.1f} {r[5]*1000:9.1f}  {r[6]:+.2f}')
    if not results:
        print('没有满足条件的位姿 —— 说明两片爪根本无法同时从相反方向夹住物块')
        print('（即：URDF 里的爪子几何对不上真实的夹爪）')


if __name__ == '__main__':
    main()
