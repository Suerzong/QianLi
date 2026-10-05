#!/usr/bin/env python3
"""检查夹爪碰撞体：类型 / 尺寸 / 世界包围盒

目的：验证"MuJoCo 用凸包做网格碰撞，把爪口糊住"的假说。
  · 若爪子的碰撞 geom 是 mjGEOM_MESH → 用的是凸包
  · 打印每个 geom 的世界 AABB，看"对面那片爪"到底在哪、有多大
"""

import sys

import numpy as np

import mujoco

sys.argv = [sys.argv[0]]
import sim_grasp as S   # noqa: E402

GEOM_TYPE = {0: 'PLANE', 1: 'HFIELD', 2: 'SPHERE', 3: 'CAPSULE', 4: 'ELLIPSOID',
             5: 'CYLINDER', 6: 'BOX', 7: 'MESH'}


def main():
    model = S.build_model()
    data = mujoco.MjData(model)
    qadr = {}
    for i in range(model.njnt):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if n:
            qadr[n] = model.jnt_qposadr[i]

    # 摆到工具朝下的姿态，便于比较
    ch, names = S.make_chain()
    seed = np.zeros(len(ch.links))
    sol, _ = S.solve_ik(ch, names, [0.30, 0.0, 0.05], -90.0, seed)
    for j in S.ARM_JOINTS:
        data.qpos[qadr[j]] = sol[names.index(j)]
    data.qpos[qadr['gripper']] = 1.745          # 完全张开
    mujoco.mj_forward(model, data)

    gl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper_link')
    mj = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                           'moving_jaw_so101_v1_link')
    print('=== 所有碰撞 geom ===')
    print(' id  body                       type      size')
    for i in range(model.ngeom):
        b = model.geom_bodyid[i]
        bn = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or 'world'
        gn = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or '(无名)'
        t = GEOM_TYPE.get(model.geom_type[i], str(model.geom_type[i]))
        sz = np.round(model.geom_size[i], 4)
        print(f' {i:3d}  {bn:24s}  {t:8s}  {sz}  名字={gn}')

    print('\n=== gripper_link / moving_jaw 的 geom 世界 AABB（夹爪全开）===')
    for tag, bid in (('gripper_link', gl), ('moving_jaw', mj)):
        print(f'--- {tag} ---')
        for i in range(model.ngeom):
            if model.geom_bodyid[i] != bid:
                continue
            # MuJoCo 提供 geom_aabb（rbounding 中心/半径）不够精确，
            # 这里用 geom_xpos + xmat + size 估 AABB
            p = data.geom_xpos[i]
            R = data.geom_xmat[i].reshape(3, 3)
            sz = model.geom_size[i]
            t = model.geom_type[i]
            if t == mujoco.mjtGeom.mjGEOM_MESH:
                # mesh: size 是该 mesh 的半尺寸（MuJoCo 填的是 mesh bbox 半长）
                half = np.abs(R) @ sz[:3]
            elif t == mujoco.mjtGeom.mjGEOM_BOX:
                half = np.abs(R) @ sz[:3]
            elif t == mujoco.mjtGeom.mjGEOM_SPHERE:
                half = np.array([sz[0]] * 3)
            else:
                half = np.abs(R) @ sz[:3]
            print(f'  geom {i} type={GEOM_TYPE.get(t, t)}')
            print(f'    中心 (世界) = ({p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f})')
            print(f'    AABB 半长   = ({half[0]*1000:.1f}, {half[1]*1000:.1f},'
                  f' {half[2]*1000:.1f}) mm')
            print(f'    z 范围      = [{p[2]-half[2]:+.4f}, '
                  f'{p[2]+half[2]:+.4f}]')

    # 两爪碰撞体之间的最小距离（引擎精确计算）
    print('\n=== 两爪碰撞体最小距离（mj_geomDistance）===')
    gl_geoms = [i for i in range(model.ngeom) if model.geom_bodyid[i] == gl]
    mj_geoms = [i for i in range(model.ngeom) if model.geom_bodyid[i] == mj]
    print('  夹爪角   最近距离(mm)   涉及 geom 对')
    for g in (1.745, 1.2, 0.6, 0.0, -0.1745):
        data.qpos[qadr['gripper']] = g
        mujoco.mj_forward(model, data)
        best = (1e9, None)
        for a in gl_geoms:
            for b in mj_geoms:
                d = mujoco.mj_geomDistance(model, data, a, b, 0.5, None)
                if d < best[0]:
                    best = (d, (a, b))
        print(f'  {g:+.4f}   {best[0]*1000:8.2f}      {best[1]}')
        if best[0] < 0:
            print('    ⚠️ 负值 = 两爪碰撞体互相穿透 → 凸包已经重叠，'
                  '爪口被糊住！')


if __name__ == '__main__':
    main()
