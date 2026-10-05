#!/usr/bin/env python3
"""两遍构建：把"基本体爪子"按引擎算出的局部坐标摆好，测 20mm 物块能否抓起

背景（本会话踩的坑）：
  · 爪子原碰撞体是 MESH → MuJoCo 用凸包 → 两爪凸包互相穿透 20mm，爪口被糊住
  · 往 compile 后的 model.geom_pos 写值**不生效**（和 gainprm 同类坑）
  → 所以改成两遍构建：第一遍算出应有的局部坐标，第二遍在 spec 里设好再编译

用法：
  ~/mj/bin/python sim_gripper_place2.py --obj-size 0.020
  ~/mj/bin/python sim_gripper_place2.py --obj-size 0.020 --sweep
"""

import argparse
import math
import sys

import numpy as np

import mujoco

import sim_grasp as S


def make_spec(obj_size, fix_local, mov_local):
    S._OBJ_SIZE_OVERRIDE[0] = obj_size
    spec = mujoco.MjSpec.from_file(S.URDF)
    wb = spec.worldbody

    gt = wb.add_geom(); gt.name = 'table'
    gt.type = mujoco.mjtGeom.mjGEOM_BOX
    gt.size = [0.4, 0.4, 0.15]; gt.pos = [0.3, 0.0, S.TABLE_Z - 0.15]
    gp = wb.add_geom(); gp.name = 'pedestal'
    gp.type = mujoco.mjtGeom.mjGEOM_BOX
    gp.size = [0.045, 0.05, (S.BASE_BOTTOM - S.TABLE_Z) / 2]
    gp.pos = [0.0, 0.0, (S.TABLE_Z + S.BASE_BOTTOM) / 2]
    cy, sy = math.cos(S.BOARD_YAW), math.sin(S.BOARD_YAW)
    cx = S.BOARD_ORIGIN[0] + cy * (S.BOARD_W / 2) - sy * (S.BOARD_H / 2)
    cyy = S.BOARD_ORIGIN[1] + sy * (S.BOARD_W / 2) + cy * (S.BOARD_H / 2)
    gb = wb.add_geom(); gb.name = 'board'
    gb.type = mujoco.mjtGeom.mjGEOM_BOX
    gb.size = [S.BOARD_W / 2, S.BOARD_H / 2, 0.0015]
    gb.pos = [cx, cyy, S.TABLE_Z + 0.0015]
    gb.quat = [math.cos(S.BOARD_YAW / 2), 0, 0, math.sin(S.BOARD_YAW / 2)]
    op = S.obj_world_pos()
    ob = wb.add_body(name='object'); ob.pos = list(op); ob.add_freejoint()
    go = ob.add_geom(); go.name = 'cube'
    go.type = mujoco.mjtGeom.mjGEOM_BOX
    go.size = [obj_size / 2] * 3
    go.mass = 0.008

    # 两片爪：位置在 spec 里设定（compile 后再改不生效）
    gf = spec.body('gripper_link').add_geom()
    gf.name = 'finger_fixed'
    gf.type = mujoco.mjtGeom.mjGEOM_BOX
    gf.size = [0.004, 0.009, 0.020]
    gf.pos = list(fix_local)
    gm = spec.body('moving_jaw_so101_v1_link').add_geom()
    gm.name = 'finger_moving'
    gm.type = mujoco.mjtGeom.mjGEOM_BOX
    gm.size = [0.004, 0.009, 0.020]
    gm.pos = list(mov_local)

    servo = {'shoulder_pan': 120, 'shoulder_lift': 120, 'elbow_flex': 120,
             'wrist_flex': 60, 'wrist_roll': 30, 'gripper': 20}
    for j in S.ALL_JOINTS:
        act = spec.add_actuator()
        act.name = f'servo_{j}'
        act.target = j
        act.trntype = mujoco.mjtTrn.mjTRN_JOINT
        act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
        act.biastype = mujoco.mjtBias.mjBIAS_AFFINE
        act.gainprm[0] = 1.0
        lo, hi = (-0.1745, 1.7453) if j == 'gripper' else (-3.2, 3.2)
        act.ctrlrange = [lo, hi]
        act.forcerange = [-3.0, 3.0] if j == 'gripper' else [-12.0, 12.0]
    return spec


def finalize(model, servo):
    """compile 后仍能生效的设定：碰撞分组 / 增益 / 转子惯量。"""
    arm_ids = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n) for n in
               ['shoulder_link', 'upper_arm_link', 'lower_arm_link',
                'wrist_link', 'gripper_link', 'moving_jaw_so101_v1_link']}
    env = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n)
           for n in ('table', 'pedestal', 'board')}
    cube = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    ff = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'finger_fixed')
    fm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'finger_moving')
    for i in range(model.ngeom):
        if i in (ff, fm):
            model.geom_contype[i], model.geom_conaffinity[i] = 1, 1
        elif model.geom_bodyid[i] in arm_ids:
            model.geom_contype[i], model.geom_conaffinity[i] = 0, 0
        elif i in env:
            model.geom_contype[i], model.geom_conaffinity[i] = 2, 2
        elif i == cube:
            model.geom_contype[i], model.geom_conaffinity[i] = 1, 3
    for i in range(model.nu):
        j = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                              i).replace('servo_', '')
        kp = servo.get(j, 120.0)
        model.actuator_gainprm[i][0] = kp
        model.actuator_biasprm[i][1] = -kp
        model.actuator_biasprm[i][2] = -2.0 * math.sqrt(kp) * 0.2
    for i in range(model.njnt):
        if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE:
            nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            model.dof_armature[model.jnt_dofadr[i]] = (
                0.002 if nm == 'gripper' else 0.02)
    return model


SERVO = {'shoulder_pan': 120, 'shoulder_lift': 120, 'elbow_flex': 120,
         'wrist_flex': 60, 'wrist_roll': 30, 'gripper': 20}


def pose_grasp(model, data, dz):
    """把臂摆到 TCP = 物块中心 + dz。返回该状态。"""
    qadr = {}
    for i in range(model.njnt):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if n:
            qadr[n] = model.jnt_qposadr[i]
    ch, names = S.make_chain()
    op = S.obj_world_pos()
    sol, _ = S.solve_ik(ch, names, [op[0], op[1], op[2] + dz], -90.0,
                        np.zeros(len(ch.links)))
    for j in S.ARM_JOINTS:
        data.qpos[qadr[j]] = sol[names.index(j)]
    data.qpos[qadr['gripper']] = 1.2
    mujoco.mj_forward(model, data)
    return qadr


def solve_local(obj_size, dz, sep_mm, sign):
    """第一遍：算出两片爪应有的局部坐标。"""
    model = finalize(make_spec(obj_size, (0, 0, 0), (0, 0, 0)).compile(), SERVO)
    data = mujoco.MjData(model)
    pose_grasp(model, data, dz)
    gl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper_link')
    mjl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                            'moving_jaw_so101_v1_link')
    cb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'object')
    R_gl = data.xmat[gl].reshape(3, 3)
    R_mj = data.xmat[mjl].reshape(3, 3)
    cube_w = data.xpos[cb]
    sep_dir = R_gl @ np.array([1.0, 0.0, 0.0])
    r = sep_mm / 1000.0
    fix_local = R_gl.T @ (cube_w + sign * r * sep_dir - data.xpos[gl])
    mov_local = R_mj.T @ (cube_w - sign * r * sep_dir - data.xpos[mjl])
    return fix_local, mov_local


def trial(model, obj_size, dz):
    import sim_sweep_offset as SW
    ch, names = S.make_chain()
    d = mujoco.MjData(model)
    return SW.trial(model, d, ch, names, 0.0, 0.0, dz, obj_size)[:3]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--obj-size', type=float, default=0.020)
    ap.add_argument('--dz', type=float, default=-0.030)
    ap.add_argument('--sep', type=float, default=14.0)
    ap.add_argument('--sweep', action='store_true')
    a = ap.parse_args()

    combos = [(+1, 12.0), (+1, 14.0), (+1, 16.0),
              (-1, 12.0), (-1, 14.0), (-1, 16.0)] if a.sweep \
        else [(+1, a.sep)]
    print(f'物块 {a.obj_size*1000:.0f}mm，'
          f'TCP 下移 {a.dz*1000:+.0f}mm')
    print('  sign  sep(mm)  爪A局部z(mm)  爪B局部z(mm)  结果      接触  位移')
    best = None
    for sign, sep in combos:
        fix_local, mov_local = solve_local(a.obj_size, a.dz, sep, sign)
        model = finalize(make_spec(a.obj_size, fix_local, mov_local).compile(),
                         SERVO)
        ok, nc, moved = trial(model, a.obj_size, a.dz)
        print(f'  {sign:+d}    {sep:5.1f}   {fix_local[2]*1000:+8.1f}   '
              f'{mov_local[2]*1000:+8.1f}   '
              f'{"✅ 抓起" if ok else "❌ 没抓起"}  {nc:2d}   {moved:6.1f}mm',
              flush=True)
        if ok and best is None:
            best = (sign, sep)
    if best:
        print(f'\n🎉 20mm 物块在"基本体爪子"下抓起来了 (sign={best[0]:+d}, '
              f'sep={best[1]}mm)')
        print('→ 证实之前"抓不了 20mm"是**凸包糊住爪口**的伪像')
    else:
        print('\n仍未抓起 —— 继续排查（爪形/位置/夹持力）')


if __name__ == '__main__':
    main()
