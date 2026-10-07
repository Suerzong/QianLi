#!/usr/bin/env python3
"""自动把"基本体爪子"摆到物块两侧，然后重跑抓取

思路（全用引擎计算，不手搓变换）：
  1. 用 IK 把臂摆到"TCP = 物块中心 + dz"
  2. 用引擎读出此时 物块 在 gripper_link 坐标系里的位置
  3. 把两片爪分别放在物块两侧（沿 gripper_link 的 x 轴），间距 = 口宽
     · 固定爪挂在 gripper_link 上 → 直接写局部坐标
     · 活动爪挂在 moving_jaw 上 → 用引擎把世界目标点换算成 jaw 局部坐标
  4. 跑一次完整抓取（预抓取→下压→闭合→抬起），看能否抓起 20mm 物块

用法：
  ~/mj/bin/python sim_gripper_place.py --dz -0.030 --sep 14
  ~/mj/bin/python sim_gripper_place.py --sweep
"""

import argparse
import math
import sys

import numpy as np

import mujoco

import sim_grasp as S


def build_and_place(obj_size, dz, sep_mm, sign=+1):
    """建模型并把两片爪摆到物块两侧。sep_mm = 每片爪中心离物块中心的距离。"""
    S._OBJ_SIZE_OVERRIDE[0] = obj_size
    spec = S.load_mujoco_spec(S.URDF)
    wb = spec.worldbody
    gt = wb.add_geom(); gt.name = 'table'
    gt.type = mujoco.mjtGeom.mjGEOM_BOX
    gt.size = [0.4, 0.4, 0.15]; gt.pos = [0.3, 0.0, S.TABLE_Z - 0.15]
    gp = wb.add_geom(); gp.name = 'pedestal'
    gp.type = mujoco.mjtGeom.mjGEOM_BOX
    gp.size = [0.045, 0.05, (S.BASE_BOTTOM - S.TABLE_Z) / 2]
    gp.pos = [0.0, 0.0, (S.TABLE_Z + S.BASE_BOTTOM) / 2]
    if S.BOARD_ORIGIN is not None and S.BOARD_YAW is not None:
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

    # 先加不带位置的爪占位（后面用引擎算出的局部坐标再改）
    body_fix = spec.body('gripper_link')
    gf = body_fix.add_geom(); gf.name = 'finger_fixed'
    gf.type = mujoco.mjtGeom.mjGEOM_BOX
    gf.size = [0.004, 0.009, 0.020]
    body_mov = spec.body('moving_jaw_so101_v1_link')
    gm = body_mov.add_geom(); gm.name = 'finger_moving'
    gm.type = mujoco.mjtGeom.mjGEOM_BOX
    gm.size = [0.004, 0.009, 0.020]

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

    model = spec.compile()
    data = mujoco.MjData(model)

    # ---- 用引擎算爪子局部坐标 ----
    qadr = {}
    for i in range(model.njnt):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if n:
            qadr[n] = model.jnt_qposadr[i]
    ch, names = S.make_chain()
    seed = np.zeros(len(ch.links))
    tgt = [op[0], op[1], op[2] + dz]
    sol, _ = S.solve_ik(ch, names, tgt, -90.0, seed)
    for j in S.ARM_JOINTS:
        data.qpos[qadr[j]] = sol[names.index(j)]
    data.qpos[qadr['gripper']] = 1.2
    mujoco.mj_forward(model, data)

    gl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper_link')
    mjl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                            'moving_jaw_so101_v1_link')
    R_gl = data.xmat[gl].reshape(3, 3)
    p_gl = data.xpos[gl]
    sep_dir = R_gl @ np.array([1.0, 0.0, 0.0])      # gripper_link 的 x 轴
    cube_w = data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                        'object')]
    r = sep_mm / 1000.0
    p_fix_w = cube_w + sign * r * sep_dir
    p_mov_w = cube_w - sign * r * sep_dir
    # 世界 → 各 body 局部
    fix_local = R_gl.T @ (p_fix_w - p_gl)
    R_mj = data.xmat[mjl].reshape(3, 3)
    mov_local = R_mj.T @ (p_mov_w - data.xpos[mjl])
    # 写回 geom
    fid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'finger_fixed')
    mid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'finger_moving')
    model.geom_pos[fid] = fix_local
    model.geom_pos[mid] = mov_local

    # 碰撞分组
    arm_ids = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n) for n in
               ['shoulder_link', 'upper_arm_link', 'lower_arm_link',
                'wrist_link', 'gripper_link', 'moving_jaw_so101_v1_link']}
    env = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n)
           for n in ('table', 'pedestal', 'board')}
    cube_g = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    for i in range(model.ngeom):
        if i in (fid, mid):
            model.geom_contype[i], model.geom_conaffinity[i] = 1, 1
        elif model.geom_bodyid[i] in arm_ids:
            model.geom_contype[i], model.geom_conaffinity[i] = 0, 0
        elif i in env:
            model.geom_contype[i], model.geom_conaffinity[i] = 2, 2
        elif i == cube_g:
            model.geom_contype[i], model.geom_conaffinity[i] = 1, 3
    for i in range(model.nu):
        nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        j = nm.replace('servo_', '')
        kp = servo.get(j, 120.0)
        model.actuator_gainprm[i][0] = kp
        model.actuator_biasprm[i][1] = -kp
        model.actuator_biasprm[i][2] = -2.0 * math.sqrt(kp) * 0.2
    for i in range(model.njnt):
        if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE:
            nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            model.dof_armature[model.jnt_dofadr[i]] = (
                0.002 if nm == 'gripper' else 0.02)
    return model, (fix_local, mov_local)


def trial(model, obj_size, dz):
    """完整抓取一次（TCP 目标 = 物块 + dz），返回 (是否抓起, 接触数, 位移mm)。"""
    import sim_sweep_offset as SW
    ch, names = S.make_chain()
    d = mujoco.MjData(model)
    return SW.trial(model, d, ch, names, 0.0, 0.0, dz, obj_size)[:3]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--obj-size', type=float, default=0.020)
    ap.add_argument('--dz', type=float, default=-0.030)
    ap.add_argument('--sep', type=float, default=14.0, help='每片爪离物块中心 mm')
    ap.add_argument('--sweep', action='store_true')
    a = ap.parse_args()

    print(f'物块 {a.obj_size*1000:.0f}mm，TCP 下移 {a.dz*1000:+.0f}mm')
    print('  sign  sep(mm)  结果      接触  位移')
    combos = [(+1, 10), (+1, 12), (+1, 14), (-1, 10), (-1, 12), (-1, 14)] \
        if a.sweep else [(+1, a.sep)]
    best = None
    for sign, sep in combos:
        model, loc = build_and_place(a.obj_size, a.dz, sep, sign)
        ok, nc, moved = trial(model, a.obj_size, a.dz)
        print(f'  {sign:+d}    {sep:5.1f}   {"✅ 抓起" if ok else "❌ 没抓起"}  '
              f'{nc:2d}   {moved:6.1f}mm', flush=True)
        if ok and best is None:
            best = (sign, sep)
    if best:
        print(f'\n🎉 可用配置: sign={best[0]:+d}, sep={best[1]}mm '
              f'（20mm 物块在基本体爪子下抓起来了）')
        print('→ 说明之前"抓不了 20mm"确实是**凸包糊住口**的伪像')


if __name__ == '__main__':
    main()
