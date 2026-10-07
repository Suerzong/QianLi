#!/usr/bin/env python3
"""探针：给 RL 用的"快速夹爪碰撞体"做体检（不碰真机）

回答 3 个问题：
  1. 两片盒子爪的**口宽 vs 夹爪角**关系（能否夹住 20mm 物块）
  2. 快速模型（盒子爪）相对 68 块 CoACD 模型的**建模块数 / 步进速度**提升
  3. 物理保真度：已验证的抓取参数 (off=(8,-4,0)mm, approach=0.6) 在
     快速模型里还能不能真的把物块抬起来（>5mm）

用法：
  ~/mj/bin/python probe_gripper_fast.py --gap        # 口宽扫描
  ~/mj/bin/python probe_gripper_fast.py --speed      # 速度对比
  ~/mj/bin/python probe_gripper_fast.py --hold       # 物理夹持保真度
  ~/mj/bin/python probe_gripper_fast.py --all
"""

import argparse
import math
import os
import time

import numpy as np

import mujoco

import sim_grasp as S


# ---------------------------------------------------------------- 快速模型
def make_fast_spec(obj_size, finger_half=(0.004, 0.009, 0.020),
                   fixed_pos=(0.010, 0.0, -0.070),
                   moving_pos=(0.0, -0.026, 0.0)):
    S._OBJ_SIZE_OVERRIDE[0] = obj_size
    spec = S.load_mujoco_spec(S.URDF)
    wb = spec.worldbody
    gt = wb.add_geom(); gt.name = 'table'
    gt.type = mujoco.mjtGeom.mjGEOM_BOX
    gt.size = [0.4, 0.4, 0.15]; gt.pos = [0.3, 0.0, S.TABLE_Z - 0.15]
    gt.rgba = [0.55, 0.55, 0.58, 1]
    gp = wb.add_geom(); gp.name = 'pedestal'
    gp.type = mujoco.mjtGeom.mjGEOM_BOX
    gp.size = [0.045, 0.05, (S.BASE_BOTTOM - S.TABLE_Z) / 2]
    gp.pos = [0.0, 0.0, (S.TABLE_Z + S.BASE_BOTTOM) / 2]
    gp.rgba = [0.3, 0.3, 0.32, 1]
    if S.BOARD_ORIGIN is not None and S.BOARD_YAW is not None:
        cy, sy = math.cos(S.BOARD_YAW), math.sin(S.BOARD_YAW)
        cx = S.BOARD_ORIGIN[0] + cy * (S.BOARD_W / 2) - sy * (S.BOARD_H / 2)
        cyy = S.BOARD_ORIGIN[1] + sy * (S.BOARD_W / 2) + cy * (S.BOARD_H / 2)
        gb = wb.add_geom(); gb.name = 'board'
        gb.type = mujoco.mjtGeom.mjGEOM_BOX
        gb.size = [S.BOARD_W / 2, S.BOARD_H / 2, 0.0015]
        gb.pos = [cx, cyy, S.TABLE_Z + 0.0015]
        gb.quat = [math.cos(S.BOARD_YAW / 2), 0, 0, math.sin(S.BOARD_YAW / 2)]
        gb.rgba = [0.9, 0.9, 0.9, 1]
    op = S.obj_world_pos()
    ob = wb.add_body(name='object'); ob.pos = list(op); ob.add_freejoint()
    go = ob.add_geom(); go.name = 'cube'
    go.type = mujoco.mjtGeom.mjGEOM_BOX
    go.size = [obj_size / 2] * 3
    go.rgba = [0.75, 0.75, 0.78, 1]; go.mass = 0.008

    for bname, pos, gname, rgba in (
            ('gripper_link', fixed_pos, 'finger_fixed', [0.9, 0.8, 0.2, 1]),
            ('moving_jaw_so101_v1_link', moving_pos, 'finger_moving',
             [0.95, 0.85, 0.25, 1])):
        g = spec.body(bname).add_geom()
        g.name = gname
        g.type = mujoco.mjtGeom.mjGEOM_BOX
        g.size = list(finger_half)
        g.pos = list(pos)
        g.rgba = rgba

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
    return spec, servo


def compile_fast(spec, servo):
    model = spec.compile()
    arm_ids = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n) for n in
               ['shoulder_link', 'upper_arm_link', 'lower_arm_link',
                'wrist_link', 'gripper_link', 'moving_jaw_so101_v1_link']}
    env = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n)
           for n in ('table', 'pedestal', 'board')}
    cube = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    ffix = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'finger_fixed')
    fmov = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'finger_moving')
    for i in range(model.ngeom):
        if i in (ffix, fmov):
            model.geom_contype[i], model.geom_conaffinity[i] = 1, 1
        elif model.geom_bodyid[i] in arm_ids:
            # 原 URDF 网格爪：凸包会把口糊死 → 碰撞全关（保留视觉）
            model.geom_contype[i], model.geom_conaffinity[i] = 0, 0
        elif i in env:
            model.geom_contype[i], model.geom_conaffinity[i] = 2, 2
        elif i == cube:
            model.geom_contype[i], model.geom_conaffinity[i] = 3, 5
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
    return model, ffix, fmov


def build_fast(obj_size, finger_half=(0.004, 0.009, 0.020),
               fixed_pos=(0.010, 0.0, -0.070), moving_pos=(0.0, -0.026, 0.0)):
    spec, servo = make_fast_spec(obj_size, finger_half=finger_half,
                                 fixed_pos=fixed_pos, moving_pos=moving_pos)
    return compile_fast(spec, servo)


def qaddr(model):
    q = {}
    for i in range(model.njnt):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if n:
            q[n] = model.jnt_qposadr[i]
    return q


# ---------------------------------------------------------------- 1. 口宽
def gap_scan():
    model, ffix, fmov = build_fast(0.020)
    data = mujoco.MjData(model)
    q = qaddr(model)
    gl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper_link')
    mj = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                           'moving_jaw_so101_v1_link')
    print('=== 口宽 vs 夹爪角（min 距离，负=穿透）===')
    print('   夹爪角    口宽(mm)   固定爪中心z   活动爪中心z   两爪z差(mm)')
    for g in [1.745, 1.4, 1.2, 0.8, 0.6, 0.4, 0.2, 0.0, -0.1745]:
        data.qpos[q['gripper']] = g
        mujoco.mj_forward(model, data)
        d = mujoco.mj_geomDistance(model, data, ffix, fmov, 1.0, None)
        zg = []
        for i, bid in enumerate((gl, mj)):
            gids = [k for k in range(model.ngeom)
                    if model.geom_bodyid[k] == bid and
                    (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, k)
                     or '') in ('finger_fixed', 'finger_moving')]
            zg.append(data.geom_xpos[gids[0]][2] if gids else float('nan'))
        print(f'   {g:+.4f}   {d*1000:8.1f}   {zg[0]:+.4f}     {zg[1]:+.4f}'
              f'     {(zg[1]-zg[0])*1000:+6.1f}')
    # 两爪口宽中心相对 gripper_link 的偏移（找对齐物块中心所需的 dz）
    data.qpos[q['gripper']] = 0.6
    mujoco.mj_forward(model, data)
    c = 0.5 * (data.geom_xpos[ffix] + data.geom_xpos[fmov])
    print(f'\n两爪中点(世界) {np.round(c,4)}，gripper_link 原点 '
          f'{np.round(data.xpos[gl],4)}')
    print(f'两爪中点 - gripper_link = {np.round((c-data.xpos[gl])*1000,2)} mm')
    tcp = data.xpos[gl] + data.xmat[gl].reshape(3, 3) @ S.FRAME_IN_GRIPPER
    print(f'TCP(FRAME_IN_GRIPPER)   {np.round(tcp,4)}')
    print(f'两爪中点 - TCP          = {np.round((c-tcp)*1000,2)} mm')
    offt = data.xmat[gl].reshape(3, 3).T @ (c - data.xpos[gl])
    print(f'两爪中点(GLB系)          = {np.round(offt,4)} m')


# ---------------------------------------------------------------- 2. 速度
def speed_bench():
    print('=== 建模块数 ===')
    for tag, fn in (('CoACD(真网格)', 'mesh'), ('盒子爪(快)', 'box')):
        t0 = time.perf_counter()
        if fn == 'mesh':
            import sim_mesh_gripper as MG
            m = MG.build(0.020)
        else:
            m, _, _ = build_fast(0.020)
        dt = time.perf_counter() - t0
        print(f'  {tag:14s} ngeom={m.ngeom:4d} nv={m.nv:3d} '
              f'build={dt:6.2f}s')
    print('\n=== mj_step 吞吐（随机 ctrl，8 线程无关的单线程）===')
    for tag, fn in (('CoACD(真网格)', 'mesh'), ('盒子爪(快)', 'box')):
        if fn == 'mesh':
            import sim_mesh_gripper as MG
            m = MG.build(0.020)
        else:
            m, _, _ = build_fast(0.020)
        d = mujoco.MjData(m)
        rng = np.random.default_rng(0)
        q = qaddr(m)
        for j in S.ARM_JOINTS:
            d.qpos[q[j]] = 0.0
        mujoco.mj_forward(m, d)
        # 预热
        for _ in range(200):
            mujoco.mj_step(m, d)
        for reps in (1, 5):
            n = 5000
            t0 = time.perf_counter()
            for i in range(n):
                d.ctrl[:] = rng.uniform(-0.5, 0.5, m.nu)
                mujoco.mj_step(m, d)
            dt = time.perf_counter() - t0
            print(f'  {tag:14s} steps={n} reps={reps} '
                  f'{n/dt:9.1f} steps/s  ({dt/n*1e6:6.1f} us/step)')


# ---------------------------------------------------------------- 3. 夹持保真
def goto_ctrl(model, data, qadr, aadr, target, steps, grip, tol=0.002,
              rounds=6):
    """在临时 MjData 里解 IK，只把结果写进 ctrl（不瞬移 qpos）。"""
    import sim_ik_dls as IK
    gl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper_link')
    e = 1e9
    for _ in range(rounds):
        tmp = mujoco.MjData(model)
        tmp.qpos[:] = data.qpos
        q, err = IK.ik_dls(model, tmp, qadr, np.asarray(target))
        for j in S.ARM_JOINTS:
            data.ctrl[aadr[j]] = q[S.ARM_JOINTS.index(j)]
        data.ctrl[aadr['gripper']] = grip
        for _ in range(steps):
            mujoco.mj_step(model, data)
        p = data.xpos[gl] + data.xmat[gl].reshape(3, 3) @ S.FRAME_IN_GRIPPER
        e = np.linalg.norm(p - np.asarray(target))
        if e < tol:
            break
    return e


def base_model(obj_size=0.020):
    """只编译 URDF 场景（爪盒子位置随便），用于读 body 变换。"""
    spec, _ = make_fast_spec(obj_size)
    return spec.compile()


ARM_HOME = None


def arm_home_qpos(model, data, qadr):
    """解一次 IK 得到"TCP 在物块上方 6cm"的臂关节角，做全局缓存。"""
    global ARM_HOME
    import sim_ik_dls as IK
    op = S.obj_world_pos()
    tmp = mujoco.MjData(model)
    mujoco.mj_resetData(model, tmp)
    q, err = IK.ik_dls(model, tmp, qadr, op + np.array([0, 0, 0.06]))
    ARM_HOME = q.copy()
    return q, err


def grasp_direct(obj_size=0.020, off_mm=(0, 0, 0), approach=0.6,
                 finger_half=(0.004, 0.009, 0.020),
                 fixed_pos=(0.010, 0.0, -0.070),
                 moving_pos=(0.0, -0.026, 0.0), settle=900, lift=700):
    """快速抓取测试：**直接**把臂 qpos 摆到"TCP=物块中心+off"的目标位姿，
    然后只发 ctrl 闭合+抬起。

    为什么可以这样：爪子的几何/碰撞与臂的构型无关，所以我们不需要每次
    重跑 400 次迭代的 IK。臂构型由 a1 = IK(TCP 在物块上方6cm) 与
    a2 = IK(TCP 在物块中心) 决定，而 a2 可以由 a1 的缓存 + 一次线性外推
    得到（两者只差一个 z 平移）。

    返回 (抬升mm, 固定爪接触数, 活动爪接触数, 夹紧后夹爪角)。
    """
    import sim_ik_dls as IK
    model, ffix, fmov = build_fast(obj_size, finger_half=finger_half,
                                   fixed_pos=fixed_pos, moving_pos=moving_pos)
    data = mujoco.MjData(model)
    q = qaddr(model)
    aadr = {}
    for i in range(model.nu):
        aadr[mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                               i).replace('servo_', '')] = i
    cb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'object')
    cg = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    cq = S.attach_handles(model)[3]
    op = S.obj_world_pos()
    off = np.asarray(off_mm) / 1000.0
    mujoco.mj_resetData(model, data)
    data.qpos[cq:cq + 3] = op
    data.qpos[cq + 3:cq + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)

    tmp = mujoco.MjData(model)
    q1, _ = IK.ik_dls(model, tmp, q, op + np.array([0, 0, 0.06]))
    tmp.qpos[:] = data.qpos
    q2, err2 = IK.ik_dls(model, tmp, q, op + off)
    for j in S.ARM_JOINTS:
        data.qpos[q[j]] = q1[S.ARM_JOINTS.index(j)]
    mujoco.mj_forward(model, data)
    for j in S.ARM_JOINTS:
        data.ctrl[aadr[j]] = q1[S.ARM_JOINTS.index(j)]
    data.ctrl[aadr['gripper']] = approach
    for _ in range(400):
        mujoco.mj_step(model, data)
    for j in S.ARM_JOINTS:
        data.ctrl[aadr[j]] = q2[S.ARM_JOINTS.index(j)]
    for _ in range(900):
        mujoco.mj_step(model, data)
    data.ctrl[aadr['gripper']] = 0.0
    for _ in range(settle):
        mujoco.mj_step(model, data)
    ang = data.qpos[q['gripper']]
    nfix = nmov = 0
    for c in range(data.ncon):
        cn = data.contact[c]
        if cg in (cn.geom1, cn.geom2):
            o = cn.geom2 if cn.geom1 == cg else cn.geom1
            if o == ffix:
                nfix += 1
            if o == fmov:
                nmov += 1
    tmp.qpos[:] = data.qpos
    q3, _ = IK.ik_dls(model, tmp, q, op + off + np.array([0, 0, 0.12]))
    for j in S.ARM_JOINTS:
        data.ctrl[aadr[j]] = q3[S.ARM_JOINTS.index(j)]
    data.ctrl[aadr['gripper']] = 0.0
    for _ in range(lift):
        mujoco.mj_step(model, data)
    up = (data.xpos[cb][2] - op[2]) * 1000
    return up, nfix, nmov, float(ang), float(err2)


def hold_test(obj_size=0.020, off_mm=(8, -4, 0), approach=0.6,
              finger_half=(0.004, 0.009, 0.020),
              fixed_pos=(0.010, 0.0, -0.070),
              moving_pos=(0.0, -0.026, 0.0), quiet=False):
    """用给定抓取参数跑一遍，看快速模型能否抬起物块。返回抬升(mm)。"""
    model, ffix, fmov = build_fast(obj_size, finger_half=finger_half,
                                   fixed_pos=fixed_pos,
                                   moving_pos=moving_pos)
    data = mujoco.MjData(model)
    q = qaddr(model)
    aadr = {}
    for i in range(model.nu):
        aadr[mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                               i).replace('servo_', '')] = i
    cb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'object')
    cg = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    cq = S.attach_handles(model)[3]
    op = S.obj_world_pos()
    mujoco.mj_resetData(model, data)
    data.qpos[cq:cq + 3] = op
    data.qpos[cq + 3:cq + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    tcp = op + np.asarray(off_mm) / 1000.0
    e1 = goto_ctrl(model, data, q, aadr, tcp + [0, 0, 0.06], 500, approach)
    e2 = goto_ctrl(model, data, q, aadr, tcp, 500, approach)
    data.ctrl[aadr['gripper']] = 0.0
    for _ in range(800):
        mujoco.mj_step(model, data)
    ang = data.qpos[q['gripper']]
    nfix = nmov = 0
    for c in range(data.ncon):
        cn = data.contact[c]
        if cg in (cn.geom1, cn.geom2):
            o = cn.geom2 if cn.geom1 == cg else cn.geom1
            if o == ffix:
                nfix += 1
            if o == fmov:
                nmov += 1
    e3 = goto_ctrl(model, data, q, aadr, tcp + [0, 0, 0.12], 600, 0.0)
    up = (data.xpos[cb][2] - op[2]) * 1000
    if not quiet:
        print(f'=== 快速模型抓取保真度（物块 {obj_size*1000:.0f}mm, '
              f'off={off_mm}mm, approach={approach}）===')
        print(f'  IK 残差: 预抓取{e1*1000:.2f}mm 下压{e2*1000:.2f}mm '
              f'抬起{e3*1000:.2f}mm')
        print(f'  夹紧后夹爪角={ang:+.3f}  固定爪接触={nfix} 活动爪接触={nmov}')
        print(f'  物块升高 {up:+.1f}mm  →  '
              f'{"✅ 抬起来了（模型可信）" if up > 5 else "❌ 没抬起"}')
    return up


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gap', action='store_true')
    ap.add_argument('--speed', action='store_true')
    ap.add_argument('--hold', action='store_true')
    ap.add_argument('--all', action='store_true')
    a = ap.parse_args()
    if a.all or a.gap:
        gap_scan()
    if a.all or a.speed:
        speed_bench()
    if a.all or a.hold:
        hold_test()
    if not (a.all or a.gap or a.speed or a.hold):
        ap.print_help()


if __name__ == '__main__':
    main()
