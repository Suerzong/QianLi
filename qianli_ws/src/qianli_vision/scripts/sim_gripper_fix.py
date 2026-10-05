#!/usr/bin/env python3
"""把夹爪碰撞体从"网格凸包"换成基本体（盒子），修复糊住的口

依据（本会话实测）：
  · 爪子所有碰撞 geom 都是 MESH → MuJoCo 用凸包
  · 两爪凸包从全开到全闭一直穿透 -20.2 ~ -21.9 mm → 口是糊死的
  · 所以仿真里大物块塞不进去，是伪像

做法：
  · 原爪子网格 geom 的碰撞全部关掉（保留视觉）
  · 新增两个盒子当"两片爪"，一片固定在 gripper_link，
    一片挂在 moving_jaw 上（随夹爪关节转动）
  · 用引擎的 mj_geomDistance 量新爪的口宽随角度的变化

用法：
  ~/mj/bin/python sim_gripper_fix.py --check       # 看口宽-角度曲线
  ~/mj/bin/python sim_gripper_fix.py --render      # 渲染确认形状
  ~/mj/bin/python sim_gripper_fix.py --scan        # 重跑尺寸扫描
"""

import argparse
import math
import sys

import numpy as np

import mujoco

# 注意：不要在这里写 sys.argv = [sys.argv[0]]！
# 那会把本脚本 main() 的 --tune/--scan 等参数一起清掉（踩过坑）。
import sim_grasp as S   # noqa: E402

# 两片爪的盒子参数（半尺寸，米）
FINGER_HALF = (0.004, 0.009, 0.020)     # 4×9×20mm 半长 → 8×18×40mm 指
# 摆位由 --tune 搜出来的（口宽：全开≈35mm、1.2≈28mm、闭合≈2mm，随角度单调）
FIXED_POS = (0.010, 0.0, -0.070)        # 固定爪在 gripper_link 下的位置
MOVING_POS = (0.0, -0.026, 0.0)         # 活动爪在 moving_jaw 下的位置
# moving_jaw 的关节 rpy=(π/2,0,0)：其局部 -y 对应 gripper_link 的 -z（向下）


def build_fixed_gripper(obj_size, fixed_pos=None, moving_pos=None):
    """带基本体爪子的模型。"""
    fp = fixed_pos or FIXED_POS
    mp = moving_pos or MOVING_POS
    S._OBJ_SIZE_OVERRIDE[0] = obj_size
    spec = mujoco.MjSpec.from_file(S.URDF)
    wb = spec.worldbody
    # 场景（与 sim_grasp 一致）
    gt = wb.add_geom(); gt.name = 'table'
    gt.type = mujoco.mjtGeom.mjGEOM_BOX
    gt.size = [0.4, 0.4, 0.15]; gt.pos = [0.3, 0.0, S.TABLE_Z - 0.15]
    gt.rgba = [0.55, 0.55, 0.58, 1]
    gp = wb.add_geom(); gp.name = 'pedestal'
    gp.type = mujoco.mjtGeom.mjGEOM_BOX
    gp.size = [0.045, 0.05, (S.BASE_BOTTOM - S.TABLE_Z) / 2]
    gp.pos = [0.0, 0.0, (S.TABLE_Z + S.BASE_BOTTOM) / 2]
    gp.rgba = [0.3, 0.3, 0.32, 1]
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

    # 两片爪：挂在 gripper_link / moving_jaw 上
    for bname, pos, gname, rgba in (
            ('gripper_link', fp, 'finger_fixed', [0.9, 0.8, 0.2, 1]),
            ('moving_jaw_so101_v1_link', mp, 'finger_moving',
             [0.95, 0.85, 0.25, 1])):
        body = spec.body(bname)
        g = body.add_geom()
        g.name = gname
        g.type = mujoco.mjtGeom.mjGEOM_BOX
        g.size = list(FINGER_HALF)
        g.pos = list(pos)
        g.rgba = rgba

    # 执行器
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
    # 碰撞分组：臂只和物块碰；桌子/棋盘只和物块碰；关掉原爪网格碰撞
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
            # 原来的网格爪/臂：全部不参与碰撞（凸包会糊住口）
            model.geom_contype[i], model.geom_conaffinity[i] = 0, 0
        elif i in env:
            model.geom_contype[i], model.geom_conaffinity[i] = 2, 2
        elif i == cube:
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
    return model, ffix, fmov


def mouth_gap(model, data, a, b, qadr, angles):
    out = []
    for g in angles:
        data.qpos[qadr['gripper']] = g
        mujoco.mj_forward(model, data)
        d = mujoco.mj_geomDistance(model, data, a, b, 1.0, None)
        out.append((g, d))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true')
    ap.add_argument('--render')
    ap.add_argument('--scan', action='store_true')
    ap.add_argument('--tune', action='store_true',
                    help='搜索两片爪的正确摆位（口宽随角度单调、全开够大）')
    a = ap.parse_args()

    if a.tune:
        print('=== 搜索爪子摆位（目标：全开 35~45mm，闭合 0~3mm）===')
        print('  moving_pos      fixed_x   开口(1.745) 开口(1.2) 闭合(0.0)')
        for mpos in [(0.0, 0.026, 0.0), (0.0, -0.026, 0.0),
                     (0.026, 0.0, 0.0), (-0.026, 0.0, 0.0)]:
            for fx in (-0.030, -0.018, -0.006, 0.006, 0.018, 0.030):
                try:
                    m, ffix, fmov = build_fixed_gripper(
                        0.020, fixed_pos=(fx, 0.0, -0.070), moving_pos=mpos)
                except Exception as e:
                    print(f'  {mpos}  x={fx:+.3f}  构建失败: {e}')
                    continue
                d = mujoco.MjData(m)
                q = {}
                for i in range(m.njnt):
                    n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i)
                    if n:
                        q[n] = m.jnt_qposadr[i]
                g = mouth_gap(m, d, ffix, fmov, q, [1.745, 1.2, 0.0])
                print(f'  {str(mpos):14s} {fx:+.3f}   '
                      f'{g[0][1]*1000:8.1f}  {g[1][1]*1000:8.1f}  '
                      f'{g[2][1]*1000:8.1f}', flush=True)
        return

    model, ffix, fmov = build_fixed_gripper(0.020)
    data = mujoco.MjData(model)
    qadr = {}
    for i in range(model.njnt):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if n:
            qadr[n] = model.jnt_qposadr[i]

    if a.check or not (a.render or a.scan):
        print('=== 新爪子的口宽 vs 夹爪角 ===')
        print('  夹爪角    口宽(mm)')
        for g, d in mouth_gap(model, data, ffix, fmov, qadr,
                              [1.745, 1.2, 0.8, 0.4, 0.0, -0.1745]):
            print(f'  {g:+.4f}   {d*1000:8.1f}')
        print('\n参考：真机完全张开约 30~40mm，完全闭合 ≈ 0'
              '（口宽应随角单调变化）')

    if a.render:
        import cv2
        ch, names = S.make_chain()
        seed = np.zeros(len(ch.links))
        sol, _ = S.solve_ik(ch, names, [0.30, 0.0, 0.05], -90.0, seed)
        for j in S.ARM_JOINTS:
            data.qpos[qadr[j]] = sol[names.index(j)]
        model.vis.global_.offwidth = 900
        model.vis.global_.offheight = 700
        model.vis.headlight.ambient[:] = [0.6, 0.6, 0.6]
        r = mujoco.Renderer(model, 700, 900)
        cam = mujoco.MjvCamera()
        gl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                               'gripper_link')
        for g, tag in ((1.745, 'open'), (0.0, 'closed')):
            data.qpos[qadr['gripper']] = g
            mujoco.mj_forward(model, data)
            p = data.xpos[gl] + data.xmat[gl].reshape(3, 3) @ S.FRAME_IN_GRIPPER
            cam.lookat[:] = [p[0], p[1], p[2] - 0.03]
            cam.distance = 0.22
            cam.azimuth = 90
            cam.elevation = -25
            r.update_scene(data, cam)
            img = r.render()
            cv2.imwrite(a.render.replace('.png', f'_{tag}.png'),
                        img[:, :, ::-1])
            print(f'渲染 {a.render.replace(".png", f"_{tag}.png")}')

    if a.scan:
        import sim_sweep_offset as SW
        ch, names = S.make_chain()
        print('=== 基本体爪子下重跑尺寸扫描（dz=-30mm） ===')
        print('  尺寸    结果      接触')
        for sz in (0.014, 0.018, 0.020, 0.024, 0.030):
            m, _, _ = build_fixed_gripper(sz)
            d = mujoco.MjData(m)
            ok, nc, moved, ga = SW.trial(m, d, ch, names, 0.0, 0.0, -0.030, sz)
            print(f'  {sz*1000:4.0f}mm  {"✅ 抓起" if ok else "❌ 没抓起"}  '
                  f'{nc:2d}', flush=True)


if __name__ == '__main__':
    main()
