#!/usr/bin/env python3
"""物理夹持验证（正确版）：不瞬移，只发 ctrl 让伺服自己动

之前版本的问题：
  IK 求解过程中直接改 data.qpos（瞬移）→ 绕过约束求解器 → 物块被挤掉。
  正确做法：IK 只用来**算目标关节角**，算完把 qpos 恢复，然后只设 ctrl，
  让物理自己走过去。

本脚本验证：给一个"人为摆好 + 夹紧"的初始状态，抬升时物块能不能被带走。
这是上 RL 前的先决条件 —— 如果物理都拿不住，RL 也学不出来。

用法：
  ~/mj/bin/python sim_hold_test.py --obj-size 0.020
"""

import argparse
import sys

import numpy as np

import mujoco

import sim_grasp as S
import sim_mesh_gripper as MG
import sim_ik_dls as IK


def solve_qpos_only(model, qadr, target, cur_q):
    """只求解关节角，不改变仿真状态（求解后恢复 qpos）。"""
    import copy
    data_tmp = mujoco.MjData(model)
    data_tmp.qpos[:] = cur_q
    q, err = IK.ik_dls(model, data_tmp, qadr, np.asarray(target))
    return q, err


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--obj-size', type=float, default=0.020)
    ap.add_argument('--offset', nargs=3, type=float, default=[8.0, -4.0, 0.0],
                    help='TCP 相对物块的偏移(mm)')
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
    gl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper_link')
    mjb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                            'moving_jaw_so101_v1_link')
    cg = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    cq = S.attach_handles(model)[3]
    cb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'object')
    op = S.obj_world_pos()
    off = np.array(a.offset) / 1000.0
    tcp = op + off

    # 摆位（IK 求解，然后把 qpos 恢复）
    q_sol, err = solve_qpos_only(model, qadr, tcp, data.qpos.copy())
    for j in S.ARM_JOINTS:
        data.qpos[qadr[j]] = q_sol[S.ARM_JOINTS.index(j)]
    data.qpos[cq:cq + 3] = op
    data.qpos[cq + 3:cq + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    print(f'摆位: TCP 目标 {np.round(tcp,4)}（IK 残差 {err*1000:.2f}mm）')

    # 找"两爪同时接触"的夹爪角
    found = None
    for ang in np.arange(0.30, -0.011, -0.01):
        data.qpos[qadr['gripper']] = ang
        mujoco.mj_forward(model, data)
        nf = nm = 0
        for c in range(data.ncon):
            cn = data.contact[c]
            if cg in (cn.geom1, cn.geom2):
                o = cn.geom2 if cn.geom1 == cg else cn.geom1
                if model.geom_bodyid[o] == gl:
                    nf += 1
                if model.geom_bodyid[o] == mjb:
                    nm += 1
        if nf and nm:
            found = ang
            print(f'两爪同时接触的角 = {ang:.2f}（固定侧{nf} 活动侧{nm}）')
            break
    ang = found if found is not None else 0.10

    # 夹紧：只发 ctrl，不瞬移
    for j in S.ARM_JOINTS:
        data.ctrl[aadr[j]] = data.qpos[qadr[j]]
    data.ctrl[aadr['gripper']] = 0.0            # 全力闭合
    for _ in range(800):
        mujoco.mj_step(model, data)
    print(f'夹紧后: 夹爪角={data.qpos[qadr["gripper"]]:+.3f} '
          f'物块偏离={np.round((data.xpos[cb]-op)*1000,1)}mm 接触={data.ncon}')
    # 记录夹持力
    f = np.zeros(6)
    for c in range(data.ncon):
        cn = data.contact[c]
        if cg in (cn.geom1, cn.geom2):
            mujoco.mj_contactForce(model, data, c, f)
            print(f'  接触力 |f|={np.linalg.norm(f[:3]):.3f} N')

    # 抬起：只改 ctrl（用求解好的关节角），不瞬移
    print('\n抬升（只发 ctrl，伺服自己动）:')
    for k in range(1, 9):
        t = tcp + np.array([0, 0, 0.02 * k])
        q_sol, err = solve_qpos_only(model, qadr, t, data.qpos.copy())
        for j in S.ARM_JOINTS:
            data.ctrl[aadr[j]] = q_sol[S.ARM_JOINTS.index(j)]
        data.ctrl[aadr['gripper']] = 0.0
        for _ in range(400):
            mujoco.mj_step(model, data)
        print(f'  抬到 +{0.02*k*1000:.0f}mm: 物块升高 '
              f'{(data.xpos[cb][2]-op[2])*1000:+7.1f}mm  '
              f'接触={data.ncon} 夹爪角={data.qpos[qadr["gripper"]]:+.3f}',
              flush=True)
    up = (data.xpos[cb][2] - op[2]) * 1000
    print(f'\n结论: 物块最终升高 {up:+.1f}mm  '
          f'{"🎉 物理上能夹住 —— 可以上 RL" if up > 5 else "❌ 物理拿不住（建模/摩擦问题）"}')


if __name__ == '__main__':
    main()
