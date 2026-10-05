#!/usr/bin/env python3
"""数字孪生实时显示（MuJoCo 窗口，跑在 Ubuntu 桌面上）

界面操作（MuJoCo 自带）：
  左键拖动 = 旋转视角    右键拖动 = 平移
  滚轮     = 缩放        Ctrl+左键 = 施加外力
  空格     = 暂停        Tab = 切换面板

脚本行为：
  在仿真里循环演示抓取流程（预抓取 → 下压 → 闭合 → 抬起 → 复位），
  并打印每一步结果。可在窗口里看到真实的接触/摩擦/夹持效果。

用法（在 VM 上带桌面显示运行）：
  MUJOCO_GL=glfw ~/mj/bin/python twin_viewer.py
  MUJOCO_GL=glfw ~/mj/bin/python twin_viewer.py --once   # 只跑一次
"""

import argparse
import math
import os
import sys
import time

import numpy as np

import mujoco

sys.argv = [sys.argv[0]]
import sim_grasp as S   # noqa: E402


def build_strong_servo():
    """和 sim_grasp 同样的场景 + 强伺服 + 正确的碰撞分组。

    碰撞分组（位掩码）——这是让仿真能用的关键：
      机械臂  contype=1, conaffinity=0
      物块    contype=2, conaffinity=1
      桌子/棋盘 contype=4, conaffinity=2
    接触判定 = (contype_a & conaffinity_b) | (contype_b & conaffinity_a)
      臂-臂    (1&0)|(1&0)=0  不碰 ✅（自碰撞会产生 75N·m 约束力压过伺服）
      臂-桌    (1&2)|(4&0)=0  不碰 ✅（网格穿透地板同样会顶住伺服）
      臂-物块  (1&1)|(2&0)=1  碰 ✅
      物块-桌  (2&2)|(4&1)=2  碰 ✅
    """
    model = S.build_model()
    arm_body_ids = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
                    for n in ['shoulder_link', 'upper_arm_link',
                              'lower_arm_link', 'wrist_link',
                              'gripper_link', 'moving_jaw_so101_v1_link']}
    env_geoms = set()
    for nm in ('table', 'pedestal', 'board'):
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, nm)
        if gid >= 0:
            env_geoms.add(gid)
    cube_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')

    for i in range(model.ngeom):
        if model.geom_bodyid[i] in arm_body_ids:
            model.geom_contype[i] = 1
            model.geom_conaffinity[i] = 0
        elif i in env_geoms:
            model.geom_contype[i] = 4
            model.geom_conaffinity[i] = 2
        elif i == cube_gid:
            model.geom_contype[i] = 2
            model.geom_conaffinity[i] = 1

    for i in range(model.nu):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        j = n.replace('servo_', '')
        if j == 'gripper':
            model.actuator_gainprm[i][0] = 8.0
            model.actuator_biasprm[i][1] = -8.0
            model.actuator_forcerange[i][:] = [-1.5, 1.5]
        else:
            kp = 120.0
            model.actuator_gainprm[i][0] = kp
            model.actuator_biasprm[i][1] = -kp
            model.actuator_biasprm[i][2] = -2.0 * math.sqrt(kp) * 0.2
            model.actuator_forcerange[i][:] = [-4.0, 4.0]
    model.vis.headlight.ambient[:] = [0.5, 0.5, 0.5]
    model.vis.headlight.diffuse[:] = [0.8, 0.8, 0.8]
    return model


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--z', type=float, default=-0.045, help='抓取高度')
    ap.add_argument('--yaw', type=float, default=-90.0)
    ap.add_argument('--once', action='store_true')
    ap.add_argument('--dwell', type=float, default=1.2)
    a = ap.parse_args()

    model = build_strong_servo()
    data = mujoco.MjData(model)
    ch, names = S.make_chain()
    op = S.obj_world_pos()

    qadr, aadr = {}, {}
    for i in range(model.njnt):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if n:
            qadr[n] = model.jnt_qposadr[i]
    for i in range(model.nu):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        aadr[n.replace('servo_', '')] = i
    cube = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'object')
    # 物块的 freejoint（qpos: 3 位置 + 4 姿态）
    cube_qadr = None
    for i in range(model.njnt):
        if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE and \
                model.jnt_bodyid[i] == cube:
            cube_qadr = model.jnt_qposadr[i]
            break

    print(f'物块 (base_link) = {np.round(op, 4)}')
    print(f'桌面 z = {S.TABLE_Z:.4f}，抓取高度 z = {a.z:.4f}')
    print('打开 MuJoCo 窗口…（左键旋转 / 右键平移 / 滚轮缩放 / 空格暂停）')

    import mujoco.viewer
    viewer = mujoco.viewer.launch_passive(model, data)

    def set_arm(target_xyz, seed):
        sol, err = S.solve_ik(ch, names, target_xyz, a.yaw, seed)
        for j in S.ARM_JOINTS:
            if j in names:
                data.ctrl[aadr[j]] = sol[names.index(j)]
        return sol, err

    def step_until(seconds, grip=None):
        t0 = time.time()
        while viewer.is_running() and time.time() - t0 < seconds:
            if grip is not None:
                data.ctrl[aadr['gripper']] = grip
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)

    def reset_all():
        mujoco.mj_resetData(model, data)
        if cube_qadr is not None:
            data.qpos[cube_qadr:cube_qadr + 3] = op
            data.qpos[cube_qadr + 3:cube_qadr + 7] = [1, 0, 0, 0]
        mujoco.mj_forward(model, data)

    cycle = 0
    while viewer.is_running():
        cycle += 1
        print(f'\n=== 第 {cycle} 轮仿真 ===')
        reset_all()
        seed = np.zeros(len(ch.links))

        # 1) 预抓取（物块上方 6cm）
        sol, err = set_arm([op[0], op[1], a.z + 0.06], seed)
        step_until(1.2, grip=1.2)
        print(f'  预抓取 IK 误差 {err*1000:.1f}mm')

        # 2) 下压
        sol2, err2 = set_arm([op[0], op[1], a.z], sol)
        step_until(1.5)
        print(f'  下压   IK 误差 {err2*1000:.1f}mm，物块 z={data.xpos[cube][2]:+.4f}')

        # 3) 慢速闭合
        for k in range(50):
            data.ctrl[aadr['gripper']] = 1.2 * (1 - k / 49.0)
            step_until(0.02)
        print(f'  闭合后 物块 z={data.xpos[cube][2]:+.4f}')

        # 4) 抬起
        sol3, _ = set_arm([op[0], op[1], a.z + 0.10], sol2)
        step_until(1.8)
        oz = data.xpos[cube][2]
        lifted = oz > op[2] + 0.02
        print(f'  抬起后 物块 z={oz:+.4f} → '
              f'{"✅ 抓起来了！" if lifted else "❌ 没抓起来"}')

        step_until(a.dwell)
        if a.once:
            print('\n（--once：保持窗口，关闭窗口退出）')
            while viewer.is_running():
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(model.opt.timestep)
            break

    viewer.close()


if __name__ == '__main__':
    main()
