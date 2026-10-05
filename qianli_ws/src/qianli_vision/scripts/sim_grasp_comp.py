#!/usr/bin/env python3
"""按 frame 诊断精确补偿后抓取（把物块对到爪口中心）

思路：
  诊断显示物块在夹爪坐标系里 X=-20.7mm（开合方向），即 TCP 原点不在爪口
  中心。这里先算出夹爪坐标系在世界的 X 轴，把目标点沿 -X 平移该偏移，
  使物块落到 frame 的 X≈0（爪口中心），再用较强夹爪力闭合、抬起。
"""

import math
import sys

import numpy as np

import mujoco

sys.argv = [sys.argv[0]]
import sim_grasp as S   # noqa: E402


def main():
    size = float(sys.argv[1]) if len(sys.argv) > 1 else 0.014
    roll = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    kp_grip = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0
    S._OBJ_SIZE_OVERRIDE[0] = size

    model = S.build_model()
    # 夹爪力加强（真舵机堵转扭矩约 2.9 N·m）
    for i in range(model.nu):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if n == 'servo_gripper':
            model.actuator_gainprm[i][0] = kp_grip
            model.actuator_biasprm[i][1] = -kp_grip
            model.actuator_biasprm[i][2] = -2.0 * math.sqrt(kp_grip) * 0.2
            model.actuator_forcerange[i][:] = [-3.0, 3.0]
    data = mujoco.MjData(model)
    ch, names = S.make_chain()
    qadr, aadr, cube, cq = S.attach_handles(model)
    op = S.obj_world_pos()
    z = S.TABLE_Z + 0.003 + size / 2
    gl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper_link')
    cg = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    arm_ids = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n) for n in
               ['shoulder_link', 'upper_arm_link', 'lower_arm_link',
                'wrist_link', 'gripper_link', 'moving_jaw_so101_v1_link']}
    R_fix = np.array([[-1.0, 0, 0], [0, 1, 0], [0, 0, -1.0]])

    def frame_axes():
        R = data.xmat[gl].reshape(3, 3)
        return R @ R_fix

    def frame_origin():
        R = data.xmat[gl].reshape(3, 3)
        return data.xpos[gl] + R @ S.FRAME_IN_GRIPPER

    def arm_contacts():
        n = 0
        for c in range(data.ncon):
            cn = data.contact[c]
            if cg in (cn.geom1, cn.geom2):
                o = cn.geom2 if cn.geom1 == cg else cn.geom1
                if model.geom_bodyid[o] in arm_ids:
                    n += 1
        return n

    # 第一步：先到物块上方，测出 frame 指向与物块在 frame 下的偏移
    mujoco.mj_resetData(model, data)
    data.qpos[cq:cq + 3] = op
    data.qpos[cq + 3:cq + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    seed = np.zeros(len(ch.links))
    s_probe, _ = S.solve_ik(ch, names, [op[0], op[1], z], -90.0, seed)
    s_probe[names.index('wrist_roll')] = roll
    for j in S.ARM_JOINTS:
        data.qpos[qadr[j]] = s_probe[names.index(j)]
    data.qpos[qadr['gripper']] = 1.2
    mujoco.mj_forward(model, data)
    Rf = frame_axes()
    pf = frame_origin()
    rel = Rf.T @ (op - pf)
    print(f'补偿前：物块在 frame 下 = ({rel[0]*1000:+.1f}, {rel[1]*1000:+.1f}, '
          f'{rel[2]*1000:+.1f}) mm')
    # 需要把 frame 沿自身 X 平移 rel[0]，使物块落在 X=0
    shift_world = Rf @ np.array([rel[0], rel[1], 0.0])
    tx, ty = op[0] - shift_world[0], op[1] - shift_world[1]
    print(f'补偿：TCP 目标平移 ({-shift_world[0]*1000:+.1f}, '
          f'{-shift_world[1]*1000:+.1f}) mm')
    print(f'新目标 = ({tx:.4f}, {ty:.4f}, {z:+.4f})')

    # 第二步：用补偿后的目标跑完整抓取
    mujoco.mj_resetData(model, data)
    data.qpos[cq:cq + 3] = op
    data.qpos[cq + 3:cq + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    s1, _ = S.solve_ik(ch, names, [tx, ty, z + 0.06], -90.0, seed)
    s1[names.index('wrist_roll')] = roll
    for j in S.ARM_JOINTS:
        data.ctrl[aadr[j]] = s1[names.index(j)]
    data.ctrl[aadr['gripper']] = 1.2
    for _ in range(800):
        mujoco.mj_step(model, data)
    s2, _ = S.solve_ik(ch, names, [tx, ty, z], -90.0, s1)
    s2[names.index('wrist_roll')] = roll
    for j in S.ARM_JOINTS:
        data.ctrl[aadr[j]] = s2[names.index(j)]
    for _ in range(1000):
        mujoco.mj_step(model, data)
    Rf = frame_axes()
    rel2 = Rf.T @ (op - frame_origin())
    print(f'下压后：物块在 frame 下 = ({rel2[0]*1000:+.1f}, {rel2[1]*1000:+.1f}, '
          f'{rel2[2]*1000:+.1f}) mm，爪-物块接触={arm_contacts()}')
    for k in range(80):
        data.ctrl[aadr['gripper']] = 1.2 * (1 - k / 79.0)
        for _ in range(15):
            mujoco.mj_step(model, data)
    print(f'闭合后：夹爪角={data.qpos[qadr["gripper"]]:+.3f} '
          f'（0=全闭；被物块挡住应>0.05） 接触={arm_contacts()}')
    s3, _ = S.solve_ik(ch, names, [tx, ty, z + 0.10], -90.0, s2)
    s3[names.index('wrist_roll')] = roll
    for j in S.ARM_JOINTS:
        data.ctrl[aadr[j]] = s3[names.index(j)]
    for _ in range(1500):
        mujoco.mj_step(model, data)
    oz = data.xpos[cube][2]
    print(f'抬起后：物块 z={oz:+.4f}（起始 {op[2]:+.4f}）接触={arm_contacts()}')
    print('结果:', '🎉 抓起来了！' if oz > op[2] + 0.005 else '❌ 没抓起来')


if __name__ == '__main__':
    main()
