#!/usr/bin/env python3
"""诊断：物块相对夹爪坐标系的位置（判断是否落在爪口开合轴线上）

原理：
  夹爪的爪口沿 gripper_frame_link 的 +X 轴开合（+X 侧固定爪，-X 侧活动爪），
  工具轴为 +Z（指向指尖）。要让物块能被夹住，它必须：
    · 在 frame 的 Z 方向落在爪口深度范围内
    · 在 frame 的 Y 方向接近 0（在开合平面内）
    · 在 frame 的 X 方向位于两爪之间
  若 Y 偏大 → 夹爪从旁边擦过（就是"夹空"）
"""

import math
import sys

import numpy as np

import mujoco

sys.argv = [sys.argv[0]]
import sim_grasp as S   # noqa: E402


def main():
    size = float(sys.argv[1]) if len(sys.argv) > 1 else 0.014
    S._OBJ_SIZE_OVERRIDE[0] = size

    model = S.build_model()
    data = mujoco.MjData(model)
    ch, names = S.make_chain()
    qadr, aadr, cube, cq = S.attach_handles(model)
    op = S.obj_world_pos()
    z = S.TABLE_Z + 0.003 + size / 2
    gl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper_link')

    # frame 在 gripper_link 下的固定变换：平移 + rpy(0,π,0)
    R_fix = np.array([[-1.0, 0, 0], [0, 1, 0], [0, 0, -1.0]])   # Ry(π)

    def frame_pose():
        p = data.xpos[gl].copy()
        R = data.xmat[gl].reshape(3, 3).copy()
        # frame 原点在 gripper_link 下的位置
        p_f = p + R @ S.FRAME_IN_GRIPPER
        R_f = R @ R_fix
        return p_f, R_f

    print(f'物块 {size*1000:.0f}mm @ ({op[0]:.4f}, {op[1]:.4f}, {op[2]:.4f})')
    print(f'目标抓取高度(TCP z) = {z:+.4f}')
    print()
    print('wrist_roll  物块在frame下的位置(X=开合向, Y=面内, Z=工具轴)   Y偏离')
    for roll in [-1.571, -1.178, -0.785, -0.393, 0.0, 0.393, 0.785, 1.178,
                 1.571]:
        mujoco.mj_resetData(model, data)
        data.qpos[cq:cq + 3] = op
        data.qpos[cq + 3:cq + 7] = [1, 0, 0, 0]
        mujoco.mj_forward(model, data)
        seed = np.zeros(len(ch.links))
        s1, _ = S.solve_ik(ch, names, [op[0], op[1], z + 0.06], -90.0, seed)
        s2, _ = S.solve_ik(ch, names, [op[0], op[1], z], -90.0, s1)
        s2[names.index('wrist_roll')] = roll
        for j in S.ARM_JOINTS:
            data.ctrl[aadr[j]] = s2[names.index(j)]
        data.ctrl[aadr['gripper']] = 1.2
        for _ in range(1400):
            mujoco.mj_step(model, data)
        p_f, R_f = frame_pose()
        rel = R_f.T @ (op - p_f)          # 物块中心在 frame 下
        print(f'  {roll:+.3f}     ({rel[0]*1000:+6.1f}, {rel[1]*1000:+6.1f}, '
              f'{rel[2]*1000:+6.1f}) mm        {abs(rel[1])*1000:5.1f}mm',
              flush=True)
    print()
    print('参考：爪口开合沿 frame X（两爪内侧开度约 19.6mm）')
    print('      Y 偏离越小越好（< 3mm 才算在开合平面内）')


if __name__ == '__main__':
    main()
