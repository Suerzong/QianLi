#!/usr/bin/env python3
"""精确测量夹爪的"口"：两爪内侧面在 gripper_frame_link 下的位置

目的：
  上面的诊断发现物块在 frame 下 X=-20.7mm（开合方向），说明 **TCP 原点
  不在爪口中心**。这里用网格顶点算出真正的爪口中心与宽度，作为抓取偏移。

方法：
  1. 取 gripper_link（含固定爪）与 moving_jaw 的网格顶点
  2. 变换到 gripper_frame_link 坐标系
  3. 只看"指尖附近"的顶点（沿工具轴 Z 最低的一段），并按 |Y| 限制
  4. 分别统计两爪在 X 方向的分布 → 得到内侧面位置、口宽、口中心
"""

from project_paths import so101_path

import math
import os
import struct
import sys

import numpy as np

import mujoco

URDF = os.path.expanduser(
    so101_path('urdf/so101.urdf'))
ASSETS = os.path.join(os.path.dirname(URDF), 'assets')


def load_stl(name):
    with open(os.path.join(ASSETS, name), 'rb') as f:
        f.read(80)
        n = struct.unpack('<I', f.read(4))[0]
        d = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
        return d[:, 12:48].copy().view('<f4').reshape(-1, 3).astype(float)


def main():
    import sim_grasp as S

    model = S.build_model()
    data = mujoco.MjData(model)
    qadr = {}
    for i in range(model.njnt):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if n:
            qadr[n] = model.jnt_qposadr[i]
    gl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper_link')
    mj = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                           'moving_jaw_so101_v1_link')
    R_fix = np.array([[-1.0, 0, 0], [0, 1, 0], [0, 0, -1.0]])

    # gripper_link 下的网格（固定爪在 wrist_roll_follower 里）
    va = load_stl('wrist_roll_follower_so101_v1.stl')
    # moving_jaw 网格在自身 link 下偏移 (0,0,0.0189)
    vb = load_stl('moving_jaw_so101_v1.stl') + np.array([0, 0, 0.0189])

    def to_frame(pts, body, R):
        return (pts @ (data.xmat[body].reshape(3, 3).T)).dot(R) + \
            (data.xpos[body] + data.xmat[body].reshape(3, 3) @ S.FRAME_IN_GRIPPER)

    # 先摆一个工具朝下的姿态，便于理解
    ch, names = S.make_chain()
    seed = np.zeros(len(ch.links))
    sol, _ = S.solve_ik(ch, names, [0.30, 0.0, 0.05], -90.0, seed)
    for j in S.ARM_JOINTS:
        data.qpos[qadr[j]] = sol[names.index(j)]

    for g in (1.2, 0.0):
        data.qpos[qadr['gripper']] = g
        mujoco.mj_forward(model, data)
        # 转到 frame 坐标
        Rgl = data.xmat[gl].reshape(3, 3)
        pgl = data.xpos[gl] + Rgl @ S.FRAME_IN_GRIPPER
        Rf = Rgl @ R_fix
        A = ((va @ Rgl.T) + data.xpos[gl] - pgl) @ Rf
        Rmj = data.xmat[mj].reshape(3, 3)
        B = ((vb @ Rmj.T) + data.xpos[mj] - pgl) @ Rf
        # 只看指口深度范围（Z 在 -10mm~+20mm）且靠近开合平面（|Y|<15mm）
        fa = A[(A[:, 2] > -0.010) & (A[:, 2] < 0.020) & (np.abs(A[:, 1]) < 0.015)]
        fb = B[(B[:, 2] > -0.010) & (B[:, 2] < 0.020) & (np.abs(B[:, 1]) < 0.015)]
        print(f'gripper={g}:  固定爪顶点 {len(fa)}, 活动爪顶点 {len(fb)}')
        if len(fa) and len(fb):
            print(f'  固定爪 X 范围: {fa[:,0].min()*1000:+.1f} ~ '
                  f'{fa[:,0].max()*1000:+.1f} mm')
            print(f'  活动爪 X 范围: {fb[:,0].min()*1000:+.1f} ~ '
                  f'{fb[:,0].max()*1000:+.1f} mm')
            # 内侧面：固定爪靠近中心的最大 X、活动爪靠近中心的最小 X
            inner_fixed = fa[:, 0].max()
            inner_move = fb[:, 0].min()
            print(f'  两爪内侧: 固定 {inner_fixed*1000:+.1f} mm / '
                  f'活动 {inner_move*1000:+.1f} mm  →  口宽 '
                  f'{(inner_fixed-inner_move)*1000:.1f} mm，'
                  f'口中心 X = {((inner_fixed+inner_move)/2)*1000:+.1f} mm')
        print()

    print('结论：抓取时应把物块放在 frame 的 X = 口中心 处，'
          '即给 TCP 目标加上补偿偏移')


if __name__ == '__main__':
    main()
