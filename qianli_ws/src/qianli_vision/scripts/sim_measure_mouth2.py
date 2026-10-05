#!/usr/bin/env python3
"""测量夹爪几何：两爪网格在 gripper_frame_link 下的包围盒（分高度层）

输出每一"层"（沿工具轴 Z 分层）两爪的 X 范围 → 直观看出爪口在哪、多宽。
"""

import os
import struct
import sys

import numpy as np

import mujoco

sys.argv = [sys.argv[0]]
import sim_grasp as S   # noqa: E402

URDF = S.URDF
ASSETS = os.path.join(os.path.dirname(URDF), 'assets')


def load_stl(name):
    with open(os.path.join(ASSETS, name), 'rb') as f:
        f.read(80)
        n = struct.unpack('<I', f.read(4))[0]
        d = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
        return d[:, 12:48].copy().view('<f4').reshape(-1, 3).astype(float)


def main():
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
    va = load_stl('wrist_roll_follower_so101_v1.stl')
    vb = load_stl('moving_jaw_so101_v1.stl') + np.array([0, 0, 0.0189])

    ch, names = S.make_chain()
    seed = np.zeros(len(ch.links))
    sol, _ = S.solve_ik(ch, names, [0.30, 0.0, 0.05], -90.0, seed)
    for j in S.ARM_JOINTS:
        data.qpos[qadr[j]] = sol[names.index(j)]

    for g in (1.2, 0.0):
        data.qpos[qadr['gripper']] = g
        mujoco.mj_forward(model, data)
        Rgl = data.xmat[gl].reshape(3, 3)
        pgl = data.xpos[gl] + Rgl @ S.FRAME_IN_GRIPPER
        Rf = Rgl @ R_fix
        A = ((va @ Rgl.T) + data.xpos[gl] - pgl) @ Rf
        Rmj = data.xmat[mj].reshape(3, 3)
        B = ((vb @ Rmj.T) + data.xpos[mj] - pgl) @ Rf
        print(f'\n===== gripper = {g} =====')
        print(f'固定爪(gripper_link) 全包围盒: X[{A[:,0].min()*1000:+.1f},'
              f'{A[:,0].max()*1000:+.1f}] Y[{A[:,1].min()*1000:+.1f},'
              f'{A[:,1].max()*1000:+.1f}] Z[{A[:,2].min()*1000:+.1f},'
              f'{A[:,2].max()*1000:+.1f}] mm')
        print(f'活动爪(moving_jaw)  全包围盒: X[{B[:,0].min()*1000:+.1f},'
              f'{B[:,0].max()*1000:+.1f}] Y[{B[:,1].min()*1000:+.1f},'
              f'{B[:,1].max()*1000:+.1f}] Z[{B[:,2].min()*1000:+.1f},'
              f'{B[:,2].max()*1000:+.1f}] mm')
        print('\n 沿工具轴分层（只取 |Y|<12mm 的顶点）:')
        print('   Z 层(mm)     固定爪X范围        活动爪X范围       口宽(mm)')
        for z0, z1 in [(0.000, 0.010), (-0.010, 0.000), (-0.020, -0.010),
                       (-0.030, -0.020), (-0.040, -0.030)]:
            fa = A[(A[:, 2] >= z0) & (A[:, 2] < z1) & (np.abs(A[:, 1]) < 0.012)]
            fb = B[(B[:, 2] >= z0) & (B[:, 2] < z1) & (np.abs(B[:, 1]) < 0.012)]
            if len(fa) < 5 or len(fb) < 5:
                print(f'   {z0*1000:+5.0f}~{z1*1000:+5.0f}     '
                      f'(顶点不足: {len(fa)}/{len(fb)})')
                continue
            inf = fa[:, 0].max()
            inm = fb[:, 0].min()
            print(f'   {z0*1000:+5.0f}~{z1*1000:+5.0f}   '
                  f'[{fa[:,0].min()*1000:+6.1f},{fa[:,0].max()*1000:+6.1f}]  '
                  f'[{fb[:,0].min()*1000:+6.1f},{fb[:,0].max()*1000:+6.1f}]  '
                  f'{(inf-inm)*1000:6.1f}')


if __name__ == '__main__':
    main()
