#!/usr/bin/env python3
"""孪生里训练抓取策略：扫描 wrist_roll（爪口朝向）等参数，找成功率最高的组合

为什么需要训练：
  ik_node 的 orientation_mode='Z' 只保证工具轴朝下，**爪口开合方向（滚转）
  是自由的**。若物块不在爪口平面内，夹爪会从旁边擦过去 —— 真机上就是
  "夹空"。孪生里可以安全地把这个角度扫一遍，找出正确值。

输出：每个 wrist_roll 下的 是否夹住 / 物块位移 / 夹爪-物块接触数

用法：
  ~/mj/bin/python sim_train.py --scan-roll
  ~/mj/bin/python sim_train.py --scan-roll --obj-size 0.014
"""

import argparse
import math
import sys

import numpy as np

import mujoco

sys.argv = [sys.argv[0]]
import sim_grasp as S   # noqa: E402


def run_once(model, data, ch, names, roll, obj_size):
    """跑一次抓取；h 覆盖 wrist_roll。返回 (是否夹住, 物块位移mm, 接触数)。"""
    qadr, aadr, cube, cq = S.attach_handles(model)
    op = S.obj_world_pos()
    z = S.TABLE_Z + 0.003 + obj_size / 2
    gl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper_link')
    cg = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    arm_ids = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n) for n in
               ['shoulder_link', 'upper_arm_link', 'lower_arm_link',
                'wrist_link', 'gripper_link', 'moving_jaw_so101_v1_link']}

    def tcp():
        return data.xpos[gl] + data.xmat[gl].reshape(3, 3) @ S.FRAME_IN_GRIPPER

    def arm_contacts():
        n = 0
        for c in range(data.ncon):
            cn = data.contact[c]
            if cg in (cn.geom1, cn.geom2):
                o = cn.geom2 if cn.geom1 == cg else cn.geom1
                if model.geom_bodyid[o] in arm_ids:
                    n += 1
        return n

    mujoco.mj_resetData(model, data)
    data.qpos[cq:cq + 3] = op
    data.qpos[cq + 3:cq + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)

    seed = np.zeros(len(ch.links))
    s1, _ = S.solve_ik(ch, names, [op[0], op[1], z + 0.06], -90.0, seed)
    s2, _ = S.solve_ik(ch, names, [op[0], op[1], z], -90.0, s1)
    s3, _ = S.solve_ik(ch, names, [op[0], op[1], z + 0.10], -90.0, s2)
    # 覆盖 wrist_roll
    for s in (s1, s2, s3):
        s[names.index('wrist_roll')] = roll

    for j in S.ARM_JOINTS:
        data.ctrl[aadr[j]] = s1[names.index(j)]
    data.ctrl[aadr['gripper']] = 1.2
    for _ in range(900):
        mujoco.mj_step(model, data)
    for j in S.ARM_JOINTS:
        data.ctrl[aadr[j]] = s2[names.index(j)]
    for _ in range(1200):
        mujoco.mj_step(model, data)
    nc_before = arm_contacts()
    for k in range(80):
        data.ctrl[aadr['gripper']] = 1.2 * (1 - k / 79.0)
        for _ in range(15):
            mujoco.mj_step(model, data)
    nc_close = arm_contacts()
    for j in S.ARM_JOINTS:
        data.ctrl[aadr[j]] = s3[names.index(j)]
    for _ in range(1500):
        mujoco.mj_step(model, data)
    oz = data.xpos[cube][2]
    moved = np.linalg.norm(data.xpos[cube] - op) * 1000
    return oz > op[2] + 0.005, moved, max(nc_before, nc_close)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--obj-size', type=float, default=0.014)
    ap.add_argument('--scan-roll', action='store_true')
    ap.add_argument('--steps', type=int, default=16)
    a = ap.parse_args()
    S._OBJ_SIZE_OVERRIDE[0] = a.obj_size

    model = S.build_model()
    data = mujoco.MjData(model)
    ch, names = S.make_chain()
    op = S.obj_world_pos()
    print(f'物块 {a.obj_size*1000:.0f}mm @ ({op[0]:.4f}, {op[1]:.4f}, '
          f'{op[2]:.4f})')
    print('\n=== 扫描 wrist_roll（爪口朝向） ===')
    print('  wrist_roll   是否夹住   物块位移   爪-物块接触')
    best = None
    for k in range(a.steps):
        roll = -math.pi + 2 * math.pi * k / a.steps
        ok, moved, nc = run_once(model, data, ch, names, roll,
                                 a.obj_size)
        flag = '✅ 夹住' if ok else '❌ 没夹住'
        print(f'  {roll:+.3f}      {flag}     {moved:6.1f}mm    {nc:2d}',
              flush=True)
        if ok and (best is None or abs(roll) < abs(best[0])):
            best = (roll, moved, nc)
    if best:
        print(f'\n🎉 找到可用爪口朝向: wrist_roll = {best[0]:+.3f} rad '
              f'({math.degrees(best[0]):+.1f}°)')
    else:
        print('\n本次扫描没有成功组合 —— 需要继续调其他参数'
              '（抓取高度/位置偏移/夹爪力）')


if __name__ == '__main__':
    main()
