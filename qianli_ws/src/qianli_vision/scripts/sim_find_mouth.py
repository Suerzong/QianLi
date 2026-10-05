#!/usr/bin/env python3
"""经验标定爪口中心：张着爪子下压，找"物块不被推动"的 TCP 偏移

原理：
  若夹爪正好套住物块（物块在爪口内），下压时物块几乎不动；
  若偏了，爪尖会撞到物块，把它推开。
  所以：扫描 TCP 偏移 → 找物块位移最小的那个 = 爪口中心。

输出：最优偏移 + 该偏移下物块在夹爪坐标系里的位置（作为以后的对齐基准）

用法：
  ~/mj/bin/python sim_find_mouth.py --obj-size 0.014
"""

import argparse
import math
import sys

import numpy as np

import mujoco

sys.argv = [sys.argv[0]]
import sim_grasp as S   # noqa: E402


def probe(model, data, ch, names, qadr, aadr, cube, cq, dx, dy, obj_size):
    """张爪下压到抓取高度，返回物块位移(mm) 与爪-物块接触数。"""
    op = S.obj_world_pos()
    z = S.TABLE_Z + 0.003 + obj_size / 2
    cg = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    arm_ids = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n) for n in
               ['shoulder_link', 'upper_arm_link', 'lower_arm_link',
                'wrist_link', 'gripper_link', 'moving_jaw_so101_v1_link']}

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
    tx, ty = op[0] + dx, op[1] + dy
    seed = np.zeros(len(ch.links))
    s1, _ = S.solve_ik(ch, names, [tx, ty, z + 0.05], -90.0, seed)
    for j in S.ARM_JOINTS:
        data.ctrl[aadr[j]] = s1[names.index(j)]
    data.ctrl[aadr['gripper']] = 1.2       # 张开
    for _ in range(600):
        mujoco.mj_step(model, data)
    s2, _ = S.solve_ik(ch, names, [tx, ty, z], -90.0, s1)
    for j in S.ARM_JOINTS:
        data.ctrl[aadr[j]] = s2[names.index(j)]
    for _ in range(1000):
        mujoco.mj_step(model, data)
    return (np.linalg.norm(data.xpos[cube] - op) * 1000, arm_contacts(),
            data.xpos[cube].copy())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--obj-size', type=float, default=0.014)
    ap.add_argument('--span', type=float, default=26.0)
    ap.add_argument('--step', type=float, default=6.5)
    a = ap.parse_args()
    S._OBJ_SIZE_OVERRIDE[0] = a.obj_size

    model = S.build_model()
    data = mujoco.MjData(model)
    ch, names = S.make_chain()
    qadr, aadr, cube, cq = S.attach_handles(model)
    op = S.obj_world_pos()
    print(f'物块 {a.obj_size*1000:.0f}mm @ ({op[0]:.4f}, {op[1]:.4f}, '
          f'{op[2]:.4f})')
    vals = [i * a.step for i in range(-int(a.span // a.step),
                                     int(a.span // a.step) + 1)]
    best = None
    print('\n dx(mm)  dy(mm)   物块位移   接触   判定')
    for dx in vals:
        for dy in vals:
            moved, nc, pos = probe(model, data, ch, names, qadr, aadr, cube,
                                   cq, dx / 1000.0, dy / 1000.0, a.obj_size)
            tag = '套住' if moved < 3.0 else ('轻碰' if moved < 10 else '撞开')
            print(f' {dx:+6.1f} {dy:+6.1f}   {moved:6.1f}mm  {nc:2d}   {tag}',
                  flush=True)
            if best is None or moved < best[0]:
                best = (moved, dx, dy, nc)
    print(f'\n最优偏移: dx={best[1]:+.1f}mm dy={best[2]:+.1f}mm '
          f'(物块仅移动 {best[0]:.1f}mm, 接触 {best[3]})')


if __name__ == '__main__':
    main()
