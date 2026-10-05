#!/usr/bin/env python3
"""经验扫描"物块该放在离固定爪面多远"，找出活动爪能真正夹住的位置

判据（比几何推算可靠）：
  · 活动爪**被物块挡住** → 闭合终角明显大于"空合"角（空合约 0.02）
  · 且抬起后物块被带走

关键修正：摆好位后必须把各臂关节 ctrl 设成当前角度，
否则伺服把臂拽回原位、爪子横扫把物块打飞（之前"物块飞出去"的真凶）。

用法：
  ~/mj/bin/python sim_sweep_place.py --obj-size 0.020
"""

import argparse
import sys

import numpy as np

import mujoco

import sim_grasp as S
import sim_fixed_jaw as FJ
import sim_mesh_gripper as MG
import sim_ik_dls as IK


def trial(model, qadr, aadr, parts, obj_size, place_mm, approach_angle,
          close_steps=120, lift=True, clearance=1.0):
    """把物块放到离固定爪面 place_mm 处，闭合，看终角与是否抬起。"""
    data = mujoco.MjData(model)
    op = S.obj_world_pos()
    cb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'object')
    cq = S.attach_handles(model)[3]

    # 第一步：把固定爪面对准"名义物块位置 + clearance"
    tgt, info = FJ.compute_target_iter(model, data, qadr, parts, obj_size,
                                       clearance)
    # 第二步：沿"缝方向"平移，使物块落在离固定爪面 place_mm 处
    #   （u 指向活动爪一侧；物块中心离固定爪面 = place_mm）
    u = info['u']
    v_fix = info['v_fix']
    want = v_fix + u * (place_mm / 1000.0)
    tgt = tgt + (op - want)
    IK.ik_dls(model, data, qadr, tgt)
    data.qpos[qadr['gripper']] = approach_angle
    data.qpos[cq:cq + 3] = op
    data.qpos[cq + 3:cq + 7] = [1, 0, 0, 0]
    for j in S.ARM_JOINTS:                        # ★ 关键：锁住臂
        data.ctrl[aadr[j]] = data.qpos[qadr[j]]
    data.ctrl[aadr['gripper']] = approach_angle
    mujoco.mj_forward(model, data)

    for k in range(close_steps + 1):
        data.ctrl[aadr['gripper']] = approach_angle * (1 - k / float(close_steps))
        for _ in range(8):
            mujoco.mj_step(model, data)
    ang = data.qpos[qadr['gripper']]
    dz_close = (data.xpos[cb][2] - op[2]) * 1000
    if not lift:
        return ang, dz_close, 0.0
    # 抬起（保持臂关节锁在 IK 解附近）
    for lift_k in range(60):
        t = tgt + np.array([0, 0, 0.10 * (lift_k + 1) / 60.0])
        IK.ik_dls(model, data, qadr, t)
        data.qpos[qadr['gripper']] = 0.0
        for j in S.ARM_JOINTS:
            data.ctrl[aadr[j]] = data.qpos[qadr[j]]
        data.ctrl[aadr['gripper']] = 0.0
        for _ in range(8):
            mujoco.mj_step(model, data)
    return ang, dz_close, (data.xpos[cb][2] - op[2]) * 1000


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--obj-size', type=float, default=0.020)
    ap.add_argument('--lo', type=float, default=6.0)
    ap.add_argument('--hi', type=float, default=18.0)
    ap.add_argument('--step', type=float, default=1.0)
    ap.add_argument('--approach-angle', type=float, default=0.6)
    a = ap.parse_args()

    model = MG.build(a.obj_size)
    qadr, aadr = {}, {}
    for i in range(model.njnt):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if n:
            qadr[n] = model.jnt_qposadr[i]
    for i in range(model.nu):
        aadr[mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                               i).replace('servo_', '')] = i
    parts = FJ.load_parts()
    print(f'物块 {a.obj_size*1000:.0f}mm，接近角 {a.approach_angle}')
    print('  物块离固定爪面(mm)  闭合终角  闭合中下沉(mm)  抬起后升高(mm)  判定')
    best = None
    p = a.lo
    while p <= a.hi + 1e-9:
        ang, dzc, dzu = trial(model, qadr, aadr, parts, a.obj_size, p,
                              a.approach_angle)
        ok = dzu > 5.0
        tag = '✅ 夹起' if ok else ('爪被挡住但没带走' if ang > 0.10 else '空合')
        print(f'  {p:8.1f}          {ang:+.3f}   {dzc:+8.1f}      '
              f'{dzu:+8.1f}     {tag}', flush=True)
        if ok and best is None:
            best = p
        p += a.step
    if best is not None:
        print(f'\n🎉 可用放置距离 = {best:.1f}mm（20mm 物块被夹起）')
    else:
        print('\n本范围没夹起，需要继续调（接近角/夹持力/缝方向）')


if __name__ == '__main__':
    main()
