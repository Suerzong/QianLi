#!/usr/bin/env python3
"""按"固定爪贴住物块一面 → 活动爪夹紧"的策略抓取

与之前的区别：
  之前把 **TCP 参考点** 对准物块 → 物块落在夹缝外
  现在把 **固定爪的夹持面** 对准物块的一个面（间隔 = 物块半宽 + 余量），
  活动爪从另一侧合上夹紧 —— 这才是真实夹爪的用法

算法（引擎 + 网格顶点，不手搓变换）：
  1. 夹爪闭合状态下，在两片爪的指尖顶点里找最近的一对 (v_fix, v_mov)
     → 间距 = 闭合缝宽，方向 u = (v_mov - v_fix)/|...|
  2. 把物块中心摆到 v_fix + u * (半宽 + 余量) 处：固定爪面正好贴住物块一面
  3. 推出的 TCP 目标 = 当前 TCP + (物块中心 - 该点)
  4. 张开 → 降到目标 → 闭合 → 抬起

用法：
  ~/mj/bin/python sim_fixed_jaw.py --obj-size 0.020
  ~/mj/bin/python sim_fixed_jaw.py --obj-size 0.020 --clearance 2
"""

import argparse
import os
import sys

import numpy as np
import trimesh

import mujoco

import sim_grasp as S
import sim_mesh_gripper as MG

PARTS_DIR = os.path.expanduser('~/mj_parts')


def load_parts():
    out = []
    with open(os.path.join(PARTS_DIR, 'manifest.txt')) as fh:
        for line in fh:
            p, body, pos, quat = line.strip().split('|')
            v = np.asarray(trimesh.load(p, force='mesh').vertices)
            out.append((v, body, np.array([float(x) for x in pos.split(',')]),
                        np.array([float(x) for x in quat.split(',')])))
    return out


def quat_to_R(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def compute_target(model, data, qadr, parts, obj_size, clearance_mm,
                   angle=0.0):
    """返回 (TCP 目标, 调试信息)。angle = 用来测缝的夹爪角（默认闭合）。"""
    gl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper_link')
    mjb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                            'moving_jaw_so101_v1_link')
    op = S.obj_world_pos()
    ch, names = S.make_chain()
    sol, _ = S.solve_ik(ch, names, list(op), -90.0, np.zeros(len(ch.links)))
    for j in S.ARM_JOINTS:
        data.qpos[qadr[j]] = sol[names.index(j)]
    data.qpos[qadr['gripper']] = angle
    mujoco.mj_forward(model, data)
    tcp0 = data.xpos[gl] + data.xmat[gl].reshape(3, 3) @ S.FRAME_IN_GRIPPER

    fix, mov = [], []
    for v, body, pos, quat in parts:
        bid = gl if body == 'gripper_link' else mjb
        R = data.xmat[bid].reshape(3, 3)
        w = (v @ quat_to_R(quat).T + pos) @ R.T + data.xpos[bid]
        (fix if bid == gl else mov).append(w)
    fix = np.vstack(fix)
    mov = np.vstack(mov)
    # 各取自己最低 35mm（指尖区）
    f = fix[fix[:, 2] < fix[:, 2].min() + 0.035]
    m = mov[mov[:, 2] < mov[:, 2].min() + 0.035]
    best = (1e9, None, None)
    for i in range(0, len(f), 256):
        blk = f[i:i + 256]
        d = np.linalg.norm(blk[:, None, :] - m[None, :, :], axis=2)
        k = np.unravel_index(np.argmin(d), d.shape)
        if d[k] < best[0]:
            best = (float(d[k]), blk[k[0]], m[k[1]])
    gap, v_fix, v_mov = best
    u = (v_mov - v_fix) / (np.linalg.norm(v_mov - v_fix) + 1e-12)
    half = obj_size / 2.0
    # 固定爪面贴住物块一面（留 clearance 余量）
    want_cube = v_fix + u * (half + clearance_mm / 1000.0)
    delta = op - want_cube
    return tcp0 + delta, dict(gap=gap, v_fix=v_fix, u=u, tcp0=tcp0,
                              delta=delta, op=op)


def compute_target_iter(model, data, qadr, parts, obj_size, clearance_mm,
                        angle=0.6, iters=6, tol_mm=0.5, verbose=True):
    """闭环迭代：摆位 → 量"固定爪面"实际位置 → 修正目标，直到对准。

    必须迭代的原因：ikpy 对 12mm 量级的偏移解不准（实测差 12.4mm），
    直接开环算出的目标根本到不了位。
    """
    gl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper_link')
    mjb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                            'moving_jaw_so101_v1_link')
    op = S.obj_world_pos()
    ch, names = S.make_chain()
    half = obj_size / 2.0
    target = op.copy()
    u = None
    v_fix = None
    err = None
    for it in range(iters):
        # 用 DLS IK（ikpy 定位误差 4~50mm，根本贴不准）
        import sim_ik_dls
        _, ikerr = sim_ik_dls.ik_dls(model, data, qadr, target, yaw_deg=-90.0)
        data.qpos[qadr['gripper']] = angle
        mujoco.mj_forward(model, data)
        tcp = data.xpos[gl] + data.xmat[gl].reshape(3, 3) @ S.FRAME_IN_GRIPPER
        fix, mov = [], []
        for v, body, pos, quat in parts:
            bid = gl if body == 'gripper_link' else mjb
            R = data.xmat[bid].reshape(3, 3)
            w = (v @ quat_to_R(quat).T + pos) @ R.T + data.xpos[bid]
            (fix if bid == gl else mov).append(w)
        fix = np.vstack(fix)
        mov = np.vstack(mov)
        f = fix[fix[:, 2] < fix[:, 2].min() + 0.045]
        m = mov[mov[:, 2] < mov[:, 2].min() + 0.045]
        best = (1e9, None, None)
        for i in range(0, len(f), 256):
            blk = f[i:i + 256]
            dd = np.linalg.norm(blk[:, None, :] - m[None, :, :], axis=2)
            k = np.unravel_index(np.argmin(dd), dd.shape)
            if dd[k] < best[0]:
                best = (float(dd[k]), blk[k[0]], m[k[1]])
        gap, v_fix, v_mov = best
        u = (v_mov - v_fix) / (np.linalg.norm(v_mov - v_fix) + 1e-12)
        want = v_fix + u * (half + clearance_mm / 1000.0)
        err = op - want
        if verbose:
            print(f'  迭代{it+1}: IK到位误差 '
                  f'{np.linalg.norm(tcp - target)*1000:5.1f}mm, '
                  f'固定爪面-物块残差 {np.linalg.norm(err)*1000:5.1f}mm')
        if np.linalg.norm(err) < tol_mm / 1000.0:
            break
        target = target + err
    return target, dict(gap=gap, u=u, v_fix=v_fix, op=op, err=err)


def run(model, data, qadr, aadr, target, obj_size, close_steps=90,
        approach_angle=0.6):
    op = S.obj_world_pos()
    cube = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'object')
    ch, names = S.make_chain()
    seed = np.zeros(len(ch.links))

    def goto(p, steps, keep=None):
        """用 DLS IK 精确到位（ikpy 误差太大）。keep 用于保持夹爪角。"""
        import sim_ik_dls
        q, err = sim_ik_dls.ik_dls(model, data, qadr, np.asarray(p),
                                   yaw_deg=-90.0)
        for j in S.ARM_JOINTS:
            data.ctrl[aadr[j]] = data.qpos[qadr[j]]
        if keep is not None:
            data.ctrl[aadr['gripper']] = keep
        for _ in range(steps):
            mujoco.mj_step(model, data)
        return q, err

    # 关键：接近时用 approach_angle，让两指尖几乎同高（张开 1.2 时活动指高 40mm，
    # 闭合时会从上方扫下来先撞到物块顶面把它推走）
    data.ctrl[aadr['gripper']] = approach_angle
    for _ in range(400):
        mujoco.mj_step(model, data)
    goto([target[0], target[1], target[2] + 0.05], 600, keep=approach_angle)
    goto(target, 900, keep=approach_angle)                    # 降到目标
    for k in range(close_steps):
        data.ctrl[aadr['gripper']] = approach_angle * (
            1 - k / (close_steps - 1.0))
        for _ in range(10):
            mujoco.mj_step(model, data)
    gap_ang = data.qpos[qadr['gripper']]
    goto([target[0], target[1], target[2] + 0.10], 1000, keep=0.0)  # 抬起
    oz = data.xpos[cube][2]
    return oz, op[2], gap_ang


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--obj-size', type=float, default=0.020)
    ap.add_argument('--clearance', type=float, default=1.0, help='贴面余量 mm')
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
    parts = load_parts()
    print('闭环迭代对准固定爪面:')
    target, info = compute_target_iter(model, data, qadr, parts,
                                       a.obj_size, a.clearance)
    print(f'物块 {a.obj_size*1000:.0f}mm，贴面余量 {a.clearance}mm')
    print(f'  闭合缝宽        = {info["gap"]*1000:.2f} mm')
    print(f'  固定爪面(世界)  = {np.round(info["v_fix"], 4)}')
    print(f'  缝方向 u        = {np.round(info["u"], 3)}')
    print(f'  对准残差        = {np.linalg.norm(info["err"])*1000:.2f} mm')
    print(f'  最终 TCP 目标   = {np.round(target, 4)}')

    oz, oz0, ga = run(model, data, qadr, aadr, target, a.obj_size)
    print(f'\n抓取结果: 物块 z {oz0:+.4f} → {oz:+.4f}  夹爪终角 {ga:+.3f}  '
          f'{"🎉 抓起来了！" if oz > oz0 + 0.005 else "❌ 没抓起来"}')
    gl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper_link')
    lo = 1e9
    for i in range(model.ngeom):
        gn = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ''
        if not gn.startswith('pg'):
            continue
        p = data.geom_xpos[i]
        R = data.geom_xmat[i].reshape(3, 3)
        half = np.abs(R) @ model.geom_size[i][:3]
        lo = min(lo, p[2] - half[2])
    print(f'  爪子最低点 z = {lo:+.4f}（棋盘顶 {S.TABLE_Z+0.003:+.4f}，'
          f'{"✅ 没穿桌面" if lo > S.TABLE_Z + 0.003 - 0.001 else "⚠️ 穿桌面"}）')


if __name__ == '__main__':
    main()
