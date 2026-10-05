#!/usr/bin/env python3
"""正确的抓取流程（终于可用）：全程只发 ctrl，绝不瞬移 qpos

本会话踩过的坑（全部已在代码里规避）：
  1. MuJoCo 对 MESH 碰撞用凸包 → 凹爪子被填平，必须用 CoACD 凸分解
  2. compile 后改 model.geom_pos / MjSpec 里改 gainprm 都不生效
  3. 脚本开头 sys.argv=[argv[0]] 会清掉 main() 的参数
  4. **IK 直接改 data.qpos（瞬移）会绕过约束求解器，把已夹住的物块挤掉** ← 真凶
     → 正确做法：IK 只在临时 MjData 里求解，主仿真只设 ctrl 让伺服走
  5. 夹爪闭合斜坡跑完时伺服还滞后 → 必须加 settle 等待合拢
  6. 接触过滤要用 geom id（不是 body id）；contact.frame 要 reshape(3,3)

抓取参数（本会话在孪生里搜出来的）：
  TCP 目标 = 物块中心 + (8, -4, 0) mm
  接近角 ≈ 0.6（此时两指尖接近同高），闭合到 0

用法：
  ~/mj/bin/python sim_grasp_ok.py --obj-size 0.020
  ~/mj/bin/python sim_grasp_ok.py --obj-size 0.020 --sweep
"""

import argparse
import sys

import numpy as np

import mujoco

import sim_grasp as S
import sim_mesh_gripper as MG
import sim_ik_dls as IK


def solve_only(model, qadr, target, cur_qpos):
    """在临时 MjData 里解 IK，**不改动主仿真状态**。"""
    tmp = mujoco.MjData(model)
    tmp.qpos[:] = cur_qpos
    q, err = IK.ik_dls(model, tmp, qadr, np.asarray(target))
    return q.tolist(), err


def goto(model, data, qadr, aadr, target, steps=500, grip=0.0,
         tol=0.002, max_rounds=6):
    """只发 ctrl 让伺服把 TCP 开到 target（不瞬移）。"""
    gl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper_link')
    for r in range(max_rounds):
        q_sol, _ = solve_only(model, qadr, target, data.qpos.copy())
        for j in S.ARM_JOINTS:
            data.ctrl[aadr[j]] = q_sol[S.ARM_JOINTS.index(j)]
        data.ctrl[aadr['gripper']] = grip
        for _ in range(steps):
            mujoco.mj_step(model, data)
        p = data.xpos[gl] + data.xmat[gl].reshape(3, 3) @ S.FRAME_IN_GRIPPER
        e = np.linalg.norm(p - np.asarray(target))
        if e < tol:
            return e
    return e


def grasp(model, data, qadr, aadr, off_mm, obj_size, approach=0.6,
          verbose=False):
    """完整抓取：预抓取 → 下压 → 夹紧(含 settle) → 抬起。返回 (成功, 详情)。"""
    op = S.obj_world_pos()
    cb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'object')
    cg = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    cq = S.attach_handles(model)[3]
    off = np.asarray(off_mm) / 1000.0
    tcp = op + off

    # 复位
    mujoco.mj_resetData(model, data)
    data.qpos[cq:cq + 3] = op
    data.qpos[cq + 3:cq + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)

    e1 = goto(model, data, qadr, aadr, tcp + np.array([0, 0, 0.06]),
              steps=500, grip=approach)
    e2 = goto(model, data, qadr, aadr, tcp, steps=500, grip=approach)
    # 夹紧 + settle
    data.ctrl[aadr['gripper']] = 0.0
    for _ in range(800):
        mujoco.mj_step(model, data)
    ang = data.qpos[qadr['gripper']]
    nc = sum(1 for c in range(data.ncon)
             if cg in (data.contact[c].geom1, data.contact[c].geom2))
    pos_drift = np.linalg.norm(data.xpos[cb] - op) * 1000
    # 抬起
    e3 = goto(model, data, qadr, aadr, tcp + np.array([0, 0, 0.12]),
              steps=600, grip=0.0)
    up = (data.xpos[cb][2] - op[2]) * 1000
    info = dict(ik=(e1, e2, e3), ang=ang, contacts=nc, drift=pos_drift,
                up=up)
    if verbose:
        print(f'    IK残差 预抓取{e1*1000:.1f}mm 下压{e2*1000:.1f}mm '
              f'抬起{e3*1000:.1f}mm')
        print(f'    夹紧后 角={ang:+.3f} 接触={nc} 物块漂移={pos_drift:.1f}mm')
    return up > 5.0, info


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--obj-size', type=float, default=0.020)
    ap.add_argument('--offset', nargs=3, type=float, default=[8.0, -4.0, 0.0])
    ap.add_argument('--approach', type=float, default=0.6)
    ap.add_argument('--sweep', action='store_true')
    ap.add_argument('--repeat', type=int, default=1)
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

    print(f'物块 {a.obj_size*1000:.0f}mm，偏移 {a.offset}，接近角 {a.approach}')
    if a.sweep:
        print('  偏移(mm)           结果      物块升高  接触  漂移')
        ok_list = []
        for dx in (4, 6, 8, 10, 12):
            for dy in (-8, -4, 0, 4):
                ok, info = grasp(model, data, qadr, aadr, [dx, dy, 0.0],
                                 a.obj_size, a.approach)
                print(f'  ({dx:+3d},{dy:+3d},+0)   '
                      f'{"✅ 抓起" if ok else "❌ 没抓起"}  '
                      f'{info["up"]:+8.1f}  {info["contacts"]:3d}  '
                      f'{info["drift"]:5.1f}mm', flush=True)
                if ok:
                    ok_list.append((dx, dy))
        print(f'\n成功 {len(ok_list)}/20 组: {ok_list}')
        return

    for i in range(a.repeat):
        ok, info = grasp(model, data, qadr, aadr, a.offset, a.obj_size,
                         a.approach, verbose=True)
        print(f'  第{i+1}次: {"🎉 抓起" if ok else "❌ 没抓起"}  '
              f'物块升高 {info["up"]:+.1f}mm', flush=True)


if __name__ == '__main__':
    main()
