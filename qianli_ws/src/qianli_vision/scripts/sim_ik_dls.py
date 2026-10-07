#!/usr/bin/env python3
"""用 MuJoCo 雅可比做精确 IK（阻尼最小二乘 DLS）

为什么需要：
  ikpy 定位精度差（实测偏差 4~50mm，还会发散），而抓取要求把爪面
  精确贴到物块面（±1mm）。真机 driver 用的也是 ikpy，所以这解释了
  之前真机上那些 30mm / 147mm 的 IK 失败。

做法：
  1. 先用 ikpy 解一个初始解（保证工具朝下的姿态大致正确）
  2. 再用引擎的雅可比做位置 DLS 迭代 → 位置精度到亚毫米
  3. 每次迭代都夹带"工具轴朝下"的次级目标，保持姿态

用法：
  from sim_ik_dls import ik_dls
  q, err = ik_dls(model, data, qadr, target_pos, yaw_deg=-90)
"""

import math
import sys
import xml.etree.ElementTree as ET

import numpy as np

import mujoco

import sim_grasp as S

_frame_rpy = [float(v) for v in ET.parse(S.URDF).getroot().find("./joint[@name='gripper_frame_joint']/origin").get('rpy').split()]
_rx,_ry,_rz = _frame_rpy
_tool_z_local = np.array([math.cos(_rz)*math.sin(_ry)*math.cos(_rx)+math.sin(_rz)*math.sin(_rx),
                          math.sin(_rz)*math.sin(_ry)*math.cos(_rx)-math.cos(_rz)*math.sin(_rx),
                          math.cos(_ry)*math.cos(_rx)])


def tool_angle_deg(model, data):
    _,rotation,_ = tcp_of(model,data)
    axis = rotation @ _tool_z_local
    return math.degrees(math.acos(float(np.clip(-axis[2],-1.,1.))))


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def tcp_of(model, data):
    gl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper_link')
    R = data.xmat[gl].reshape(3, 3)
    return data.xpos[gl] + R @ S.FRAME_IN_GRIPPER, R, gl


def solve_only(model, qadr, target, cur_qpos, yaw=-90.0):
    """Solve on scratch data without moving the physical simulation."""
    scratch = mujoco.MjData(model)
    scratch.qpos[:] = cur_qpos
    q, error = ik_dls(model, scratch, qadr, np.asarray(target), yaw_deg=yaw)
    if tool_angle_deg(model,scratch) > 5.:
        error = float('inf')
    return q.tolist(), error


def ik_dls(model, data, qadr, target_pos, yaw_deg=-90.0, iters=400,
           lam=2e-4, step=0.6, seed=None, rot_weight=0.08, verbose=False, multi_seed=True):
    """位置 DLS IK + 工具轴保持。

    返回 (关节字典, 位置误差米)。
    """
    ch, names = S.make_chain()
    if seed is None:
        sol, _ = S.solve_ik(ch, names, list(target_pos), yaw_deg,
                            np.zeros(len(ch.links)))
    else:
        sol = seed
    if multi_seed:
        current = np.zeros(len(ch.links))
        middle = current.copy()
        for j in S.ARM_JOINTS:
            current[names.index(j)] = data.qpos[qadr[j]]
            middle[names.index(j)] = np.mean(ch.links[names.index(j)].bounds)
        bent = np.zeros(len(ch.links))
        for j,value in zip(S.ARM_JOINTS,(0.,-1.,1.5,-.7,-2.2)):
            bent[names.index(j)] = value
        best = None
        for candidate in (sol,current,np.zeros(len(ch.links)),middle,bent):
            q,error = ik_dls(model,data,qadr,target_pos,yaw_deg,iters,lam,step,
                            candidate,rot_weight,verbose,multi_seed=False)
            angle = tool_angle_deg(model,data)
            score = error/.002+angle/5.
            if best is None or score < best[0]:
                best = (score,q.copy(),error)
            if error <= .002 and angle <= 5.:
                best = (score,q.copy(),error)
                break
        for j,value in zip(S.ARM_JOINTS,best[1]):
            data.qpos[qadr[j]] = value
        mujoco.mj_forward(model,data)
        return best[1],best[2]
    for j in S.ARM_JOINTS:
        data.qpos[qadr[j]] = sol[names.index(j)]
    mujoco.mj_forward(model, data)

    Rdes = rot_z(math.radians(yaw_deg)) @ rot_x(math.pi)   # 工具朝下姿态
    joints = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)
              for j in S.ARM_JOINTS]
    dofs = np.array([model.jnt_dofadr[j] for j in joints])
    qadrs = np.array([model.jnt_qposadr[j] for j in joints])
    jlo = np.array([model.jnt_range[j][0] for j in joints])
    jhi = np.array([model.jnt_range[j][1] for j in joints])

    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    for it in range(iters):
        mujoco.mj_forward(model, data)
        p, R, gl = tcp_of(model, data)
        e_pos = target_pos - p
        # The TCP fixed joint rotates the gripper frame by pi about Y.
        # Constrain its actual Z axis, not the parent gripper body's Z axis.
        axis = R @ _tool_z_local
        e_rot = Rdes[:,2]-axis
        if np.linalg.norm(e_pos) < 1e-4 and np.linalg.norm(e_rot) < 1e-3:
            break
        mujoco.mj_jac(model, data, jacp, jacr, p, gl)
        skew = np.array([[0.,-axis[2],axis[1]],
                         [axis[2],0.,-axis[0]],[-axis[1],axis[0],0.]])
        J = np.vstack([jacp[:, dofs], -skew @ jacr[:, dofs] * rot_weight])
        e = np.concatenate([e_pos, e_rot * rot_weight])
        A = J @ J.T + lam * np.eye(6)
        dq = J.T @ np.linalg.solve(A, e)
        # 限速 + 限位
        dq = np.clip(dq * step, -0.08, 0.08)
        q = data.qpos[qadrs] + dq
        data.qpos[qadrs] = np.clip(q, jlo, jhi)
    mujoco.mj_forward(model, data)
    p, _, _ = tcp_of(model, data)
    return data.qpos[qadrs].copy(), float(np.linalg.norm(target_pos - p))


def main():
    """自测：随机目标点，测 DLS IK 的定位精度。"""
    import sim_mesh_gripper as MG
    model = MG.build(0.020)
    data = mujoco.MjData(model)
    qadr = {}
    for i in range(model.njnt):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if n:
            qadr[n] = model.jnt_qposadr[i]
    op = S.obj_world_pos()
    print('DLS IK 定位精度自测（目标 = 物块附近随机偏移）:')
    print('  目标偏移(mm)              IK 残差(mm)')
    for off in [(0, 0, 0), (0.012, -0.010, 0), (0.02, 0.015, -0.01),
                (-0.015, 0.02, 0.005), (0.03, 0.0, -0.02)]:
        tgt = op + np.array(off)
        q, err = ik_dls(model, data, qadr, tgt)
        print(f'  ({off[0]*1000:+6.1f},{off[1]*1000:+6.1f},{off[2]*1000:+6.1f})'
              f'        {err*1000:8.3f}')


if __name__ == '__main__':
    main()
