#!/usr/bin/env python3
"""仿真抓取训练：在数字孪生里试抓，找出能成功的抓取参数（不碰真机）

流程（全程物理仿真）：
  1. 场景 = 真机 URDF + 桌面 + 底座 + 棋盘 + 物块（几何全部实测标定）
  2. 给 5 个臂关节 + 夹爪加"位置伺服"执行器（模拟 STS3215）
  3. 用 ikpy（与 ik_node 同一库）解 IK：让 TCP 到达 (物块xy, 抓取高度)，工具朝下
  4. 闭合夹爪 → 抬起 → 看物块是否被带起来（真的物理接触+摩擦）

用法：
  ~/mj/bin/python sim_grasp.py --obj-size 0.012
  ~/mj/bin/python sim_grasp.py --scan-size      # 扫描物块尺寸
"""

import argparse
import math
import os

import numpy as np

import mujoco
from ikpy.chain import Chain

URDF = os.path.expanduser(
    '~/legacy/arm/arm-final/ros2_ws/install/so101_bringup/share/so101_bringup'
    '/urdf/so101.urdf')

TABLE_Z = -0.0524
BASE_BOTTOM = -0.0024
BOARD_ORIGIN = (0.3420, 0.0584)
BOARD_YAW = math.radians(-97.75)
BOARD_W, BOARD_H = 0.228, 0.162
OBJ_SIZE = 0.02
OBJ_GRID = (0.111, 0.0)
# 运行时覆盖物块尺寸（孪生里扫不同尺寸，找爪口能容纳的上限）
_OBJ_SIZE_OVERRIDE = [None]


def obj_size():
    return _OBJ_SIZE_OVERRIDE[0] or OBJ_SIZE


ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
              'wrist_roll']
ALL_JOINTS = ARM_JOINTS + ['gripper']
FRAME_IN_GRIPPER = np.array([-0.0079, -0.000218121, -0.0981274])


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def obj_world_pos():
    cy, sy = math.cos(BOARD_YAW), math.sin(BOARD_YAW)
    ox = BOARD_ORIGIN[0] + cy * OBJ_GRID[0] - sy * OBJ_GRID[1]
    oy = BOARD_ORIGIN[1] + sy * OBJ_GRID[0] + cy * OBJ_GRID[1]
    return np.array([ox, oy, TABLE_Z + 0.003 + obj_size() / 2])


def build_model():
    """搭场景 + 给关节加位置伺服执行器。"""
    spec = mujoco.MjSpec.from_file(URDF)
    wb = spec.worldbody

    gt = wb.add_geom()
    gt.name = 'table'
    gt.type = mujoco.mjtGeom.mjGEOM_BOX
    gt.size = [0.4, 0.4, 0.15]
    gt.pos = [0.3, 0.0, TABLE_Z - 0.15]
    gt.rgba = [0.55, 0.55, 0.58, 1]

    gp = wb.add_geom()
    gp.name = 'pedestal'
    gp.type = mujoco.mjtGeom.mjGEOM_BOX
    gp.size = [0.045, 0.05, (BASE_BOTTOM - TABLE_Z) / 2]
    gp.pos = [0.0, 0.0, (TABLE_Z + BASE_BOTTOM) / 2]
    gp.rgba = [0.3, 0.3, 0.32, 1]

    cy, sy = math.cos(BOARD_YAW), math.sin(BOARD_YAW)
    cx = BOARD_ORIGIN[0] + cy * (BOARD_W / 2) - sy * (BOARD_H / 2)
    cyy = BOARD_ORIGIN[1] + sy * (BOARD_W / 2) + cy * (BOARD_H / 2)
    gb = wb.add_geom()
    gb.name = 'board'
    gb.type = mujoco.mjtGeom.mjGEOM_BOX
    gb.size = [BOARD_W / 2, BOARD_H / 2, 0.0015]
    gb.pos = [cx, cyy, TABLE_Z + 0.0015]
    gb.quat = [math.cos(BOARD_YAW / 2), 0, 0, math.sin(BOARD_YAW / 2)]
    gb.rgba = [0.9, 0.9, 0.9, 1]

    op = obj_world_pos()
    ob = wb.add_body(name='object')
    ob.pos = list(op)
    ob.add_freejoint()
    go = ob.add_geom()
    go.name = 'cube'
    go.type = mujoco.mjtGeom.mjGEOM_BOX
    go.size = [obj_size() / 2] * 3
    go.rgba = [0.75, 0.75, 0.78, 1]
    go.mass = 0.008

    servo = {'shoulder_pan': 120, 'shoulder_lift': 120, 'elbow_flex': 120,
             'wrist_flex': 60, 'wrist_roll': 30, 'gripper': 8}
    for j in ALL_JOINTS:
        act = spec.add_actuator()
        act.name = f'servo_{j}'
        act.target = j
        act.trntype = mujoco.mjtTrn.mjTRN_JOINT
        act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
        act.biastype = mujoco.mjtBias.mjBIAS_AFFINE
        act.gainprm[0] = 1.0          # 真正的增益在 compile 后设置
        act.biasprm[1] = 0.0
        act.biasprm[2] = 0.0
        lo, hi = (-0.1745, 1.7453) if j == 'gripper' else (-3.2, 3.2)
        act.ctrlrange = [lo, hi]
        act.forcerange = [-2.0, 2.0] if j == 'gripper' else [-12.0, 12.0]

    model = spec.compile()
    apply_collision_groups(model)
    # 关键：位置伺服增益必须在 **compile 之后** 的 model 上设置。
    # 在 MjSpec 里改 act.gainprm 不会生效（实测关节完全不跟随）。
    for i in range(model.nu):
        nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        j = nm.replace('servo_', '')
        kp = servo.get(j, 40.0)
        model.actuator_gainprm[i][0] = kp
        model.actuator_biasprm[i][1] = -kp
        model.actuator_biasprm[i][2] = -2.0 * math.sqrt(kp) * 0.2
    # 关键：加转子惯量（armature）。
    # URDF 的连杆转动惯量只有 ~1e-4 kg·m²，配 kp=120 时自然频率
    # ω=√(kp/I)≈1000 rad/s，远超 2ms 步长的奈奎斯特频率 → 数值发散
    # （实测 wrist_roll 在 -2.8~+1.7 之间疯狂振荡）。
    # 真舵机有减速箱转子惯量，加上它就稳定了。
    for i in range(model.njnt):
        if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE:
            d = model.jnt_dofadr[i]
            nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            model.dof_armature[d] = (0.002 if nm == 'gripper' else 0.02)
    return model


def apply_collision_groups(model):
    """碰撞分组（关键！否则仿真里机械臂跟不上控制量）。

    机械臂 contype=1, conaffinity=0；物块 contype=2, conaffinity=1；
    桌子/棋盘 contype=4, conaffinity=2。接触判定 =
    (contype_a & conaffinity_b) | (contype_b & conaffinity_a)：
      臂-臂   (1&0)|(1&0)=0 不碰  ← 自碰撞会产生 75N·m 约束力压过伺服
      臂-桌   (1&2)|(4&0)=0 不碰  ← 网格穿透地板同样顶住伺服
      臂-物块 (1&1)|(2&0)=1 碰
      物块-桌 (2&2)|(4&1)=2 碰
    """
    arm_ids = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
               for n in ['shoulder_link', 'upper_arm_link', 'lower_arm_link',
                         'wrist_link', 'gripper_link',
                         'moving_jaw_so101_v1_link']}
    env = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n)
           for n in ('table', 'pedestal', 'board')}
    cube = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    world_body = 0
    for i in range(model.ngeom):
        if model.geom_bodyid[i] in arm_ids:
            model.geom_contype[i], model.geom_conaffinity[i] = 1, 0
        elif i in env:
            model.geom_contype[i], model.geom_conaffinity[i] = 4, 2
        elif i == cube:
            model.geom_contype[i], model.geom_conaffinity[i] = 2, 1
        elif model.geom_bodyid[i] == world_body:
            # MuJoCo 从 URDF 载入时会自动加一个地板平面(z=0)，正好穿过
            # 机械臂基座 → 12 个接触、巨大约束力，伺服完全跟不动。
            # 我们有自己的桌面，把其余无名 world geom 的碰撞全部关掉。
            model.geom_contype[i], model.geom_conaffinity[i] = 0, 0


def make_chain():
    ch = Chain.from_urdf_file(URDF, base_elements=['base_link'],
                              active_links_mask=None)
    names = [getattr(l, 'name', '') for l in ch.links]
    ch.active_links_mask = [n in ARM_JOINTS for n in names]
    return ch, names


def solve_ik(ch, names, target_xyz, yaw_deg, seed):
    """解 IK：TCP 到 target_xyz，工具轴朝下，给定偏航。返回 (关节向量, 误差)。"""
    R = rot_z(math.radians(yaw_deg)) @ rot_x(math.pi)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = target_xyz
    sol = ch.inverse_kinematics_frame(T, initial_position=seed,
                                      orientation_mode='Z')
    fk = ch.forward_kinematics(sol)
    err = np.linalg.norm(fk[:3, 3] - np.array(target_xyz))
    return sol, err


def attach_handles(model):
    qadr, aadr = {}, {}
    for i in range(model.njnt):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if n:
            qadr[n] = model.jnt_qposadr[i]
    for i in range(model.nu):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        aadr[n.replace('servo_', '')] = i
    cube = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'object')
    cube_q = None
    for i in range(model.njnt):
        if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE and \
                model.jnt_bodyid[i] == cube:
            cube_q = model.jnt_qposadr[i]
    return qadr, aadr, cube, cube_q


def try_grasp(model, data, ch, names, aadr, cube, cube_q):
    """跑一次 预抓取→下压→闭合→抬起，返回 (物块最终z, 初始z, 是否抬起)。"""
    op = obj_world_pos()
    z = TABLE_Z + 0.003 + obj_size() / 2      # TCP 对准物块中心
    mujoco.mj_resetData(model, data)
    data.qpos[cube_q:cube_q + 3] = op
    data.qpos[cube_q + 3:cube_q + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    seed = np.zeros(len(ch.links))
    s1, _ = solve_ik(ch, names, [op[0], op[1], z + 0.06], -90.0, seed)
    for j in ARM_JOINTS:
        data.ctrl[aadr[j]] = s1[names.index(j)]
    data.ctrl[aadr['gripper']] = 1.2
    for _ in range(700):
        mujoco.mj_step(model, data)
    s2, _ = solve_ik(ch, names, [op[0], op[1], z], -90.0, s1)
    for j in ARM_JOINTS:
        data.ctrl[aadr[j]] = s2[names.index(j)]
    for _ in range(900):
        mujoco.mj_step(model, data)
    for k in range(60):
        data.ctrl[aadr['gripper']] = 1.2 * (1 - k / 59.0)
        for _ in range(12):
            mujoco.mj_step(model, data)
    s3, _ = solve_ik(ch, names, [op[0], op[1], z + 0.10], -90.0, s2)
    for j in ARM_JOINTS:
        data.ctrl[aadr[j]] = s3[names.index(j)]
    for _ in range(1200):
        mujoco.mj_step(model, data)
    oz = data.xpos[cube][2]
    return oz, op[2], oz > op[2] + 0.005


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--obj-size', type=float, default=None)
    ap.add_argument('--scan-size', action='store_true')
    a = ap.parse_args()
    if a.obj_size:
        _OBJ_SIZE_OVERRIDE[0] = a.obj_size

    ch, names = make_chain()

    if a.scan_size:
        print('=== 扫描物块尺寸（爪口开度实测约 19.6mm） ===')
        print('  尺寸    抓取高度z   物块最终z   结果')
        for sz in [0.010, 0.012, 0.014, 0.016, 0.018, 0.020]:
            _OBJ_SIZE_OVERRIDE[0] = sz
            model = build_model()
            data = mujoco.MjData(model)
            qadr, aadr, cube, cube_q = attach_handles(model)
            oz, oz0, ok = try_grasp(model, data, ch, names, aadr, cube,
                                     cube_q)
            z = TABLE_Z + 0.003 + sz / 2
            print(f'  {sz*1000:4.0f}mm   {z:+.4f}    {oz:+.4f}    '
                  f'{"✅ 抓起" if ok else "❌ 没抓起"}', flush=True)
        return

    model = build_model()
    data = mujoco.MjData(model)
    qadr, aadr, cube, cube_q = attach_handles(model)
    op = obj_world_pos()
    print(f'物块 {obj_size()*1000:.0f}mm，位置 ({op[0]:.4f}, {op[1]:.4f}, '
          f'{op[2]:.4f})')
    oz, oz0, ok = try_grasp(model, data, ch, names, aadr, cube, cube_q)
    print(f'物块 z: {oz0:+.4f} → {oz:+.4f}  '
          f'{"✅ 抓起" if ok else "❌ 没抓起"}')


if __name__ == '__main__':
    main()
