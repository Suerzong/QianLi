#!/usr/bin/env python3
"""用真网格的凸分解重建夹爪碰撞（不用手搓盒子）

原理：
  · MuJoCo 对 MESH 碰撞只能用**凸包** → 凹的爪子会被填平，爪口糊住
  · 解法：把每个爪子网格用 CoACD 拆成若干**凸块**，每块单独作为 geom
    → 拼起来就还原了真实的凹形（爪口是空的）

流程：
  1. --decompose : 把 3 个爪子网格做凸分解，零件存到 ~/mj_parts/
  2. --check     : 用分解后的零件建模型，量"口宽 vs 夹爪角"
  3. --scan      : 重跑物块尺寸扫描（看 20mm 能不能抓）

用法：
  ~/mj/bin/python sim_mesh_gripper.py --decompose
  ~/mj/bin/python sim_mesh_gripper.py --check
  ~/mj/bin/python sim_mesh_gripper.py --scan
"""

import argparse
import math
import os
import sys

import numpy as np

import mujoco

import sim_grasp as S

ASSETS = os.path.join(os.path.dirname(S.URDF), 'assets')
PARTS_DIR = os.path.expanduser('~/mj_parts')

# (网格文件, 挂在哪个 body, 局部平移, 局部四元数)
# 平移/四元数来自 URDF 里该 visual/collision 的 origin
MESH_SPECS = [
    ('wrist_roll_follower_so101_v1.stl', 'gripper_link',
     (8.32667e-17, -0.000218214, 0.000949706), (0.0, -1.0, 0.0, 0.0)),
    ('sts3215_03a_v1.stl', 'gripper_link',
     (0.0077, 0.0001, -0.0234), (0.70710678, -0.70710678, 0.0, 0.0)),
    ('moving_jaw_so101_v1.stl', 'moving_jaw_so101_v1_link',
     (-5.55112e-17, -5.55112e-17, 0.0189), (1.0, 0.0, 0.0, 0.0)),
]


def decompose():
    import coacd
    import trimesh
    os.makedirs(PARTS_DIR, exist_ok=True)
    manifest = []
    for fn, body, pos, quat in MESH_SPECS:
        src = os.path.join(ASSETS, fn)
        m = trimesh.load(src, force='mesh')
        cm = coacd.Mesh(np.asarray(m.vertices), np.asarray(m.faces))
        parts = coacd.run_coacd(cm, threshold=0.06, max_convex_hull=24)
        tag = fn.replace('.stl', '')
        print(f'{fn}: 顶点 {len(m.vertices)} → 拆成 {len(parts)} 块凸体',
              flush=True)
        for i, (v, f) in enumerate(parts):
            out = os.path.join(PARTS_DIR, f'{tag}_p{i:02d}.stl')
            trimesh.Trimesh(vertices=v, faces=f).export(out)
            manifest.append((out, body, pos, quat))
    with open(os.path.join(PARTS_DIR, 'manifest.txt'), 'w') as fh:
        for out, body, pos, quat in manifest:
            fh.write(f'{out}|{body}|{pos[0]},{pos[1]},{pos[2]}|'
                     f'{quat[0]},{quat[1]},{quat[2]},{quat[3]}\n')
    print(f'共 {len(manifest)} 块，已存到 {PARTS_DIR}，清单 manifest.txt')


def load_manifest():
    out = []
    with open(os.path.join(PARTS_DIR, 'manifest.txt')) as fh:
        for line in fh:
            p, body, pos, quat = line.strip().split('|')
            out.append((p, body, tuple(float(x) for x in pos.split(',')),
                        tuple(float(x) for x in quat.split(','))))
    return out


def make_spec(obj_size):
    S._OBJ_SIZE_OVERRIDE[0] = obj_size
    spec = mujoco.MjSpec.from_file(S.URDF)
    wb = spec.worldbody
    wb.add_light(name='grasp_key', pos=[0.2, -0.3, 0.7], dir=[0, 0, -1],
                 diffuse=[0.8, 0.8, 0.8])
    wb.add_light(name='grasp_fill', pos=[0.4, 0.5, 0.5], dir=[0, -1, -1],
                 diffuse=[0.5, 0.5, 0.5])
    gt = wb.add_geom(); gt.name = 'table'
    gt.type = mujoco.mjtGeom.mjGEOM_BOX
    gt.size = [0.4, 0.4, 0.15]; gt.pos = [0.3, 0.0, S.TABLE_Z - 0.15]
    gt.rgba = [0.35, 0.38, 0.43, 1]
    gp = wb.add_geom(); gp.name = 'pedestal'
    gp.type = mujoco.mjtGeom.mjGEOM_BOX
    gp.size = [0.045, 0.05, (S.BASE_BOTTOM - S.TABLE_Z) / 2]
    gp.pos = [0.0, 0.0, (S.TABLE_Z + S.BASE_BOTTOM) / 2]
    cy, sy = math.cos(S.BOARD_YAW), math.sin(S.BOARD_YAW)
    cx = S.BOARD_ORIGIN[0] + cy * (S.BOARD_W / 2) - sy * (S.BOARD_H / 2)
    cyy = S.BOARD_ORIGIN[1] + sy * (S.BOARD_W / 2) + cy * (S.BOARD_H / 2)
    gb = wb.add_geom(); gb.name = 'board'
    gb.type = mujoco.mjtGeom.mjGEOM_BOX
    gb.size = [S.BOARD_W / 2, S.BOARD_H / 2, 0.0015]
    gb.pos = [cx, cyy, S.TABLE_Z + 0.0015]
    gb.quat = [math.cos(S.BOARD_YAW / 2), 0, 0, math.sin(S.BOARD_YAW / 2)]
    gb.rgba = [0.9, 0.9, 0.85, 1]
    op = S.obj_world_pos()
    ob = wb.add_body(name='object'); ob.pos = list(op); ob.add_freejoint()
    go = ob.add_geom(); go.name = 'cube'
    go.type = mujoco.mjtGeom.mjGEOM_BOX
    go.size = [obj_size / 2] * 3
    go.mass = 0.008
    go.rgba = [0.05, 0.8, 0.7, 1]

    # 加凸分解零件
    for i, (path, body, pos, quat) in enumerate(load_manifest()):
        mn = f'part{i:03d}'
        mesh = spec.add_mesh()
        mesh.name = mn
        mesh.file = path
        g = spec.body(body).add_geom()
        g.name = f'pg{i:03d}'
        g.type = mujoco.mjtGeom.mjGEOM_MESH
        g.meshname = mn
        g.pos = list(pos)
        g.quat = list(quat)

    # Configure masks before compilation: MuJoCo also builds per-body masks
    # and collision BVHs, so editing only model.geom_contype is insufficient.
    # Groups: object=1, jaws=2, environment=4, proximal arm=8.
    for geom in spec.geoms:
        if geom.name.startswith('pg'):
            geom.contype, geom.conaffinity = 2, 5
        elif geom.name in ('table', 'board', 'pedestal'):
            geom.contype, geom.conaffinity = 4, 11
        elif geom.name == 'cube':
            geom.contype, geom.conaffinity = 1, 6
        elif geom.parent.name in ('shoulder_link', 'upper_arm_link',
                                  'lower_arm_link', 'wrist_link'):
            geom.contype, geom.conaffinity = 8, 4
        else:
            # Fixed base / duplicate gripper meshes are visual only.
            geom.contype, geom.conaffinity = 0, 0
    ranges = {j.name: list(j.range) for j in spec.joints if j.name}
    servo = {'shoulder_pan': 120, 'shoulder_lift': 120, 'elbow_flex': 120,
             'wrist_flex': 60, 'wrist_roll': 30, 'gripper': 20}
    for j in S.ALL_JOINTS:
        act = spec.add_actuator()
        act.name = f'servo_{j}'
        act.target = j
        act.trntype = mujoco.mjtTrn.mjTRN_JOINT
        act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
        act.biastype = mujoco.mjtBias.mjBIAS_AFFINE
        kp = servo[j]
        # Assign complete parameter arrays; indexed writes to MjSpec arrays
        # can edit a temporary copy in some versions of the Python binding.
        act.gainprm = [kp] + [0.0] * 9
        act.biasprm = [0.0, -kp, -0.4 * math.sqrt(kp)] + [0.0] * 7
        act.ctrlrange = ranges[j]
        act.ctrllimited = True
        # Conservative simulation limits; actual torque curves need measuring.
        torque = 1.5 if j == 'gripper' else 3.0
        act.forcerange = [-torque, torque]
        act.forcelimited = True
        spec.joint(j).armature = 0.002 if j == 'gripper' else 0.02
    return spec


def build(obj_size):
    return make_spec(obj_size).compile()


def pose_and_measure(model, angles):
    data = mujoco.MjData(model)
    qadr = {}
    for i in range(model.njnt):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if n:
            qadr[n] = model.jnt_qposadr[i]
    ch, names = S.make_chain()
    sol, _ = S.solve_ik(ch, names, [0.30, 0.0, 0.05], -90.0,
                        np.zeros(len(ch.links)))
    for j in S.ARM_JOINTS:
        data.qpos[qadr[j]] = sol[names.index(j)]
    gl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper_link')
    mj = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                           'moving_jaw_so101_v1_link')
    ga = [i for i in range(model.ngeom) if model.geom_bodyid[i] == gl
          and (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ''
               ).startswith('pg')]
    gb = [i for i in range(model.ngeom) if model.geom_bodyid[i] == mj
          and (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ''
               ).startswith('pg')]
    rows = []
    for g in angles:
        data.qpos[qadr['gripper']] = g
        mujoco.mj_forward(model, data)
        best = 1e9
        for a in ga:
            for b in gb:
                d = mujoco.mj_geomDistance(model, data, a, b, 1.0, None)
                best = min(best, d)
        rows.append((g, best))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--decompose', action='store_true')
    ap.add_argument('--check', action='store_true')
    ap.add_argument('--scan', action='store_true')
    a = ap.parse_args()

    if a.decompose:
        decompose()
        return

    if a.check or not a.scan:
        model = build(0.020)
        print('=== 真网格凸分解后的口宽 vs 夹爪角 ===')
        print('  夹爪角    口宽(mm)')
        for g, d in pose_and_measure(model, [1.745, 1.2, 0.8, 0.4, 0.0,
                                             -0.1745]):
            tag = '  ← 负值=穿透' if d < 0 else ''
            print(f'  {g:+.4f}   {d*1000:8.1f}{tag}')

    if a.scan:
        import sim_sweep_offset as SW
        ch, names = S.make_chain()
        print('=== 真网格爪子下重跑尺寸扫描（dz=-30mm）===')
        print('  尺寸    结果      接触')
        for sz in (0.012, 0.014, 0.016, 0.018, 0.020):
            m = build(sz)
            d = mujoco.MjData(m)
            ok, nc, moved, ga = SW.trial(m, d, ch, names, 0.0, 0.0, -0.030, sz)
            print(f'  {sz*1000:4.0f}mm  {"✅ 抓起" if ok else "❌ 没抓起"}  '
                  f'{nc:2d}', flush=True)


if __name__ == '__main__':
    main()
