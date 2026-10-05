#!/usr/bin/env python3
"""算"两指之间的缝在哪"，把 TCP 目标对准它 → 正确抓取

原理（全用引擎 + 网格顶点）：
  1. 把臂摆到某个 TCP 位置
  2. 取"固定侧"和"活动侧"两片指的网格顶点（世界坐标）
  3. 在指尖附近找最近的一对顶点 → 那一对的中点 = **夹缝中心**
  4. 需要的偏移 = 物块中心 - 夹缝中心
     → TCP 目标 = 当前 TCP + 偏移，物块就正好落在缝里

用法：
  ~/mj/bin/python sim_align_mouth.py --obj-size 0.020
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
    """读回凸分解零件：(顶点, body名) 列表。"""
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--obj-size', type=float, default=0.020)
    ap.add_argument('--angle', type=float, default=1.2,
                    help='对缝时用的夹爪角（张开度）')
    a = ap.parse_args()

    model = MG.build(a.obj_size)
    data = mujoco.MjData(model)
    qadr = {}
    for i in range(model.njnt):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if n:
            qadr[n] = model.jnt_qposadr[i]
    qadr_all = S.attach_handles(model)
    cq = qadr_all[3]
    op = S.obj_world_pos()
    gl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper_link')
    mjb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                            'moving_jaw_so101_v1_link')
    parts = load_parts()

    ch, names = S.make_chain()

    def vertices_world():
        """当前位姿下两片指的世界顶点。"""
        fix, mov = [], []
        for v, body, pos, quat in parts:
            bid = gl if body == 'gripper_link' else mjb
            p = data.xpos[bid]
            R = data.xmat[bid].reshape(3, 3)
            Rl = quat_to_R(quat)
            w = (v @ Rl.T + pos) @ R.T + p
            (fix if bid == gl else mov).append(w)
        return np.vstack(fix), np.vstack(mov)

    # 先摆到名义 TCP = 物块中心
    sol, _ = S.solve_ik(ch, names, list(op), -90.0, np.zeros(len(ch.links)))
    for j in S.ARM_JOINTS:
        data.qpos[qadr[j]] = sol[names.index(j)]
    data.qpos[qadr['gripper']] = a.angle
    mujoco.mj_forward(model, data)
    tcp0 = data.xpos[gl] + data.xmat[gl].reshape(3, 3) @ S.FRAME_IN_GRIPPER

    fix, mov = vertices_world()
    print(f'固定侧 z 范围 [{fix[:,2].min():+.4f}, {fix[:,2].max():+.4f}]')
    print(f'活动侧 z 范围 [{mov[:,2].min():+.4f}, {mov[:,2].max():+.4f}]')
    # 各取自己最低的 35mm（指尖区）
    f = fix[fix[:, 2] < fix[:, 2].min() + 0.035]
    m = mov[mov[:, 2] < mov[:, 2].min() + 0.035]
    print(f'指尖区域顶点: 固定侧 {len(f)}, 活动侧 {len(m)}')
    # 最近一对顶点（分批算，省内存）
    best = (1e9, None, None)
    for i in range(0, len(f), 256):
        blk = f[i:i + 256]
        d = np.linalg.norm(blk[:, None, :] - m[None, :, :], axis=2)
        k = np.unravel_index(np.argmin(d), d.shape)
        if d[k] < best[0]:
            best = (float(d[k]), blk[k[0]], m[k[1]])
    gap, vf, vm = best
    mouth = (vf + vm) / 2
    print(f'两指最窄处间距 = {gap*1000:.2f} mm')
    print(f'夹缝中心(世界) = ({mouth[0]:+.4f}, {mouth[1]:+.4f}, '
          f'{mouth[2]:+.4f})')
    print(f'当前 TCP       = ({tcp0[0]:+.4f}, {tcp0[1]:+.4f}, {tcp0[2]:+.4f})')
    offset = op - mouth
    print(f'需要的 TCP 偏移 = ({offset[0]*1000:+.1f}, {offset[1]*1000:+.1f}, '
          f'{offset[2]*1000:+.1f}) mm')
    tgt = tcp0 + offset
    print(f'修正后 TCP 目标 = ({tgt[0]:+.4f}, {tgt[1]:+.4f}, {tgt[2]:+.4f})')
    print(f'物块中心        = ({op[0]:+.4f}, {op[1]:+.4f}, {op[2]:+.4f})')
    np.save('/tmp/mouth_offset.npy', offset)
    print('\n偏移已存 /tmp/mouth_offset.npy')


if __name__ == '__main__':
    main()
