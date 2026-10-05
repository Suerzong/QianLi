#!/usr/bin/env python3
"""渲染数字孪生场景（多角度，便于直观检查）

产出：
  <prefix>_overview.png  整体俯视（类似真相机视角）
  <prefix>_side.png      侧视（看高度关系）
  <prefix>_closeup.png   夹爪接近物块特写

用法：
  ~/mj/bin/python render_twin.py --prefix /tmp/twin --height 0.05
  MUJOCO_GL=egl ~/mj/bin/python render_twin.py --prefix /tmp/twin
"""

import argparse
import math
import os
import sys

import numpy as np

import mujoco

sys.argv = [sys.argv[0]]
import sim_grasp as S   # noqa: E402  复用场景与 IK


def set_arm_above(model, data, ch, names, qadr, z_offset):
    """用 IK 把 TCP 摆到物块上方 z_offset 处（工具朝下）。"""
    op = S.obj_world_pos()
    target = [op[0], op[1], op[2] + z_offset]
    seed = np.zeros(len(ch.links))
    sol, err = S.solve_ik(ch, names, target, -90.0, seed)
    for j in S.ARM_JOINTS:
        if j in names:
            data.qpos[qadr[j]] = sol[names.index(j)]
    data.qpos[qadr['gripper']] = 1.2      # 张开
    mujoco.mj_forward(model, data)
    return err


def render(model, data, path, lookat, dist, azim, elev, h=600, w=800):
    # 离屏渲染缓冲默认 640x480，这里调大
    model.vis.global_.offwidth = max(w, model.vis.global_.offwidth)
    model.vis.global_.offheight = max(h, model.vis.global_.offheight)
    r = mujoco.Renderer(model, h, w)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = lookat
    cam.distance = dist
    cam.azimuth = azim
    cam.elevation = elev
    opt = mujoco.MjvOption()
    r.update_scene(data, cam, opt)
    img = r.render()
    import cv2
    cv2.imwrite(path, img[:, :, ::-1])
    print(f'  已渲染 {path}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--prefix', default='/tmp/twin')
    ap.add_argument('--height', type=float, default=0.05,
                    help='夹爪停在物块上方多高（米）')
    a = ap.parse_args()

    model = S.build_model()
    # 补一盏灯（默认 headlight 太暗）
    model.vis.headlight.ambient[:] = [0.45, 0.45, 0.45]
    model.vis.headlight.diffuse[:] = [0.7, 0.7, 0.7]
    data = mujoco.MjData(model)
    ch, names = S.make_chain()

    qadr = {}
    for i in range(model.njnt):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if n:
            qadr[n] = model.jnt_qposadr[i]

    err = set_arm_above(model, data, ch, names, qadr, a.height)
    op = S.obj_world_pos()
    print(f'物块位置 {np.round(op, 4)}；IK 误差 {err*1000:.1f}mm')

    # 让物块落到棋盘上（消除初始悬空）
    for _ in range(400):
        mujoco.mj_step(model, data)
    # 再把臂摆回去（物理步进会让它垂一点，这里保持运动学姿态）
    set_arm_above(model, data, ch, names, qadr, a.height)

    print('渲染中…')
    # 1) 整体俯视（真相机大致在上方偏后）
    render(model, data, f'{a.prefix}_overview.png',
           lookat=[0.22, 0.0, -0.03], dist=0.72, azim=118, elev=-42)
    # 2) 侧视（看高度关系：底座、桌面、物块）
    render(model, data, f'{a.prefix}_side.png',
           lookat=[0.22, 0.0, -0.02], dist=0.62, azim=250, elev=-14)
    # 3) 夹爪接近物块特写
    render(model, data, f'{a.prefix}_closeup.png',
           lookat=[op[0], op[1], op[2] + 0.01], dist=0.24, azim=140,
           elev=-32)


if __name__ == '__main__':
    main()
