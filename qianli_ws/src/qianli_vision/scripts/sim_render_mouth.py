#!/usr/bin/env python3
"""渲染：抓取姿态下夹爪与物块的相对位置（侧视 + 俯视）

在孪生里把机械臂摆到抓取位姿，渲染两张图，直接看清爪口与物块的关系。
"""

import math
import sys

import numpy as np

import mujoco

sys.argv = [sys.argv[0]]
import sim_grasp as S   # noqa: E402


def render(model, data, path, lookat, dist, azim, elev, h=600, w=800):
    model.vis.global_.offwidth = max(w, model.vis.global_.offwidth)
    model.vis.global_.offheight = max(h, model.vis.global_.offheight)
    r = mujoco.Renderer(model, h, w)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = lookat
    cam.distance = dist
    cam.azimuth = azim
    cam.elevation = elev
    r.update_scene(data, cam)
    img = r.render()
    import cv2
    cv2.imwrite(path, img[:, :, ::-1])
    print(f'  {path}')


def main():
    size = float(sys.argv[1]) if len(sys.argv) > 1 else 0.014
    roll = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    S._OBJ_SIZE_OVERRIDE[0] = size
    model = S.build_model()
    data = mujoco.MjData(model)
    ch, names = S.make_chain()
    qadr, aadr, cube, cq = S.attach_handles(model)
    op = S.obj_world_pos()
    z = S.TABLE_Z + 0.003 + size / 2

    mujoco.mj_resetData(model, data)
    data.qpos[cq:cq + 3] = op
    data.qpos[cq + 3:cq + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    seed = np.zeros(len(ch.links))
    s1, _ = S.solve_ik(ch, names, [op[0], op[1], z + 0.06], -90.0, seed)
    s2, _ = S.solve_ik(ch, names, [op[0], op[1], z], -90.0, s1)
    s2[names.index('wrist_roll')] = roll
    for j in S.ARM_JOINTS:
        data.ctrl[aadr[j]] = s2[names.index(j)]
    data.ctrl[aadr['gripper']] = 1.2
    for _ in range(1500):
        mujoco.mj_step(model, data)
    print(f'物块 {size*1000:.0f}mm, roll={roll:+.2f}')
    gl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper_link')
    p = data.xpos[gl] + data.xmat[gl].reshape(3, 3) @ S.FRAME_IN_GRIPPER
    print(f'TCP = ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})  '
          f'物块 = ({op[0]:.4f}, {op[1]:.4f}, {op[2]:.4f})')

    model.vis.headlight.ambient[:] = [0.5, 0.5, 0.5]
    model.vis.headlight.diffuse[:] = [0.8, 0.8, 0.8]
    mid = [(p[0] + op[0]) / 2, (p[1] + op[1]) / 2, op[2] + 0.01]
    # 侧视（从 +Y 方向看，能看清爪口开合方向 = 世界 -Y）
    render(model, data, '/tmp/mouth_side.png', mid, 0.13, 90, -12)
    # 俯视（看爪口是否套住物块）
    render(model, data, '/tmp/mouth_top.png', mid, 0.13, 180, -70)
    # 另一个侧视（从 +X 看）
    render(model, data, '/tmp/mouth_side2.png', mid, 0.13, 0, -12)


if __name__ == '__main__':
    main()
