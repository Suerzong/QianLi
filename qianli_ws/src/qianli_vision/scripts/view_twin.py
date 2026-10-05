#!/usr/bin/env python3
"""数字孪生实时窗口（跑在 Ubuntu 桌面）

用**真网格凸分解**后的爪子（不是被凸包糊住的版本），
可以直观对比"仿真里的爪子"和"你手上真实的爪子"。

窗口操作（MuJoCo 自带）：
  左键拖动=旋转   右键拖动=平移   滚轮=缩放   空格=暂停   Tab=面板

用法：
  DISPLAY=:0 XAUTHORITY=<...> MUJOCO_GL=glfw ~/mj/bin/python view_twin.py
  ~/mj/bin/python view_twin.py --dz -0.030   # 指定下压高度
  ~/mj/bin/python view_twin.py --hold        # 只停在抓取姿态（便于细看）
"""

import argparse
import math
import os
import sys
import time

import numpy as np

import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_grasp as S
import sim_mesh_gripper as MG


def reset_object(model, data):
    qadr, aadr, cube, cq = S.attach_handles(model)
    op = S.obj_world_pos()
    data.qpos[cq:cq + 3] = op
    data.qpos[cq + 3:cq + 7] = [1, 0, 0, 0]
    return cq, op


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--obj-size', type=float, default=0.020)
    ap.add_argument('--dz', type=float, default=0.0)
    ap.add_argument('--hold', action='store_true')
    ap.add_argument('--loop', action='store_true', default=True)
    a = ap.parse_args()

    print(f'构建孪生（{a.obj_size*1000:.0f}mm 物块，真网格凸分解爪子）...',
          flush=True)
    model = MG.build(a.obj_size)
    data = mujoco.MjData(model)
    qadr, aadr, cube, cq = S.attach_handles(model)
    ch, names = S.make_chain()
    print(f'模型 geom 数 = {model.ngeom}', flush=True)

    viewer = None
    try:
        import mujoco.viewer as mv
        viewer = mv.launch_passive(model, data)
        print('窗口已打开 ✅', flush=True)
    except Exception as e:
        print(f'窗口打开失败: {e}', flush=True)
        print('（无显示环境时只能用离屏渲染）', flush=True)
        return

    op = S.obj_world_pos()
    z0 = S.TABLE_Z + 0.003 + a.obj_size / 2
    seed = np.zeros(len(ch.links))

    def go(dz, steps):
        sol, _ = S.solve_ik(ch, names, [op[0], op[1], z0 + dz], -90.0, seed)
        for j in S.ARM_JOINTS:
            data.ctrl[aadr[j]] = sol[names.index(j)]
        for _ in range(steps):
            if viewer.is_running():
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(model.opt.timestep)

    # 起始：张开夹爪
    reset_object(model, data)
    mujoco.mj_forward(model, data)
    data.ctrl[aadr['gripper']] = 1.2

    if a.hold:
        print('停在抓取姿态（可以拖动鼠标细看爪子与物块的关系）', flush=True)
        go(a.dz, 900)
        data.ctrl[aadr['gripper']] = 1.2
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)
        return

    print('循环演示：预抓取 → 下压 → 闭合 → 抬起 → 复位', flush=True)
    while viewer.is_running():
        reset_object(model, data)
        mujoco.mj_forward(model, data)
        data.ctrl[aadr['gripper']] = 1.2
        go(a.dz + 0.06, 500)          # 预抓取
        go(a.dz, 700)                 # 下压
        for k in range(60):           # 闭合
            data.ctrl[aadr['gripper']] = 1.2 * (1 - k / 59.0)
            for _ in range(8):
                if not viewer.is_running():
                    return
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(model.opt.timestep)
        go(a.dz + 0.10, 700)          # 抬起
        oz = data.xpos[cube][2]
        print(f'  物块 z = {oz:+.4f} (起始 {op[2]:+.4f}) '
              f'{"✅ 抓起" if oz > op[2] + 0.005 else "❌ 没抓起"}', flush=True)
        for _ in range(400):          # 停一下再循环
            if not viewer.is_running():
                return
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == '__main__':
    main()
