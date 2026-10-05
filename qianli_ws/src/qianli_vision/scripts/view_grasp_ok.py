#!/usr/bin/env python3
"""实时窗口（可用版）：演示已经验证成功的抓取流程

与旧版区别：用**只发 ctrl、不瞬移**的正确流程（旧版瞬移会把物块挤掉），
所以窗口里能看到真正把 20mm 物块夹起来。

窗口操作：左键转视角 / 右键平移 / 滚轮缩放 / 空格暂停 / Tab 面板

用法（在 Ubuntu 桌面）：
  DISPLAY=:0 XAUTHORITY=<...> MUJOCO_GL=glfw ~/mj/bin/python view_grasp_ok.py
"""

import argparse
import os
import sys
import time

import numpy as np

import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_grasp as S
import sim_mesh_gripper as MG
import sim_grasp_ok as GO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--obj-size', type=float, default=0.020)
    ap.add_argument('--offset', nargs=3, type=float, default=[8.0, -4.0, 0.0])
    ap.add_argument('--approach', type=float, default=0.6)
    a = ap.parse_args()

    print('构建孪生（真网格凸分解爪子）...', flush=True)
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
    op = S.obj_world_pos()
    cq = S.attach_handles(model)[3]
    cb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'object')
    off = np.asarray(a.offset) / 1000.0
    tcp = op + off

    print('打开窗口...', flush=True)
    try:
        import mujoco.viewer as mv
        viewer = mv.launch_passive(model, data)
    except Exception as e:
        print(f'窗口打开失败: {e}', flush=True)
        return
    print('窗口已打开 ✅ 演示抓取循环', flush=True)

    def run_steps(n, grip=None):
        if grip is not None:
            data.ctrl[aadr['gripper']] = grip
        for _ in range(n):
            if not viewer.is_running():
                return False
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)
        return True

    def move_to(target, grip, steps=420):
        """只发 ctrl 让伺服开过去（不瞬移）"""
        q_sol, _ = GO.solve_only(model, qadr, target, data.qpos.copy())
        for j in S.ARM_JOINTS:
            data.ctrl[aadr[j]] = q_sol[S.ARM_JOINTS.index(j)]
        return run_steps(steps, grip)

    while viewer.is_running():
        mujoco.mj_resetData(model, data)
        data.qpos[cq:cq + 3] = op
        data.qpos[cq + 3:cq + 7] = [1, 0, 0, 0]
        mujoco.mj_forward(model, data)
        if not move_to(tcp + np.array([0, 0, 0.06]), a.approach):
            return
        if not move_to(tcp, a.approach):
            return
        if not run_steps(600, 0.0):           # 夹紧 + settle
            return
        if not move_to(tcp + np.array([0, 0, 0.12]), 0.0):
            return
        up = (data.xpos[cb][2] - op[2]) * 1000
        print(f'  物块升高 {up:+.1f}mm  '
              f'{"🎉 抓起" if up > 5 else "❌ 没抓起"}', flush=True)
        for _ in range(300):                   # 停顿展示
            if not viewer.is_running():
                return
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == '__main__':
    main()
