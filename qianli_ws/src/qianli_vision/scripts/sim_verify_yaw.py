#!/usr/bin/env python3
"""验证"按物块朝向对齐爪面"这个算法（在孪生里）

用户提出的策略：
  1. 视觉分辨物块的边缘 → 得到物块朝向
  2. 调整固定爪的朝向，使其接触面与物块的某一个面**平行**
  3. 再放下去抓

几何依据：
  · 夹爪的两片爪沿 gripper_frame_link 的 +X 方向开合，
    爪的接触面法向 = 开合方向
  · 命令的偏航 θ 使 frame 的 X 轴在 base 系里指向角度 θ
    （R = Rz(θ)·Rx(π) → X 轴 = (cosθ, sinθ, 0)）
  · 要让爪面贴平物块面：开合方向要对齐物块的**面法向**
    → θ = 物块朝向 φ（模 90°，立方体 90° 等价）
  · 横向偏移是在夹爪自身坐标系里的固定量：(4, 8) mm
    （由已验证的 θ=-90°、base 偏移 (8,-4) 反推）
    → base 偏移 = Rz(θ) · (4, 8, 0) mm

本脚本对"物块旋转 30°"的情形做对照实验：
  A) 固定 -90°（旧做法）        B) 对齐物块朝向（新做法）

用法：
  ~/mj/bin/python sim_verify_yaw.py --cube-yaw 30
"""

import argparse
import math
import sys

import numpy as np

import mujoco

import sim_grasp as S
import sim_mesh_gripper as MG
import sim_grasp_ok as GO

GRIP_OFF_LOCAL = np.array([4.0, 8.0, 0.0])   # 夹爪自身坐标系里的横向偏移 mm


def offset_base(theta_deg, off_local=GRIP_OFF_LOCAL):
    """把夹爪坐标系里的偏移旋转到 base 系。"""
    t = math.radians(theta_deg)
    R = np.array([[math.cos(t), -math.sin(t), 0],
                  [math.sin(t), math.cos(t), 0],
                  [0, 0, 1]])
    return (R @ off_local)


def set_cube_yaw(model, data, cq, op, yaw_deg):
    c, s = math.cos(math.radians(yaw_deg) / 2), math.sin(
        math.radians(yaw_deg) / 2)
    data.qpos[cq:cq + 3] = op
    data.qpos[cq + 3:cq + 7] = [c, 0, 0, s]
    mujoco.mj_forward(model, data)


def run(model, data, qadr, aadr, op, cq, cube_yaw, theta, obj_size,
        approach=0.6):
    off = offset_base(theta)
    tcp = op + off / 1000.0
    mujoco.mj_resetData(model, data)
    set_cube_yaw(model, data, cq, op, cube_yaw)
    e1 = GO.goto(model, data, qadr, aadr, tcp + np.array([0, 0, 0.06]),
                 steps=500, grip=approach, yaw=theta)
    e2 = GO.goto(model, data, qadr, aadr, tcp, steps=500, grip=approach,
                 yaw=theta)
    data.ctrl[aadr['gripper']] = 0.0
    for _ in range(800):
        mujoco.mj_step(model, data)
    cb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'object')
    e3 = GO.goto(model, data, qadr, aadr, tcp + np.array([0, 0, 0.12]),
                 steps=600, grip=0.0, yaw=theta)
    up = (data.xpos[cb][2] - op[2]) * 1000
    return up > 5.0, up, (e1, e2, e3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--obj-size', type=float, default=0.020)
    ap.add_argument('--cube-yaw', type=float, default=30.0,
                    help='物块相对 base 系的旋转角（度）')
    ap.add_argument('--approach', type=float, default=0.6)
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
    cq = S.attach_handles(model)[3]
    op = S.obj_world_pos()

    print(f'物块 {a.obj_size*1000:.0f}mm，物块朝向 {a.cube_yaw:+.1f}°')
    print(f'夹爪自身系偏移 = {GRIP_OFF_LOCAL[:2]} mm\n')
    print('  命令偏航θ   是否对齐物块面   base偏移(mm)      结果     升高(mm)')
    for theta in (-90.0, a.cube_yaw, a.cube_yaw - 90.0, a.cube_yaw + 45.0):
        aligned = (abs(((theta - a.cube_yaw) % 90.0 + 45) % 90 - 45) < 1.0)
        off = offset_base(theta)
        ok, up, errs = run(model, data, qadr, aadr, op, cq, a.cube_yaw,
                           theta, a.obj_size, a.approach)
        print(f'  {theta:+7.1f}°      {"✅ 平行" if aligned else "❌ 不平行"}'
              f'        ({off[0]:+6.2f},{off[1]:+6.2f})   '
              f'{"🎉 抓起" if ok else "❌ 没抓起"}  {up:+8.1f}', flush=True)


if __name__ == '__main__':
    main()
