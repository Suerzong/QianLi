#!/usr/bin/env python3
"""为 RL 快速环境标定夹爪碰撞体（两片盒子爪）。

背景（本会话踩的坑）：
  · sim_gripper_fix.py 的两片盒子爪**未经验证**：活动爪中心比固定爪高
    26~37mm，闭合时只有固定爪碰到物块（6 接触 vs 0），物块抬不起来。
  · 从引擎量出真实铰链：轴 = gripper_link 的 -y，铰链点 gl(-19, 18.8, 22)mm
    （活动爪 body 原点 gl(20.2, 18.8, -23.4)mm 不在铰链上！）
  · 活动爪的局部 z 轴恒等于 gl 的 -z（朝下），所以指面始终竖直（不歪）。
  · 但爪中心位置是**一维轨迹**：x 和 z 都由夹爪角唯一决定，不能独立指定。

本脚本的做法：不猜摆位，直接以"夹持角"为参数反推 moving_pos，
再用**真实抓取仿真**当裁判（物理抬起物块才算数）。

用法：
  ~/mj/bin/python tune_fingers.py --cand        # 列出候选摆位与几何指标
  ~/mj/bin/python tune_fingers.py --pick        # 逐候选跑真抓取，选最好的
"""

import argparse
import math

import numpy as np

import mujoco

import sim_grasp as S
import probe_gripper_fast as P

GL = 'gripper_link'
MJ = 'moving_jaw_so101_v1_link'

# 物块 20mm：z 范围 gl(-108.1, -88.1)mm，TCP 在 gl z=-98.13mm
# 指面半高 10mm。指中心放在 z=-0.099 → 指面 gl z 从 -0.109 到 -0.089
# → 完整盖住 20mm 物块的侧面（物块两端各留 ~0.9mm 余量）
FINGER_HALF = (0.004, 0.009, 0.010)
FIXED_X = 0.014          # 内表面在 gl x=+0.010 = 物块右面
FINGER_Z = -0.099        # 指中心 gl z（低于 TCP 约 0.9mm）
Y_GL = -0.004            # 来自已验证的 dy=-4mm


def jaw_pose(model, data, q, g):
    """返回 gl 系下 (gripper_link 旋转, moving_jaw 的 pos, moving_jaw 的 R)。"""
    data.qpos[q['gripper']] = g
    mujoco.mj_forward(model, data)
    gl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, GL)
    mj = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, MJ)
    Rg = data.xmat[gl].reshape(3, 3)
    return Rg, Rg.T @ (data.xpos[mj] - data.xpos[gl]), Rg.T @ data.xmat[mj].reshape(3, 3)


def moving_pos_for(model, data, q, gstar, obj_size, finger_half, z_gl,
                   fixed_x, y_gl):
    """求"在 gstar 处内表面恰好贴物块左面"所需的 moving_pos（moving_jaw 局部）。

    注意：指面的法向 = 活动爪局部 x 轴。所以内表面的 gl x 坐标 =
    爪中心 gl x + half_x*(Rm 的 0 行 · 局部x轴在 gl 的水平投影)。这里直接
    构造爪中心目标点为 (-(obj/2+half_x), y, z)，因为活动爪局部 x 轴在
    gl 系里**没有 y 分量、且水平分量就是 ±1**（实测 Rm 的 0 行 2 分量为 0），
    即指面严格竖直、法向沿 gl x。
    """
    Rg, pm, Rm = jaw_pose(model, data, q, gstar)
    target = np.array([-(obj_size / 2 + finger_half[0]), y_gl, z_gl])
    mp = Rm.T @ (target - pm)
    # 顺便算出固定爪中心（与 g 无关）
    fixed_pos = np.array([fixed_x, y_gl, z_gl])
    return fixed_pos, mp, Rm


def cand_table(obj_size=0.020, finger_half=FINGER_HALF, z_gl=FINGER_Z,
               fixed_x=FIXED_X, y_gl=Y_GL):
    """扫夹持角 gstar，输出每个 gstar 下的几何指标。"""
    model = P.base_model(obj_size)
    data = mujoco.MjData(model)
    q = P.qaddr(model)
    gl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, GL)
    gl_ = gl
    rows = []
    for gstar in np.arange(0.20, 1.7001, 0.02):
        fixed_pos, mp, Rm = moving_pos_for(model, data, q, gstar, obj_size,
                                           finger_half, z_gl, fixed_x, y_gl)
        # 爪中心离铰链的距离：铰链 gl(-19, 18.8, 22)mm。mp 是局部平移，
        # 爪中心 gl = pm + Rm@mp = target（构造保证）。
        pivot_gl = np.array([-0.019, 0.0188, 0.022])
        r = np.linalg.norm(np.array([-(obj_size / 2 + finger_half[0]),
                                     y_gl, z_gl]) - pivot_gl)
        # 用"爪子装在活动爪上扫过"的方式算口宽 & 两爪中心 z 差
        m2, ffix, fmov = P.build_fast(
            obj_size, finger_half=tuple(finger_half),
            fixed_pos=tuple(fixed_pos), moving_pos=tuple(mp))
        d2 = mujoco.MjData(m2)
        q2 = P.qaddr(m2)
        gl2 = mujoco.mj_name2id(m2, mujoco.mjtObj.mjOBJ_BODY, GL)
        gaps, dzs, dcdg = [], [], []
        gs = np.arange(-0.10, 1.7451, 0.02)
        cxs = np.empty(len(gs))
        for i, g in enumerate(gs):
            d2.qpos[q2['gripper']] = g
            mujoco.mj_forward(m2, d2)
            Rg2 = d2.xmat[gl2].reshape(3, 3)
            cf = Rg2.T @ (d2.geom_xpos[ffix] - d2.xpos[gl2])
            cm = Rg2.T @ (d2.geom_xpos[fmov] - d2.xpos[gl2])
            cxs[i] = cm[0]
            gaps.append(mujoco.mj_geomDistance(m2, d2, ffix, fmov, 1.0, None))
            dzs.append(cm[2] - cf[2])
        dcx = float(np.gradient(cxs, gs)[int(np.argmin(np.abs(gs - gstar)))])
        gi = int(np.argmin(np.abs(gs - gstar)))
        # 该爪子"闭合到 0"的角（口宽过零且从正到负）
        gclose = None
        for i in range(1, len(gs)):
            if gaps[i - 1] > 0 >= gaps[i]:
                gclose = gs[i]
                break
        rows.append(dict(gstar=float(gstar), mp=np.array(mp), r=r,
                         gap=float(gaps[gi]), dz=float(dzs[gi]),
                         dcx=dcx, gclose=gclose,
                         gap_open=float(gaps[-1]),
                         fixed_pos=np.array(fixed_pos)))
    return rows


def show(obj_size=0.020):
    rows = cand_table(obj_size)
    print('=== 候选夹持角（指面在 gstar 处正好贴物块两侧）===')
    print('  gstar   距铰链mm  dc_x/dg mm/rad  gstar处口宽  两爪z差  闭合角   '
          '全开口宽')
    for r in rows:
        gc = f'{r["gclose"]:+.2f}' if r['gclose'] is not None else '  --  '
        ok = '  ← 推荐' if (r['gclose'] is not None and
                           abs(r['gstar'] - r['gclose']) < 0.02 and
                           abs(r['dz']) < 0.005) else ''
        print(f'  {r["gstar"]:+.2f}   {r["r"]*1000:6.1f}    '
              f'{r["dcx"]*1000:+10.1f}     {r["gap"]*1000:+7.2f}  '
              f'{r["dz"]*1000:+6.2f}   {gc}   {r["gap_open"]*1000:6.1f}{ok}')
    return rows


def pick(obj_size=0.020, offsets=((0, 0, 0), (4, -4, 0), (0, -4, 0),
                                 (0, -8, 0), (8, -4, 0), (4, 0, 0),
                                 (-4, -4, 0), (0, 4, 0),
                                 (4, -4, -0.006), (4, -4, 0.006))):
    """逐候选摆位跑**真实抓取**（快速直摆版），成功抬起物块才算数。"""
    rows = cand_table(obj_size)
    # 粗筛：闭合角存在、两爪 z 差小、全开够大
    short = [r for r in rows if r['gclose'] is not None and
             abs(r['dz']) < 0.010 and r['gap_open'] > 0.030]
    print(f'粗筛后 {len(short)}/{len(rows)} 个候选进入真实抓取测试'
          f'（每个 × {len(offsets)} 偏移）', flush=True)
    results = []
    for r in short:
        best = None
        for off in offsets:
            up, nfix, nmov, ang, e = P.grasp_direct(
                obj_size=obj_size, off_mm=off, approach=r['gclose'] + 0.15,
                finger_half=FINGER_HALF, fixed_pos=tuple(r['fixed_pos']),
                moving_pos=tuple(r['mp']))
            two = (nfix > 0 and nmov > 0)
            score = up if two else up - 100.0      # 必须两侧接触
            if best is None or score > best[0]:
                best = (score, up, off, nfix, nmov, ang)
        results.append((r['gstar'], r, best))
        print(f'  gstar={r["gstar"]:+.2f} close={r["gclose"]:+.2f} '
              f'dz={r["dz"]*1000:+.2f}mm | 最好 up={best[1]:+7.1f}mm '
              f'off={best[2]} 接触(固/活)={best[3]}/{best[4]} '
              f'夹爪角={best[5]:+.3f}', flush=True)
    results.sort(key=lambda t: -t[2][0])
    print('\n=== 排名（按最高抬升，且要求两侧接触）===')
    for g, r, b in results[:6]:
        print(f'  gstar={g:+.2f}  up={b[1]:+7.1f}mm  off={b[2]}  '
              f'接触={b[3]}/{b[4]}  moving_pos(mm)='
              f'{np.round(r["mp"]*1000,2)}')
    if results:
        g, r, b = results[0]
        print(f'\n最佳摆位：\n  FIXED_POS  = {tuple(np.round(r["fixed_pos"],6))}'
              f'\n  MOVING_POS = {tuple(np.round(r["mp"],6))}'
              f'\n  APPROACH   = {r["gclose"]+0.15:.3f}'
              f'\n  G_CLOSE    = {r["gclose"]:.3f}')
    return results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cand', action='store_true')
    ap.add_argument('--pick', action='store_true')
    a = ap.parse_args()
    if a.pick:
        pick()
    else:
        show()


if __name__ == '__main__':
    main()
