#!/usr/bin/env python3
"""训练出的抓取策略（在孪生里验证）：偏移补偿 + 螺旋搜索

策略来源（孪生里扫出来的）：
  1. TCP 参考点比爪口低 3.4cm → 目标点沿工具轴下移 dz=-30mm
  2. 横向对齐要求 ±2mm（很紧）→ 一次不成就换小偏移重试
  3. 物块尺寸必须 12~16mm（20mm 塞不进 19.6mm 的爪口）

本脚本验证：给定位姿误差（初始错位），螺旋搜索能把成功率提到多少。

用法：
  ~/mj/bin/python sim_policy.py            # 验证策略鲁棒性
  ~/mj/bin/python sim_policy.py --list     # 打印策略参数
"""

import argparse
import math
import sys

import numpy as np

import mujoco

sys.argv = [sys.argv[0]]
import sim_grasp as S   # noqa: E402
import sim_sweep_offset as SW   # noqa: E402

def make_spiral(radius_mm=4, pitch_mm=1.0):
    """生成 1mm 步长的二维螺旋搜索点（按到原点距离排序，先近后远）。"""
    pts = []
    n = int(radius_mm / pitch_mm)
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            pts.append((i * pitch_mm, j * pitch_mm))
    pts.sort(key=lambda p: (p[0] ** 2 + p[1] ** 2))
    return pts


# ---- 训练得到的策略参数 ----
POLICY = {
    'obj_size': 0.014,      # 物块尺寸（12~16mm 可用）
    'dz': -0.030,           # TCP 目标沿工具轴下移（补偿 3.4cm 参考点偏移）
    # 1mm 步长二维螺旋：X/Y 都要搜（实测 Y 方向容差更紧，只有 0 可行）
    'spiral': make_spiral(4, 1.0),
    'grip_kp': 20.0,
}


def build(sz, kp_grip):
    S._OBJ_SIZE_OVERRIDE[0] = sz
    m = S.build_model()
    for i in range(m.nu):
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if n == 'servo_gripper':
            m.actuator_gainprm[i][0] = kp_grip
            m.actuator_biasprm[i][1] = -kp_grip
            m.actuator_biasprm[i][2] = -2.0 * math.sqrt(kp_grip) * 0.2
            m.actuator_forcerange[i][:]=[-3.0, 3.0]
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args()
    if a.list:
        print('训练得到的抓取策略：')
        for k, v in POLICY.items():
            print(f'  {k} = {v}')
        return

    model = build(POLICY['obj_size'], POLICY['grip_kp'])
    ch, names = S.make_chain()
    print(f"策略: 物块 {POLICY['obj_size']*1000:.0f}mm, "
          f"dz={POLICY['dz']*1000:+.0f}mm, 二维螺旋 {len(POLICY['spiral'])} 点")
    print('\n模拟"视觉/IK 有 (ex, ey) mm 误差"时，策略的最终结果：')
    print('  真实误差(mm)     尝试次数  净偏移(mm)      结果')
    ok_count = 0
    cases = [(0, 0), (2, 0), (0, 2), (-2, 0), (0, -2), (3, 3), (-3, 3),
             (4, 0), (0, 4), (2, -2), (-2, -2)]
    for ex, ey in cases:
        result = None
        for attempt, (ox, oy) in enumerate(POLICY['spiral'], start=1):
            nx, ny = ox - ex, oy - ey
            d = mujoco.MjData(model)
            ok, nc, moved, ga = SW.trial(model, d, ch, names, nx / 1000.0,
                                         ny / 1000.0, POLICY['dz'],
                                         POLICY['obj_size'])
            if ok:
                result = (attempt, nx, ny)
                break
        if result:
            ok_count += 1
            print(f'  ({ex:+3.0f},{ey:+3.0f})          第{result[0]:2d}次   '
                  f'({result[1]:+4.0f},{result[2]:+4.0f})mm   ✅', flush=True)
        else:
            print(f'  ({ex:+3.0f},{ey:+3.0f})          全失败   --          ❌',
                  flush=True)
    print(f'\n成功率 {ok_count}/{len(cases)}  '
          f'({ok_count/len(cases)*100:.0f}%)')


if __name__ == '__main__':
    main()
