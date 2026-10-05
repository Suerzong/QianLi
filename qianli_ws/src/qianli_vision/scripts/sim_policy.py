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

# ---- 训练得到的策略参数 ----
POLICY = {
    'obj_size': 0.014,      # 物块尺寸（12~16mm 可用）
    'dz': -0.030,           # TCP 目标沿工具轴下移（补偿 3.4cm 参考点偏移）
    'spiral_mm': [0.0, -2.0, 2.0, -4.0, 4.0, -6.0, 6.0, -8.0, 8.0],   # 横向搜索偏移（X 方向）
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
          f"dz={POLICY['dz']*1000:+.0f}mm, 螺旋搜索 "
          f"{POLICY['spiral_mm']} mm")
    print('\n模拟"视觉/IK 有 err mm 横向误差"时，策略的最终结果：')
    print('  初始误差(mm)  尝试次数  净偏移(mm)  结果')
    ok_count = 0
    cases = [-6, -4, -3, -2, -1, 0, 1, 2, 3, 4, 6]
    for err in cases:
        # 我们以为物块在原点，实际偏了 err；每次尝试再加一个搜索偏移 off，
        # 于是仿真里 TCP 相对物块的净偏移 = off - err
        result = None
        for attempt, off in enumerate(POLICY['spiral_mm'], start=1):
            net = (off - err) / 1000.0
            d = mujoco.MjData(model)
            ok, nc, moved, ga = SW.trial(model, d, ch, names, net, 0.0,
                                         POLICY['dz'], POLICY['obj_size'])
            if ok:
                result = (attempt, off - err)
                break
        if result:
            ok_count += 1
            print(f'  {err:+6.1f}      第{result[0]}次     '
                  f'{result[1]:+5.1f}mm   ✅', flush=True)
        else:
            print(f'  {err:+6.1f}      全失败        --      ❌', flush=True)
    print(f'\n成功率 {ok_count}/{len(cases)}  '
          f'({ok_count/len(cases)*100:.0f}%)')


if __name__ == '__main__':
    main()
