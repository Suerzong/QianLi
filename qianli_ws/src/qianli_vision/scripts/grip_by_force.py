#!/usr/bin/env python3
"""按力合爪：读 Present_Load 闭到接触为止，而不是闷头合到某个位置

为什么需要闭环
--------------
EVA 泡棉是**可压缩**的，标称 40mm 的块实际可能在 39~41mm，压下去还能再小。
纯位置控制只有两种结局：
  · 目标开度给小了 → 夹太松，抬起时掉；
  · 目标开度给大了 → 压扁料，甚至把块从爪里挤出去。
而"夹住"这件事物理上就是**负载上升**，直接测它最可靠。

三个副产品
----------
1. **接触确认**：负载上升 = 确实夹到东西了，不再是"我以为夹到了"；
2. **防夹碎**：阈值就是最大允许压力；
3. **失败可检**：超时还没到阈值 → 报"没夹到"，而不是硬合到底。

两个必须注意的实现细节
----------------------
* **只能给夹爪单独使能扭矩。** `set_torque(True)` 会使能全部 6 个舵机，
  机械臂会瞬间 snap 到寄存器里残留的目标位置 —— 很危险。所以这里用
  `sync_write_register` 只对夹爪那一个 ID 写 Torque_Enable。
* **Present_Load 不是普通无符号数**：低 10 位是幅值(0~1000=0~100%)，
  bit10 是方向位。当无符号读时反向负载会变成 1024+，看着像"过载 102%"
  其实方向和大小都不对。已在 `servo_protocol.decode_load` 里处理。

**串口是独占的**：驱动在跑的时候这个脚本打不开 /dev/ttyACM0。
要么先停驱动（`pkill -f driver_node`），要么把它做成驱动的一个服务。

用法::

    # 只读，不动任何寄存器（先确认地址对不对、静态负载多少）
    ~/mj/bin/python grip_by_force.py --read-only
    # 真的按力合爪
    ~/mj/bin/python grip_by_force.py --close --load-threshold 12
"""

from __future__ import annotations

from project_paths import default_arm_port

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gripper_model import GripperModel, JOINTS  # noqa: E402

from so101_bringup.servo_protocol import (  # noqa: E402
    REG_GOAL_POSITION, REG_TORQUE_ENABLE, FeetechSerialBus,
    decode_load)

PORT = default_arm_port()
ZERO_RAW = [2078, 1980, 3076, 2035, 2033, 2030]
IDS = [1, 2, 3, 4, 5, 6]
GRIPPER_ID = IDS[-1]
GRIPPER_ZERO = ZERO_RAW[-1]


def rad_to_raw(rad, zero):
    return int(round(zero + rad * 4096.0 / (2 * math.pi))) & 0x0FFF


def raw_to_rad(raw, zero):
    d = (raw - zero) & 0x0FFF
    if d > 2048:
        d -= 4096
    return d * 2 * math.pi / 4096.0


def write_word(bus, servo_id, address, value):
    """只写一个舵机的一个寄存器（不碰其它 5 个）。"""
    bus.sync_write_register(address, 2, [servo_id],
                            bytes([value & 0xFF, (value >> 8) & 0xFF]))


def write_byte(bus, servo_id, address, value):
    bus.sync_write_register(address, 1, [servo_id], bytes([value & 0xFF]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default=PORT)
    ap.add_argument('--read-only', action='store_true',
                    help='只读负载/位置，不写任何寄存器')
    ap.add_argument('--close', action='store_true', help='真的按力合爪')
    ap.add_argument('--load-threshold', type=float, default=12.0,
                    help='负载百分比阈值，到这就认为夹住并停住')
    ap.add_argument('--step-rad', type=float, default=0.008)
    ap.add_argument('--step-s', type=float, default=0.06)
    ap.add_argument('--max-travel-rad', type=float, default=0.60,
                    help='最多往闭合方向走这么多，超过判失败（防止硬合到底）')
    ap.add_argument('--backoff-rad', type=float, default=0.015,
                    help='到位后回退一点，避免持续顶死')
    ap.add_argument('--timeout-s', type=float, default=30.0)
    args = ap.parse_args()

    if not (args.read_only or args.close):
        print('请指定 --read-only 或 --close（默认什么都不做）')
        return 1

    # 注意：构造函数是 (port, baud, timeout_s)，**没有 ids 参数** ——
    # 舵机 ID 列表是类属性 FeetechSerialBus.IDS。
    # 而且它用 exclusive=True 打开串口，所以驱动在跑时这里必然打不开。
    bus = FeetechSerialBus(args.port)
    try:
        print(f'已连接 {args.port}')
        loads = bus.read_loads()
        pos = bus.read_positions()
        print()
        print(f'{"ID":>3} {"位置(raw)":>10} {"角度":>10} {"负载":>9}  方向  关节')
        print('-' * 60)
        for i, sid in enumerate(IDS):
            rd = raw_to_rad(pos[i], ZERO_RAW[i])
            pct, positive = loads[i]
            name = JOINTS[i] if i < len(JOINTS) else '?'
            print(f'{sid:>3} {pos[i]:>10} {math.degrees(rd):>9.2f}° '
                  f'{pct:>8.1f}%  {"+" if positive else "-"}     {name}')

        if args.read_only:
            print('\n--read-only：到此为止，没有写任何寄存器。')
            print('（这一步是在验证 Present_Load 地址 60 读得到、'
                  '且静态负载接近 0）')
            return 0

        m = GripperModel(stride=24)
        cur_rad = raw_to_rad(pos[-1], GRIPPER_ZERO)
        op0 = m.jaw_opening(cur_rad)
        print()
        print(f'当前夹爪角 {math.degrees(cur_rad):+.2f}°  '
              f'开度 {op0*1000 if op0 else float("nan"):.1f}mm')
        print(f'目标：往闭合方向走到负载 >= {args.load_threshold:.1f}%')
        print(f'保护：最多走 {args.max_travel_rad:.3f} rad，'
              f'超时 {args.timeout_s:.0f}s，超了就判"没夹到"')

        # **只给夹爪使能扭矩**，其余 5 个保持松着（见文件头说明）
        write_byte(bus, GRIPPER_ID, REG_TORQUE_ENABLE, 1)
        time.sleep(0.3)
        print(f'已只对夹爪(ID{GRIPPER_ID})使能扭矩，其余 5 个关节保持失力')
        print()
        print(f'{"步":>4} {"角度":>9} {"开度":>10} {"负载":>8}  状态')
        print('-' * 56)

        target = cur_rad
        travelled = 0.0
        hit = False
        pct = 0.0
        t_end = time.time() + args.timeout_s
        k = 0
        while travelled < args.max_travel_rad and time.time() < t_end:
            k += 1
            target -= args.step_rad
            travelled += args.step_rad
            write_word(bus, GRIPPER_ID, REG_GOAL_POSITION,
                       rad_to_raw(target, GRIPPER_ZERO))
            time.sleep(args.step_s)
            pct, positive = bus.read_gripper_load()
            op = m.jaw_opening(target)
            op_s = f'{op*1000:8.1f}mm' if op is not None else '       n/a'
            print(f'{k:>4} {math.degrees(target):>8.2f}° {op_s} '
                  f'{pct:>7.1f}%  {"← 接触" if pct >= args.load_threshold else ""}')
            if pct >= args.load_threshold:
                hit = True
                break

        if hit:
            back = target + args.backoff_rad
            write_word(bus, GRIPPER_ID, REG_GOAL_POSITION,
                       rad_to_raw(back, GRIPPER_ZERO))
            print(f'\n✅ 夹住：负载 {pct:.1f}% >= {args.load_threshold:.1f}%')
            print(f'   回退 {args.backoff_rad:.3f} rad 避免持续顶死，'
                  f'最终角度 {math.degrees(back):+.2f}°')
        else:
            print(f'\n❌ 走了 {travelled:.3f} rad 负载始终没到 '
                  f'{args.load_threshold:.1f}%（最后 {pct:.1f}%）')
            print('   → 判为"没夹到东西"。物块可能不在爪里，或阈值设太高。')
            print('   不硬合到底。夹爪保持当前位置，扭矩未断。')
            rc = 2
        return 0 if hit else 2
    finally:
        bus.close()


if __name__ == '__main__':
    sys.exit(main())
