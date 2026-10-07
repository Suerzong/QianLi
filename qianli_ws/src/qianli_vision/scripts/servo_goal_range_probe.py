#!/usr/bin/env python3
"""探测 Goal_Position 是否接受 > 4095 的多圈目标（扭矩必须为 0，只回读验证）

背景
----
`wrist_roll` 的 URDF 行程是 320°(3641 计数)，比一圈 4096 略小；但只要零点
``zero_raw=3053`` 就必然有一边越过 4095，于是 ``raw_max`` 被硬截到 4095，
正向 71° 行程被吃掉。同样的问题也出现在 ``elbow_flex``（吃掉 7°）。

要修只有两条路：
  A. 舵机固件支持 Goal_Position > 4095（多圈）→ 纯软件修复，不动 EEPROM；
  B. 不支持 → 必须改 Homing_Offset 把零位挪到行程中间（lerobot 的做法）。

本脚本用**扭矩关闭 + 写后回读**判定 A，不产生任何运动：
写 Goal_Position 只是写 SRAM 寄存器，扭矩为 0 时舵机不会动；
结束后恢复原值。绝不写 EEPROM、绝不使能力矩。

用法::

    ~/mj/bin/python servo_goal_range_probe.py
"""

from __future__ import annotations

from project_paths import default_arm_port

import argparse
import json
import sys
import time

import serial

PORT = default_arm_port()
BAUD = 1_000_000
JOINT_NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
               'wrist_roll', 'gripper']
SERVO_IDS = [1, 2, 3, 4, 5, 6]

REG_TORQUE_ENABLE = 40
REG_GOAL_POSITION = 42
REG_PRESENT_POSITION = 56

# 用真实需要值做测试：elbow 需要 4177，wrist_roll 需要 4905
TEST_VALUES = [4095, 4177, 4905, 0, 300]


def checksum(bs):
    return (~sum(bs)) & 0xFF


def read_reg(ser, sid, addr, length, attempts=2):
    body = [sid, 4, 0x02, addr, length]
    pkt = bytes([0xFF, 0xFF] + body + [checksum(body)])
    for _ in range(attempts):
        ser.reset_input_buffer()
        ser.write(pkt)
        ser.flush()
        time.sleep(0.006)
        resp = ser.read(2 + 1 + 1 + 1 + length + 1)
        if len(resp) >= 6 + length and resp[0] == 0xFF and resp[1] == 0xFF:
            if resp[:len(pkt)] == pkt and len(resp) == len(pkt):
                continue
            return resp[4], resp[5:5 + length]
    return None, None


def read_u8(ser, sid, addr):
    err, data = read_reg(ser, sid, addr, 1)
    return err, (None if data is None else data[0])


def read_u16(ser, sid, addr):
    err, data = read_reg(ser, sid, addr, 2)
    return err, (None if data is None else data[0] | (data[1] << 8))


def write_u16(ser, sid, addr, value):
    """WRITE 指令；本机实测不回状态包，所以 fire-and-forget。"""
    body = [sid, 5, 0x03, addr, value & 0xFF, (value >> 8) & 0xFF]
    ser.write(bytes([0xFF, 0xFF] + body + [checksum(body)]))
    ser.flush()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=PORT)
    ap.add_argument('--baud', type=int, default=BAUD)
    ap.add_argument('--json', default='/tmp/servo_goal_range_probe.json')
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2, exclusive=True)
    except Exception as exc:  # noqa: BLE001
        print(f'❌ 打不开 {args.port}（driver 还在占用？）: {exc}')
        return 1
    time.sleep(0.2)

    torque = [read_u8(ser, sid, REG_TORQUE_ENABLE)[1] for sid in SERVO_IDS]
    print(f'扭矩状态: {torque}')
    if any(v is None for v in torque):
        print('❌ 有舵机无应答，先排查供电/串口')
        ser.close()
        return 1
    if any(v != 0 for v in torque):
        print('❌ 有舵机扭矩使能中 —— 写 Goal_Position 会造成运动，拒绝执行。')
        print('   请先停掉驱动/`/arm/stop`，确认扭矩全 0 再试。')
        ser.close()
        return 1

    print('✅ 6 个舵机扭矩均为 0：写 Goal_Position 不会产生运动\n')
    report = {'read_at': time.strftime('%Y-%m-%d %H:%M:%S'), 'servos': {}}

    for name, sid in zip(JOINT_NAMES, SERVO_IDS):
        _, original = read_u16(ser, sid, REG_GOAL_POSITION)
        entry = {'original_goal': original, 'tests': []}
        print(f'━━━ ID{sid} {name}  (原 Goal_Position = {original}) ━━━')
        for value in TEST_VALUES:
            write_u16(ser, sid, REG_GOAL_POSITION, value)
            time.sleep(0.02)
            _, echo = read_u16(ser, sid, REG_GOAL_POSITION)
            ok = echo == value
            entry['tests'].append({'wrote': value, 'read_back': echo, 'exact': ok})
            mark = '✅ 原样接受' if ok else f'❌ 被改成 {echo}'
            print(f'   写 {value:5d} → 回读 {echo}   {mark}')
        if original is not None:
            write_u16(ser, sid, REG_GOAL_POSITION, original)
            time.sleep(0.02)
            _, echo = read_u16(ser, sid, REG_GOAL_POSITION)
            entry['restored_to'] = echo
            entry['restore_ok'] = echo == original
            print(f'   恢复原值 {original} → 回读 {echo} '
                  f'{"✅" if echo == original else "⚠️ 恢复失败"}')
        report['servos'][name] = entry
        print()

    ser.close()

    multi_turn = [name for name, e in report['servos'].items()
                  if all(t['exact'] for t in e['tests'])]
    print('=' * 70)
    if len(multi_turn) == 6:
        print('结论：✅ 6 个舵机都接受 >4095 的目标 → 可以用纯软件方式放宽限位')
    elif multi_turn:
        print(f'结论：⚠️ 只有部分舵机接受多圈目标：{multi_turn}')
        print('      其余关节需要改 Homing_Offset 把零位挪到行程中间')
    else:
        print('结论：❌ 舵机不接受 >4095 的目标（或写入被忽略）')
        print('      → 必须改 Homing_Offset 重新定零，把行程挪进 0..4095')
    print('=' * 70)

    with open(args.json, 'w') as fh:
        json.dump(report, fh, indent=2, ensure_ascii=False)
    print(f'📄 报告已写入 {args.json}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
