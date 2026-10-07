#!/usr/bin/env python3
"""清除舵机残留的越界 Goal_Position（扭矩必须为 0；写 SRAM，无运动）

现象：driver 连不上，日志反复报
    servo 1 reported error flags 0x02      (0x02 = 角度限位)

原因：之前用 Goal_Position 探针测多圈支持时，把 Goal_Position 恢复成了"原值 0"，
而 0 低于 6 个舵机各自的 Min_Position_Limit(674/856/787/838/505/2034)，
固件因此持续置"角度限位"错误标志。

修法：把每个舵机的 Goal_Position 写成它当前的 Present_Position —— 既在限位内，
又是零位移目标，扭矩关闭时不会有任何动作。
"""

from __future__ import annotations

from project_paths import default_arm_port

import sys
import time

import serial

PORT = default_arm_port()
BAUD = 1_000_000
NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
         'wrist_roll', 'gripper']
REG_TORQUE_ENABLE = 40
REG_MIN_LIMIT = 9
REG_MAX_LIMIT = 11
REG_GOAL_POSITION = 42
REG_PRESENT_POSITION = 56


def checksum(bs):
    return (~sum(bs)) & 0xFF


def read_reg(ser, sid, addr, length, attempts=4):
    body = [sid, 4, 0x02, addr, length]
    pkt = bytes([0xFF, 0xFF] + body + [checksum(body)])
    for _ in range(attempts):
        ser.reset_input_buffer()
        ser.write(pkt)
        ser.flush()
        time.sleep(0.008)
        resp = ser.read(2 + 1 + 1 + 1 + length + 1)
        if len(resp) >= 6 + length and resp[0] == 0xFF and resp[1] == 0xFF:
            if resp[:len(pkt)] == pkt and len(resp) == len(pkt):
                continue
            return resp[4], resp[5:5 + length]
    return None, None


def u16(data):
    return None if data is None else data[0] | (data[1] << 8)


def write_u16(ser, sid, addr, value):
    body = [sid, 5, 0x03, addr, value & 0xFF, (value >> 8) & 0xFF]
    ser.write(bytes([0xFF, 0xFF] + body + [checksum(body)]))
    ser.flush()
    time.sleep(0.03)


def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.2, exclusive=True)
    except Exception as exc:  # noqa: BLE001
        print(f'❌ 打不开 {PORT}: {exc}')
        return 1
    time.sleep(0.2)

    torque = []
    for sid in range(1, 7):
        err, data = read_reg(ser, sid, REG_TORQUE_ENABLE, 1)
        torque.append(None if data is None else data[0])
    print(f'扭矩状态: {torque}')
    if any(v is None for v in torque):
        print('❌ 有舵机无应答')
        ser.close()
        return 1
    if any(v != 0 for v in torque):
        print('❌ 扭矩使能中，拒绝执行')
        ser.close()
        return 1

    print(f'\n{"ID":>3} {"关节":<14}{"错误标志":>9}{"Goal":>8}{"Present":>9}'
          f'{"Min":>7}{"Max":>7}  处理')
    fixed = []
    for sid, name in zip(range(1, 7), NAMES):
        err, goal_d = read_reg(ser, sid, REG_GOAL_POSITION, 2)
        _, pres_d = read_reg(ser, sid, REG_PRESENT_POSITION, 2)
        _, min_d = read_reg(ser, sid, REG_MIN_LIMIT, 2)
        _, max_d = read_reg(ser, sid, REG_MAX_LIMIT, 2)
        goal, present = u16(goal_d), u16(pres_d)
        lo, hi = u16(min_d), u16(max_d)
        flags = [] if err is None else [b for b in (0x01, 0x02, 0x04, 0x08, 0x20)
                                        if err & b]
        action = '—'
        if err == 0x02 or (goal is not None and lo is not None
                           and not (lo <= goal <= hi)):
            target = present if present is not None else 2048
            write_u16(ser, sid, REG_GOAL_POSITION, target)
            time.sleep(0.05)
            err2, goal2_d = read_reg(ser, sid, REG_GOAL_POSITION, 2)
            action = f'Goal→{u16(goal2_d)} 标志→0x{err2:02X}'
            fixed.append(name)
        print(f'{sid:>3} {name:<14}{("0x%02X" % err) if err is not None else "?":>9}'
              f'{goal:>8}{present:>9}{lo:>7}{hi:>7}  {action}')

    print()
    if fixed:
        print(f'✅ 已修正：{fixed}')
    else:
        print('✅ 无需修正')

    # 复核
    print('\n复核错误标志：')
    allok = True
    for sid, name in zip(range(1, 7), NAMES):
        err, _ = read_reg(ser, sid, REG_PRESENT_POSITION, 2)
        _, goal_d = read_reg(ser, sid, REG_GOAL_POSITION, 2)
        ok = err == 0
        allok &= ok
        print(f'  ID{sid} {name:<14} err=0x{err:02X}  Goal={u16(goal_d)}  '
              f'{"✅" if ok else "❌"}')
    ser.close()
    return 0 if allok else 1


if __name__ == '__main__':
    sys.exit(main())
