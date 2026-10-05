#!/usr/bin/env python3
"""读 6 个舵机的原始位置/错误标志/温度/负载，并与配置的限位对比

Feetech STS3215 协议：
  读:   FF FF ID LEN 02 ADDR LEN CHK
  应答: FF FF ID LEN ERR PARAM... CHK     ← ERR 就是错误标志
关键寄存器：
  0x38 当前位置(2B)   0x3C 当前负载(2B)   0x3E 电压(1B)   0x3F 温度(1B)

配置（driver_params.yaml）：
  JOINT        zero  min   max
  shoulder_pan 2078   826  3330
  shoulder_lift 1980  842  3118
  elbow_flex   3076  1974  4095   ← 注意 max 顶到 4095
  wrist_flex   2035   954  3116
  wrist_roll   3053  1264  4095   ← 同样顶到 4095（回绕截断）
  gripper      2030  1916  3168

错误标志位（Feetech）：
  0x01 电压  0x02 角度限位  0x04 过热  0x08 电子/磁编码  0x20 过载

用法：
  ~/mj/bin/python read_servo_limits.py          # 需先停 driver 释放串口
"""

import sys
import time

import serial

PORT = '/dev/ttyACM0'
BAUD = 1000000
JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
          'wrist_roll', 'gripper']
ZERO = [2078, 1980, 3076, 2035, 3053, 2030]
RAW_MIN = [826, 842, 1974, 954, 1264, 1916]
RAW_MAX = [3330, 3118, 4095, 3116, 4095, 3168]
ERR_BITS = {0x01: '电压', 0x02: '角度限位', 0x04: '过热', 0x08: '编码',
            0x20: '过载'}


def checksum(bs):
    return (~sum(bs)) & 0xFF


def read_reg(ser, sid, addr, length):
    # Feetech: LENGTH = 参数个数 + 2。读指令参数是 [ADDR, LEN] 两个 → LENGTH=4
    # （之前写成 3 所以所有舵机都不应答）
    body = [sid, 4, 0x02, addr, length]
    pkt = bytes([0xFF, 0xFF] + body + [checksum(body[0:])])
    ser.reset_input_buffer()
    ser.write(pkt)
    ser.flush()
    time.sleep(0.015)
    resp = ser.read(2 + 1 + 1 + 1 + length + 1)
    if len(resp) < 6 + length or resp[0] != 0xFF or resp[1] != 0xFF:
        return None, None, resp
    err = resp[4]
    data = resp[5:5 + length]
    return err, data, resp


def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.25,
                            exclusive=True)
    except Exception as e:
        print(f'❌ 打不开串口（driver 还在跑？）: {e}')
        return 1
    time.sleep(0.2)
    print(f'{PORT} @{BAUD} 已打开\n')
    print('  ID 关节            原始位置  相对零点   配置范围        状态')
    bad = []
    for i, name in enumerate(JOINTS, start=1):
        err, data, raw = read_reg(ser, i, 0x38, 2)
        if data is None:
            print(f'  {i:2d} {name:14s}  ❌ 无应答  (原始: '
                  f'{raw.hex() if raw else "空"})')
            bad.append((i, name, 'no-reply'))
            continue
        pos = data[0] | (data[1] << 8)
        rel = pos - ZERO[i - 1]
        lo, hi = RAW_MIN[i - 1], RAW_MAX[i - 1]
        inside = lo <= pos <= hi
        flags = [v for k, v in ERR_BITS.items() if err & k]
        status = '✅ 在范围内' if inside and not flags else ''
        if not inside:
            status += f' ⚠️ 超限(距{lo if pos < lo else hi} '
            status += f'{pos - (lo if pos < lo else hi):+d})'
        if flags:
            status += f' ❌ 错误: {"+".join(flags)}(0x{err:02X})'
        if (not inside) or flags:
            bad.append((i, name, status))
        print(f'  {i:2d} {name:14s} {pos:7d}   {rel:+7d}   '
              f'[{lo},{hi}]   {status}')
    print()
    # 额外读温度/电压/负载
    print('  额外状态：')
    for i, name in enumerate(JOINTS, start=1):
        e1, t, _ = read_reg(ser, i, 0x3F, 1)
        e2, v, _ = read_reg(ser, i, 0x3E, 1)
        e3, l, _ = read_reg(ser, i, 0x3C, 2)
        ts = f'{t[0]}°C' if t else '?'
        vs = f'{v[0]/10:.1f}V' if v else '?'
        ls = f'{l[0] | (l[1] << 8)}' if l else '?'
        print(f'    {i} {name:14s} 温度 {ts:6s} 电压 {vs:6s} 负载 {ls}')
    ser.close()
    print()
    if bad:
        print('⚠️ 有问题的舵机：')
        for i, name, s in bad:
            print(f'   ID{i} {name}: {s}')
    else:
        print('✅ 全部舵机位置在配置范围内、无错误标志')
    return 0


if __name__ == '__main__':
    sys.exit(main())
