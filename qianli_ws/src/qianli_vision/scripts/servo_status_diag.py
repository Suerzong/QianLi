#!/usr/bin/env python3
"""带校验和的舵机状态诊断（只读；用于判断 0x02 是真实故障还是总线噪声）"""

from __future__ import annotations

import sys
import time

import serial

PORT = '/dev/ttyACM0'
BAUD = 1_000_000
NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
         'wrist_roll', 'gripper']


def checksum(bs):
    return (~sum(bs)) & 0xFF


def read_reg(ser, sid, addr, length, attempts=5):
    """返回 (err, data)；校验和不对就重试。"""
    body = [sid, 4, 0x02, addr, length]
    pkt = bytes([0xFF, 0xFF] + body + [checksum(body)])
    bad = 0
    for _ in range(attempts):
        ser.reset_input_buffer()
        ser.write(pkt)
        ser.flush()
        time.sleep(0.008)
        resp = ser.read(2 + 1 + 1 + 1 + length + 1)
        if len(resp) != 6 + length:
            bad += 1
            continue
        if resp[0] != 0xFF or resp[1] != 0xFF:
            bad += 1
            continue
        if resp[:len(pkt)] == pkt:
            bad += 1
            continue
        if checksum(resp[2:-1]) != resp[-1]:
            bad += 1
            continue
        return resp[4], resp[5:5 + length], bad
    return None, None, bad


def u16(d):
    return None if d is None else d[0] | (d[1] << 8)


def tc16(v):
    return None if v is None else (v - 65536 if v >= 32768 else v)


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.2, exclusive=True)
    time.sleep(0.2)
    print(f'{"ID":>3} {"关节":<14}{"err采样(6次)":>20}{"Status":>8}{"Goal":>8}'
          f'{"Present":>9}{"Offset":>8}{"Min":>7}{"Max":>7}{"坏包":>6}')
    summary = {}
    for sid, name in enumerate(NAMES, 1):
        errs = []
        bad_total = 0
        for _ in range(6):
            err, _, bad = read_reg(ser, sid, 56, 2)
            errs.append(err)
            bad_total += bad
            time.sleep(0.05)
        _, st_d, b = read_reg(ser, sid, 65, 1); bad_total += b
        _, goal_d, b = read_reg(ser, sid, 42, 2); bad_total += b
        _, pres_d, b = read_reg(ser, sid, 56, 2); bad_total += b
        _, off_d, b = read_reg(ser, sid, 31, 2); bad_total += b
        _, min_d, b = read_reg(ser, sid, 9, 2); bad_total += b
        _, max_d, b = read_reg(ser, sid, 11, 2); bad_total += b
        st = None if st_d is None else st_d[0]
        print(f'{sid:>3} {name:<14}'
              f'{str(["0x%02X" % e if e is not None else "?" for e in errs]):>20}'
              f'{("0x%02X" % st) if st is not None else "?":>8}'
              f'{str(u16(goal_d)):>8}{str(tc16(u16(pres_d))):>9}'
              f'{str(tc16(u16(off_d))):>8}{str(u16(min_d)):>7}'
              f'{str(u16(max_d)):>7}{bad_total:>6}')
        summary[name] = errs
    ser.close()
    print()
    for name, errs in summary.items():
        clean = all(e == 0 for e in errs)
        stable = len(set(errs)) == 1
        print(f'  {name:<14} {"✅ 全 0" if clean else ("恒定 0x%02X" % errs[0] if stable else "抖动 -> 总线噪声")}'
              f'   {set("0x%02X" % e for e in errs if e is not None)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
