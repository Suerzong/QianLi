#!/usr/bin/env python3
"""把舵机 EEPROM 的角度限位改成实测机械行程（根治 0x02 角度限位锁存）。

背景：舵机 EEPROM 里的 Min/Max Angle Limit 出厂值太窄（elbow 3069），
而实测机械行程到 4064。命令一旦越过 EEPROM 限位就报 0x02 并锁存、
切断全部力矩。软件层限位改不动这个，必须改舵机 EEPROM。

寄存器（Feetech STS/SCS）：
  9  = Min Angle Limit (2B, EEPROM)
  11 = Max Angle Limit (2B, EEPROM)
  55 = Lock (0=可写, 1=锁定)
"""
import argparse
import json
import math
import sys
import time

import numpy as np
import yaml
from pathlib import Path

sys.path.insert(0, '/home/ros/legacy/arm/arm-final/ros2_ws/src/so101_bringup')
import so101_bringup.servo_protocol as sp
from so101_bringup.servo_protocol import FeetechSerialBus

CONFIG = ('/home/ros/legacy/arm/arm-final/ros2_ws/install/so101_bringup/share/'
          'so101_bringup/config/driver_params.yaml')
REG_MIN, REG_MAX, REG_LOCK = 9, 11, 55
ERR = {}


def patched(packet, expected_id, expected_data_size):
    expected_length = expected_data_size + 2
    if (len(packet) != expected_length + 4 or packet[:2] != sp.HEADER
            or packet[2] != expected_id or packet[3] != expected_length
            or sp.packet_checksum(packet[2:-1]) != packet[-1]):
        raise sp.ServoProtocolError('bad status packet')
    if packet[4]:
        ERR[expected_id] = packet[4]
    return packet[5:-1]


sp.parse_status_packet = patched


def u16le(v):
    return bytes((v & 0xFF, (v >> 8) & 0xFF))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--apply', action='store_true')
    ap.add_argument('--source', choices=('measured', 'frozen'),
                    default='measured',
                    help='measured=实测机械死点；frozen=冻结的软件限位')
    ap.add_argument('--margin', type=int, default=150,
                    help='在实测死点基础上外扩的余量（计数）')
    a = ap.parse_args()
    cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
    names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
             'wrist_roll', 'gripper']
    if a.source == 'measured':
        rep = json.load(open('/tmp/joint_ranges_merged.json'))
        raw_min = [int(rep['joints'][n]['measured_min_unwrapped'])
                   for n in names]
        raw_max = [int(rep['joints'][n]['measured_max_unwrapped'])
                   for n in names]
        raw_min = [max(0, v - a.margin) for v in raw_min]
        raw_max = [min(4095, v + a.margin) for v in raw_max]
    else:
        raw_min = list(cfg['raw_min'])
        raw_max = list(cfg['raw_max'])
    ids = [1, 2, 3, 4, 5, 6]

    bus = FeetechSerialBus('/dev/ttyACM0', timeout_s=0.15)
    print('目标 EEPROM 限位（= 冻结的实测机械行程）：')
    for n, mn, mx in zip(names, raw_min, raw_max):
        print(f'  {n:<15} {mn:5d}..{mx:5d}')
    print('\n当前 EEPROM 限位：')
    cur = {}
    for sid, n in zip(ids, names):
        lo = bus.read_word(sid, REG_MIN)
        hi = bus.read_word(sid, REG_MAX)
        lk = bus.read_byte(sid, REG_LOCK)
        cur[sid] = (lo, hi, lk)
        need = (lo != raw_min[sid - 1] or hi != raw_max[sid - 1])
        print(f'  servo{sid} {n:<15} {lo:5d}..{hi:5d}  lock={lk}'
              f'{"   ← 需更新" if need else "   OK"}')
    if not a.apply:
        print('\nPLAN ONLY（--apply 才写 EEPROM）')
        bus.close()
        return 0

    print('\n写 EEPROM...')
    bus.set_torque(False)
    time.sleep(0.5)
    for sid in ids:
        if cur[sid][2]:                    # 锁定 -> 解锁
            bus.sync_write_register(REG_LOCK, 1, [sid], bytes((0,)))
            time.sleep(0.05)
    time.sleep(0.2)
    bus.sync_write_register(REG_MIN, 2, ids,
                            b''.join(u16le(v) for v in raw_min))
    time.sleep(0.3)
    bus.sync_write_register(REG_MAX, 2, ids,
                            b''.join(u16le(v) for v in raw_max))
    time.sleep(0.5)

    print('回读校验：')
    ok = True
    for sid, n in zip(ids, names):
        lo = bus.read_word(sid, REG_MIN)
        hi = bus.read_word(sid, REG_MAX)
        good = (lo == raw_min[sid - 1] and hi == raw_max[sid - 1])
        ok = ok and good
        print(f'  servo{sid} {n:<15} {lo:5d}..{hi:5d}  '
              f'{"✅" if good else "❌"}')

    # 尝试清锁存错误：写一个落在新限位内的目标，再开关力矩
    ERR.clear()
    now = bus.read_positions()
    print(f'\n当前 raw {now}')
    goal = [int(np.clip(v, lo_, hi_)) for v, lo_, hi_
            in zip(now, raw_min, raw_max)]
    bus.write_positions(goal)
    time.sleep(0.3)
    bus.set_torque(True)
    time.sleep(0.8)
    ERR.clear()
    try:
        now2 = bus.read_positions()
        print(f'重新读取 OK: {now2}')
    except Exception as exc:
        print(f'重新读取仍失败: {exc}')
    print('错误位:', {k: f'0x{v:02x}' for k, v in ERR.items()} or '无 ✅')
    print(f'\nEEPROM 限位写入{"成功" if ok else "未完全成功"}')
    bus.close()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
