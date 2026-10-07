#!/usr/bin/env python3
"""把越界的肘关节挪回 EEPROM 合法区间，并尝试清 0x02 锁存。"""
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

cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
bus = FeetechSerialBus('/dev/ttyACM0', timeout_s=0.15)

# 读当前 EEPROM 限位
lo, hi = [], []
for sid in bus.IDS:
    lo.append(bus.read_word(sid, 9))
    hi.append(bus.read_word(sid, 11))
lo, hi = np.array(lo), np.array(hi)
print('EEPROM 限位:', lo.tolist(), hi.tolist())

for attempt in range(1, 4):
    ERR.clear()
    cur = np.array(bus.read_positions())
    print(f'\n[{attempt}] 当前 raw {cur.tolist()}  错误 {dict(ERR)}')
    # 目标：把每个越界关节挪进限位内（留 60 计数余量）
    goal = np.clip(cur, lo + 60, hi - 60)
    moved = np.abs(goal - cur) > 1
    print(f'     目标 raw {goal.tolist()}  调整 {np.where(moved)[0].tolist()}')
    try:
        bus.set_torque(False)
        time.sleep(0.3)
        bus.write_positions(goal.tolist())
        time.sleep(0.4)
        bus.set_torque(True)
        time.sleep(1.2)
    except Exception as exc:
        print(f'     写入异常: {exc}')
        continue
    ERR.clear()
    try:
        now = np.array(bus.read_positions())
        print(f'     现在 raw {now.tolist()}')
    except Exception as exc:
        print(f'     读取失败: {exc}')
        continue
    print(f'     错误位 {dict(ERR) or "无 ✅"}')
    if not ERR:
        print('\n✅ 锁存错误已清除，且所有关节都在限位内')
        break
bus.close()
