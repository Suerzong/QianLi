#!/usr/bin/env python3
"""尝试软件清除舵机锁存错误：写入合法目标位 -> 关/开力矩 -> 复读错误位。"""
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, '/home/ros/legacy/arm/arm-final/ros2_ws/src/so101_bringup')
import so101_bringup.servo_protocol as sp
from so101_bringup.servo_protocol import FeetechSerialBus

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

SAFE = os.path.expanduser('~/QianLi/qianli_ws/config/safe_limits.json')
sl = json.load(open(SAFE))
slo, shi = np.array(sl['raw_lo']), np.array(sl['raw_hi'])

bus = FeetechSerialBus('/dev/ttyACM0', timeout_s=0.08)
try:
    cur = np.array(bus.read_positions())
    print('当前 raw:', cur.tolist())
    tgt = np.clip(cur, slo, shi)
    print('钳位后目标:', tgt.tolist())
    print('错误位(前):', {k: f'0x{v:02x}' for k, v in ERR.items()} or '无')
    ERR.clear()
    bus.set_torque(False)
    time.sleep(0.5)
    print('写入合法目标位...')
    bus.write_positions(tgt.tolist())
    time.sleep(0.5)
    print('错误位(写后):', {k: f'0x{v:02x}' for k, v in ERR.items()} or '无')
    ERR.clear()
    bus.set_torque(True)
    time.sleep(1.0)
    print('力矩:', bus.read_torque_states())
    ERR.clear()
    now = np.array(bus.read_positions())
    print('复读位置 OK:', now.tolist())
    print('错误位(清除尝试后):',
          {k: f'0x{v:02x}' for k, v in ERR.items()} or '无（已清除）')
except Exception as e:
    print('失败:', type(e).__name__, e)
finally:
    bus.close()
