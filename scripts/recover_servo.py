#!/usr/bin/env python3
"""舵机故障恢复：读错误位/自身限位，尝试清除锁存错误。

该库把状态包里的 error 位当异常抛出，导致连位置都读不到。
这里临时打补丁忽略 error 位，先把现场读清楚，再尝试清除。
"""

from project_paths import default_arm_port

from project_paths import arm_source_path, driver_params_path
import math
import sys
import time

import numpy as np
import yaml
from pathlib import Path

sys.path.insert(0, arm_source_path())
import so101_bringup.servo_protocol as sp
from so101_bringup.servo_protocol import FeetechSerialBus

CONFIG = (driver_params_path())

_orig_parse = sp.parse_status_packet
ERRORS = {}


def patched(packet, expected_id, expected_data_size):
    data = _orig_parse.__wrapped__(packet, expected_id, expected_data_size) \
        if hasattr(_orig_parse, '__wrapped__') else None
    # 自己解析，忽略 error 位
    expected_length = expected_data_size + 2
    if (len(packet) != expected_length + 4 or packet[:2] != sp.HEADER
            or packet[2] != expected_id or packet[3] != expected_length
            or sp.packet_checksum(packet[2:-1]) != packet[-1]):
        raise sp.ServoProtocolError('bad status packet')
    err = packet[4]
    if err:
        ERRORS[expected_id] = err
    return packet[5:-1]


sp.parse_status_packet = patched

cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])

bus = FeetechSerialBus(default_arm_port(), timeout_s=0.08)
IDS = list(bus.IDS)

print('=== 当前状态（忽略 error 位）===')
try:
    raw = bus.read_positions()
    print('raw 位置:', raw)
    q = (np.array(raw) - zero) * direction * 2 * math.pi / 4096
    print('关节角:', np.round(q, 4).tolist())
except Exception as e:
    print('读位置失败:', e)
    raw = None

try:
    print('力矩使能:', bus.read_torque_states())
except Exception as e:
    print('读力矩失败:', e)

print('错误位记录:', {k: f'0x{v:02x}' for k, v in ERRORS.items()})

print('\n=== 舵机自身角度限位 (EEPROM 9/11) ===')
for sid in IDS:
    try:
        lo = bus.read_word(sid, 9)
        hi = bus.read_word(sid, 11)
        cur = raw[sid - 1] if raw else None
        flag = ''
        if cur is not None:
            if cur < lo:
                flag = f'  ⚠ 当前位置低于下限 {lo-cur} 计数'
            elif cur > hi:
                flag = f'  ⚠ 当前位置高于上限 {cur-hi} 计数'
        print(f'  servo{sid}: 限位 {lo}..{hi}  当前 {cur}{flag}')
    except Exception as e:
        print(f'  servo{sid}: 读取失败 {e}')

print('\n=== 尝试清除锁存错误 ===')
try:
    bus.set_torque(False)
    print('已关闭力矩')
    time.sleep(0.8)
except Exception as e:
    print('关力矩失败:', e)
ERRORS.clear()
try:
    bus.set_torque(True)
    print('已重新使能力矩')
    time.sleep(0.8)
except Exception as e:
    print('使能失败:', e)

ERRORS.clear()
try:
    raw2 = bus.read_positions()
    print('重新读位置 OK:', raw2)
except Exception as e:
    print('重新读位置仍失败:', e)
print('清除后错误位:', {k: f'0x{v:02x}' for k, v in ERRORS.items()} or '无')
bus.close()
