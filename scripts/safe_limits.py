#!/usr/bin/env python3
"""读取舵机自身角度限位(EEPROM 9/11)与驱动配置限位，输出安全交集。

根因：driver_params.yaml 里几个关节的 raw_min/raw_max 超出了舵机自身
EEPROM 限位，命令打到那里会触发 0x02 角度限位错误并锁存、切断力矩。
本工具把两者取交集，供所有运动脚本使用。
"""
import json
import sys

import numpy as np
import yaml
from pathlib import Path

sys.path.insert(0, '/home/ros/legacy/arm/arm-final/ros2_ws/src/so101_bringup')
import so101_bringup.servo_protocol as sp
from so101_bringup.servo_protocol import FeetechSerialBus

CONFIG = ('/home/ros/legacy/arm/arm-final/ros2_ws/install/so101_bringup/share/'
          'so101_bringup/config/driver_params.yaml')
OUT = '/home/ros/QianLi/qianli_ws/config/safe_limits.json'


def patched(packet, expected_id, expected_data_size):
    expected_length = expected_data_size + 2
    if (len(packet) != expected_length + 4 or packet[:2] != sp.HEADER
            or packet[2] != expected_id or packet[3] != expected_length
            or sp.packet_checksum(packet[2:-1]) != packet[-1]):
        raise sp.ServoProtocolError('bad status packet')
    return packet[5:-1]


sp.parse_status_packet = patched

cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
zero = np.array(cfg['zero_raw'])
direction = np.array(cfg['direction'])
d_lo = np.array(cfg['raw_min'])
d_hi = np.array(cfg['raw_max'])
d_lo, d_hi = np.minimum(d_lo, d_hi), np.maximum(d_lo, d_hi)

bus = FeetechSerialBus('/dev/ttyACM0', timeout_s=0.08)
s_lo, s_hi = [], []
for sid in bus.IDS:
    s_lo.append(bus.read_word(sid, 9))
    s_hi.append(bus.read_word(sid, 11))
cur = bus.read_positions()
bus.close()
s_lo, s_hi = np.array(s_lo), np.array(s_hi)
s_lo, s_hi = np.minimum(s_lo, s_hi), np.maximum(s_lo, s_hi)

names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
         'wrist_roll', 'gripper']
safe_lo = np.maximum(d_lo, s_lo)
safe_hi = np.minimum(d_hi, s_hi)
print(f'{"关节":>15}{"驱动限位":>18}{"舵机自身":>18}{"安全交集":>18}  当前')
for i, n in enumerate(names):
    bad = '  ⚠' if (d_lo[i] < s_lo[i] or d_hi[i] > s_hi[i]) else ''
    print(f'{n:>15}{f"{d_lo[i]}..{d_hi[i]}":>18}'
          f'{f"{s_lo[i]}..{s_hi[i]}":>18}'
          f'{f"{safe_lo[i]}..{safe_hi[i]}":>18}  {cur[i]}{bad}')

# 弧度换算
lo_rad = (safe_lo - zero) * direction * 2 * np.pi / 4096
hi_rad = (safe_hi - zero) * direction * 2 * np.pi / 4096
lo_rad, hi_rad = np.minimum(lo_rad, hi_rad), np.maximum(lo_rad, hi_rad)
json.dump({'raw_lo': safe_lo.tolist(), 'raw_hi': safe_hi.tolist(),
           'rad_lo': lo_rad.tolist(), 'rad_hi': hi_rad.tolist(),
           'zero_raw': zero.tolist(), 'direction': direction.tolist(),
           'names': names,
           'note': 'driver限位 ∩ 舵机EEPROM限位；命令必须落在此区间内'},
          open(OUT, 'w'), indent=2)
print(f'\n已写 {OUT}')
print('安全弧度限位:')
for n, a, b in zip(names, lo_rad, hi_rad):
    print(f'  {n:>15}: {a:+.4f} .. {b:+.4f}')
