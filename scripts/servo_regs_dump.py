#!/usr/bin/env python3
"""dump 舵机 0x00-0x3F 寄存器，找 P/D/I 增益与内存表布局。

用法: servo_regs_dump.py [servo_id] [起始地址] [长度]
"""

from project_paths import default_arm_port

from project_paths import arm_source_path
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, arm_source_path())
from so101_bringup.servo_protocol import FeetechSerialBus

sid = int(sys.argv[1]) if len(sys.argv) > 1 else 1
start = int(sys.argv[2], 0) if len(sys.argv) > 2 else 0x00
length = int(sys.argv[3], 0) if len(sys.argv) > 3 else 0x40

bus = FeetechSerialBus(default_arm_port(), timeout_s=0.05)
data = bus.read_bytes(sid, start, length)
print(f'servo {sid} 寄存器 {start:#04x}..{start+length-1:#04x}:')
for i in range(0, len(data), 8):
    chunk = data[i:i + 8]
    row = '  '.join(f'{b:02x}' for b in chunk)
    addr = start + i
    desc = ''
    if addr == 0x03:
        desc = 'ID'
    elif addr == 0x06:
        desc = '? 限位低字?'
    elif addr in (0x09,):
        desc = 'MinAngleLimit'
    elif addr in (0x0B,):
        desc = 'MaxAngleLimit'
    elif addr in (0x18, 0x19, 0x1A):
        desc = 'P/D/I 系数'
    elif addr in (0x1C, 0x1D):
        desc = '速度限制?'
    elif addr == 0x28:
        desc = 'TorqueEnable(0x28)'
    elif addr == 0x2A:
        desc = 'GoalPos(0x2A)'
    elif addr == 0x38:
        desc = 'PresentPos(0x38)'
    elif addr == 0x3C:
        desc = 'PresentLoad'
    print(f'  {addr:02x}: {row}  {desc}')
bus.close()
