#!/usr/bin/env python3
"""读全部舵机当前 P/D/I 增益（寄存器 21/22/23，HX-30HM STS 兼容）。"""
import sys

sys.path.insert(0, '/home/ros/legacy/arm/arm-final/ros2_ws/src/so101_bringup')
from so101_bringup.servo_protocol import FeetechSerialBus

bus = FeetechSerialBus('/dev/ttyACM0', timeout_s=0.08)
for sid in (1, 2, 3, 4, 5, 6):
    p = bus.read_byte(sid, 21)
    d = bus.read_byte(sid, 22)
    i = bus.read_byte(sid, 23)
    print(f'servo {sid}: P={p} D={d} I={i}')
bus.close()
