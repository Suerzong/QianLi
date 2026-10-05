#!/usr/bin/env python3
"""机械臂串口诊断：写测试 + Feetech 协议 ping 舵机

SO-101 舵机（Feetech STS3215）协议：
  FF FF ID LEN INSTR PARAMS... CHECKSUM
  Ping 指令 = 0x01，CHECKSUM = ~(ID+LEN+INSTR) & 0xFF
  例如 ID=1: FF FF 01 02 01 FB

诊断项：
  1. 能否打开 /dev/ttyACM0 @1000000
  2. write() 是否超时（关键：区分"USB 卡死" vs "舵机不应答"）
  3. ping ID 1~6，看是否有舵机响应（有响应=舵机已上电）
"""

import sys
import time

import serial

PORT = '/dev/ttyACM0'
BAUD = 1000000


def ping_packet(sid):
    body = bytes([sid, 0x02, 0x01])
    chk = (~sum(body)) & 0xFF
    return b'\xff\xff' + body + bytes([chk])


print(f'=== 1. 打开 {PORT} @ {BAUD} ===')
try:
    s = serial.Serial(PORT, BAUD, timeout=0.3, write_timeout=0.5)
    print(f'  ✅ 打开成功 (is_open={s.is_open})')
except Exception as e:
    print(f'  ❌ 打开失败: {e}')
    sys.exit(1)

print('\n=== 2. 写入测试（关键：write 是否超时）===')
try:
    t0 = time.time()
    n = s.write(b'\x00' * 8)
    dt = (time.time() - t0) * 1000
    print(f'  ✅ 写入 {n} 字节，耗时 {dt:.1f} ms')
except Exception as e:
    print(f'  ❌ 写入失败/超时: {e}')
    print('  → 说明 USB CDC 端点卡死，需要重新插拔 USB 或复位设备')

print('\n=== 3. 清空缓冲并 ping 舵机 ID 1~6 ===')
s.reset_input_buffer()
found = []
for sid in range(1, 7):
    try:
        s.reset_input_buffer()
        s.write(ping_packet(sid))
        time.sleep(0.05)
        resp = s.read(32)
        if resp:
            print(f'  ID {sid}: 响应 {resp.hex(" ")}')
            found.append(sid)
        else:
            print(f'  ID {sid}: 无响应')
    except Exception as e:
        print(f'  ID {sid}: 异常 {e}')
        break

print()
if found:
    print(f'✅ 舵机在线: {found}（串口链路正常，舵机已上电）')
else:
    print('❌ 无任何舵机响应。可能原因：')
    print('   a) 机械臂电源没开（舵机未上电）')
    print('   b) 舵机 ID 不是 1~6')
    print('   c) 波特率不对（默认 1000000）')
    print('   d) USB 链路卡死 → 重新插拔 USB 并重启 driver')

s.close()
