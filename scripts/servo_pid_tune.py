#!/usr/bin/env python3
"""安全地改 P/D/I 增益（寄存器 21/22/23，REG_LOCK=55）。

流程与 so101_bringup/tools/tune_servo_pid.py 一致：
捕获当前目标位置 -> 短暂关力矩 -> 解锁 EEPROM -> 写增益 -> 锁定
-> 恢复目标与力矩 -> 校验读回。

用法: servo_pid_tune.py --p 32 --d 64 --i 0 --ids 1 2 3 4 5
"""
import argparse
import sys
import time

sys.path.insert(0, '/home/ros/legacy/arm/arm-final/ros2_ws/src/so101_bringup')
from so101_bringup.servo_protocol import FeetechSerialBus

REG_P, REG_D, REG_I, REG_LOCK = 21, 22, 23, 55


def rd(fn, n=8, tag=''):
    last = None
    for _ in range(n):
        try:
            return fn()
        except Exception as exc:
            last = exc
            time.sleep(0.2)
    raise RuntimeError(f'{tag}: {last}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--p', type=int, required=True)
    ap.add_argument('--d', type=int, default=None)
    ap.add_argument('--i', type=int, default=None)
    ap.add_argument('--ids', type=int, nargs='+', required=True)
    a = ap.parse_args()
    if not 0 <= a.p <= 255:
        raise SystemExit('p out of range')
    if a.d is not None and not 0 <= a.d <= 255:
        raise SystemExit('d out of range')
    if a.i is not None and not 0 <= a.i <= 255:
        raise SystemExit('i out of range')

    bus = FeetechSerialBus('/dev/ttyACM0', timeout_s=0.08)
    ids = tuple(a.ids)
    try:
        captured = rd(bus.read_positions, tag='读位置')
        before = {}
        for sid in ids:
            before[sid] = (rd(lambda s=sid: bus.read_byte(s, REG_P), tag='读P'),
                           rd(lambda s=sid: bus.read_byte(s, REG_D), tag='读D'),
                           rd(lambda s=sid: bus.read_byte(s, REG_I), tag='读I'))
        rd(lambda: bus.write_positions(captured), n=3, tag='写目标')
        bus.set_torque(False)
        time.sleep(0.05)
        bus.sync_write_register(REG_LOCK, 1, ids, bytes([0] * len(ids)))
        time.sleep(0.02)
        bus.sync_write_register(REG_P, 1, ids, bytes([a.p] * len(ids)))
        if a.d is not None:
            bus.sync_write_register(REG_D, 1, ids, bytes([a.d] * len(ids)))
        if a.i is not None:
            bus.sync_write_register(REG_I, 1, ids, bytes([a.i] * len(ids)))
        time.sleep(0.04)
        bus.sync_write_register(REG_LOCK, 1, ids, bytes([1] * len(ids)))
        time.sleep(0.04)
        after = {}
        for sid in ids:
            after[sid] = (bus.read_byte(sid, REG_P), bus.read_byte(sid, REG_D),
                          bus.read_byte(sid, REG_I))
        ok = all(v[0] == a.p and (a.d is None or v[1] == a.d)
                 and (a.i is None or v[2] == a.i) for v in after.values())
        bus.write_positions(captured)
        bus.set_torque(True)
        tor = rd(bus.read_torque_states, tag='读力矩')
        print('before:', before)
        print('after :', after)
        print('torque:', tor)
        print('RESULT:', 'OK' if ok and all(t == 1 for t in tor) else 'FAIL')
        if not ok:
            raise SystemExit(1)
    finally:
        bus.close()


if __name__ == '__main__':
    main()
