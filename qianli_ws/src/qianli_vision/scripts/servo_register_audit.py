#!/usr/bin/env python3
"""只读审计 6 个舵机的寄存器 —— 机械限位修复的"真值来源"

背景
----
真机的软限位有两套互相矛盾的来源：

1. ``driver_params.yaml`` 的 ``raw_min/raw_max``
   —— 目前是从 **URDF 限位** 反算出来的（zero_raw ± URDF 角度）。
   URDF 限位是 onshape-to-robot 自动生成的语义值，**不是机械行程**。
   其中 elbow_flex / wrist_roll 的正向行程还被 4095（12 位单圈）截断，
   于是软件只允许 ~89.6° / ~91.6°，而 URDF 自己写的是 96.8° / 162.8°。
2. 舵机 EEPROM 里的 ``Min/Max_Position_Limit``（addr 9/11）
   —— 厂家预烧写的电子限位，才是"机械上允许走到哪"的硬件证据。

本脚本 **只发 READ 指令（0x02），绝不写任何寄存器**，把两套数据同时打印出来，
用于判断"软件判危险、机械还有余量"到底差在哪里。

用法（必须先停掉 driver，释放 /dev/ttyACM0）::

    ~/mj/bin/python servo_register_audit.py
    ~/mj/bin/python servo_register_audit.py --json /tmp/servo_registers.json
"""

from __future__ import annotations

from project_paths import default_arm_port

import argparse
import json
import sys
import time

import serial

PORT = default_arm_port()
BAUD = 1_000_000
JOINT_NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
               'wrist_roll', 'gripper']

# --- Feetech STS3215 / HX-30HM 寄存器表（十进制地址）----------------------
# 与 lerobot motors_bus.py 的 STS3215 表、以及本仓库 servo_protocol.py 一致。
REG = {
    'model_number':        (3,  2, False),
    'id':                  (5,  1, False),
    'baud':                (6,  1, False),
    'min_position_limit':  (9,  2, False),
    'max_position_limit':  (11, 2, False),
    'max_temperature':     (13, 1, False),
    'max_torque':          (16, 2, False),
    'p_gain':              (21, 1, False),
    'd_gain':              (22, 1, False),
    'i_gain':              (23, 1, False),
    'min_startup_force':   (24, 2, False),
    'cw_dead_band':        (26, 1, False),
    'ccw_dead_band':       (27, 1, False),
    'protection_current':  (28, 2, False),
    'angular_resolution':  (30, 1, False),
    'homing_offset':       (31, 2, True),
    'mode':                (33, 1, False),
    'protective_torque':   (34, 1, False),
    'protection_time':     (35, 1, False),
    'overload_torque':     (36, 1, False),
    'torque_enable':       (40, 1, False),
    'torque_limit':        (48, 2, False),
    'lock':                (55, 1, False),
    'present_position':    (56, 2, True),
    'present_speed':       (58, 2, True),
    'present_load':        (60, 2, True),
    'present_voltage':     (62, 1, False),
    'present_temperature': (63, 1, False),
    'status':              (65, 1, False),
    'moving':              (66, 1, False),
    'present_current':     (69, 2, True),
}

# 当前 driver_params.yaml（2026-10-06 读到的一致值）
CONFIGURED = {
    'zero_raw': [2078, 1980, 3076, 2035, 3053, 2030],
    'raw_min':  [826, 842, 1974, 954, 1264, 1916],
    'raw_max':  [3330, 3118, 4095, 3116, 4095, 3168],
}
URDF_LIMITS = {
    'shoulder_pan':  (-1.91986, 1.91986),
    'shoulder_lift': (-1.74533, 1.74533),
    'elbow_flex':    (-1.69, 1.69),
    'wrist_flex':    (-1.65806, 1.65806),
    'wrist_roll':    (-2.74385, 2.84121),
    'gripper':       (-0.174533, 1.74533),
}
RAW_PER_REV = 4096.0
DEG_PER_COUNT = 360.0 / RAW_PER_REV


def checksum(bs):
    return (~sum(bs)) & 0xFF


def read_reg(ser, sid, addr, length, attempts=3):
    """发一条 READ 指令并解析应答；返回 (error_flags, data_bytes)。"""
    body = [sid, 4, 0x02, addr, length]      # LENGTH = 参数个数(2) + 2 = 4
    pkt = bytes([0xFF, 0xFF] + body + [checksum(body)])
    for _ in range(attempts):
        ser.reset_input_buffer()
        ser.write(pkt)
        ser.flush()
        time.sleep(0.012)
        resp = ser.read(2 + 1 + 1 + 1 + length + 1)
        if len(resp) >= 6 + length and resp[0] == 0xFF and resp[1] == 0xFF:
            # CH343 半双工时可能先把指令帧回显回来，重读一次
            if resp[:len(pkt)] == pkt and len(resp) == len(pkt):
                continue
            return resp[4], resp[5:5 + length]
    return None, None


def decode(data, signed):
    if data is None:
        return None
    value = int.from_bytes(data, 'little')
    if signed and value >= 0x8000:
        value -= 0x10000
    return value


ERR_BITS = {0x01: '电压', 0x02: '角度限位', 0x04: '过热', 0x08: '编码',
            0x20: '过载'}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=PORT)
    ap.add_argument('--baud', type=int, default=BAUD)
    ap.add_argument('--json', default='/tmp/servo_registers.json')
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.25, exclusive=True)
    except Exception as exc:  # noqa: BLE001
        print(f'❌ 打不开 {args.port}（driver 还在占用？）: {exc}')
        return 1
    time.sleep(0.2)
    print(f'🔒 只读审计 {args.port} @{args.baud}（本脚本不发送任何写指令）\n')

    report = {'port': args.port, 'baud': args.baud,
              'read_at': time.strftime('%Y-%m-%d %H:%M:%S'), 'servos': {}}
    missing = []

    for sid, name in zip(range(1, 7), JOINT_NAMES):
        entry = {}
        errs = set()
        for field, (addr, size, signed) in REG.items():
            err, data = read_reg(ser, sid, addr, size)
            entry[field] = decode(data, signed)
            entry[field + '_err'] = err
            if err is not None:
                errs.add(err)
        entry['error_flags_union'] = sorted(errs)
        report['servos'][name] = entry
        if entry['present_position'] is None:
            missing.append(name)

    ser.close()

    # ---------------- 打印 ----------------
    for sid, name in zip(range(1, 7), JOINT_NAMES):
        e = report['servos'][name]
        pos = e['present_position']
        lo, hi = e['min_position_limit'], e['max_position_limit']
        zero = CONFIGURED['zero_raw'][sid - 1]
        cfg_lo, cfg_hi = CONFIGURED['raw_min'][sid - 1], CONFIGURED['raw_max'][sid - 1]
        print(f'━━━ ID{sid} {name} ━━━')
        if pos is None:
            print('   ❌ 无应答（舵机未上电 / 总线问题）')
            continue
        print(f'  型号             {e["model_number"]}')
        print(f'  Homing_Offset    {e["homing_offset"]}')
        print(f'  Mode             {e["mode"]}   MaxTorque {e["max_torque"]}   '
              f'TorqueLimit {e["torque_limit"]}   Lock {e["lock"]}')
        print(f'  PID              P={e["p_gain"]} D={e["d_gain"]} I={e["i_gain"]}   '
              f'死区 CW={e["cw_dead_band"]} CCW={e["ccw_dead_band"]}')
        print(f'  分辨率           {e["angular_resolution"]}   '
              f'保护电流 {e["protection_current"]}   '
              f'最小启动力 {e["min_startup_force"]}')
        print(f'  力矩使能         {e["torque_enable"]}   '
              f'Moving {e["moving"]}   Status {e["status"]}   '
              f'Err {e["error_flags_union"]}')
        print(f'  电压/温度        {e["present_voltage"]/10.0:.1f}V  '
              f'{e["present_temperature"]}°C')
        print(f'  当前位置         {pos}   相对 zero_raw({zero})  {pos - zero:+d} '
              f'({(pos - zero) * DEG_PER_COUNT:+.1f}°)   负载 {e["present_load"]}')
        print(f'  ── 限位对比 ──')
        if lo is not None and hi is not None:
            print(f'  舵机电子限位     [{lo}, {hi}]  '
                  f'行程 {hi - lo} ({(hi - lo) * DEG_PER_COUNT:.1f}°)'
                  f'  相对 zero: [{lo - zero:+d}, {hi - zero:+d}]')
            print(f'                   即 {(lo - zero) * DEG_PER_COUNT:+.1f}° '
                  f'~ {(hi - zero) * DEG_PER_COUNT:+.1f}°')
            print(f'  在范围内?        {lo <= pos <= hi}'
                  + ('' if lo <= pos <= hi else '   ⚠️ 当前位置已在舵机电子限位之外'))
        print(f'  软件软限位       [{cfg_lo}, {cfg_hi}]  '
              f'即 {(cfg_lo - zero) * DEG_PER_COUNT:+.1f}° ~ '
              f'{(cfg_hi - zero) * DEG_PER_COUNT:+.1f}°')
        urdf = URDF_LIMITS[name]
        print(f'  URDF 限位        {urdf[0]:+.3f} ~ {urdf[1]:+.3f} rad  '
              f'({urdf[0] * 57.2958:+.1f}° ~ {urdf[1] * 57.2958:+.1f}°)')
        if lo is not None and hi is not None:
            print(f'  软限位比舵机限位窄 '
                  f'{(cfg_lo - lo) * DEG_PER_COUNT:+.1f}° ~ '
                  f'{(cfg_hi - hi) * DEG_PER_COUNT:+.1f}°'
                  f'   （负数=软件比硬件更保守，就是"吞掉"行程的地方）')
        print()

    if missing:
        print(f'⚠️ 无应答舵机：{missing}')

    with open(args.json, 'w') as fh:
        json.dump(report, fh, indent=2, ensure_ascii=False)
    print(f'📄 报告已写入 {args.json}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
