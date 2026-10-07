#!/usr/bin/env python3
"""把关节行程挪进舵机 12 位量程：改写 Homing_Offset（带备份、闭环校验、可回滚）

问题
----
`wrist_roll` 实测机械行程 raw ``[1144, 4993]``（338.3°），跨过了 4095 单圈边界。
只要零点在 ``zero_raw=3053``，URDF 要求的窗口 ``[1264, 4905]`` 就有 810 计数
（71.2°）落在 12 位量程外 —— 旧代码直接把 ``raw_max`` 截到 4095，
那 71.2° 就被**静默吞掉**了。

正确做法是把整段行程平移进量程，而不是截断行程。lerobot 的
``set_half_turn_homings()`` 就是这么做的。

编码（实测确定，别想当然）
--------------------------
Feetech 的 ``Homing_Offset`` **写入**用 12 位"符号+幅值"（符号位 = bit 11），
见 lerobot ``feetech/tables.py``::

    STS_SMS_SERIES_ENCODINGS_TABLE = { "Homing_Offset": 11 }

但本机固件**回读**回来的却是 16 位二进制补码。两者不对称，所以：

* 写：``encode_sign_magnitude(v, 11)``
* 读：按二进制补码解读
* **判定成功与否只看 ``Present_Position``**，不信回读的 offset
  （实测关系为 ``Present = Actual_Position - Homing_Offset``，Actual 恒定）

安全性
------
* 只写 EEPROM 的 Homing_Offset(addr 31)，**不使能扭矩、不产生任何运动**；
* 开始前记录原 offset 与 Present_Position，写入后逐个校验；
* 迭代闭环：偏差大了就按实测增益修正，最多 4 次；
* 任何一步失败自动按原值回滚并再次校验；
* ``--restore`` 可从备份文件一键还原。

用法::

    # 只读：看清当前状态，不写任何 EEPROM
    ~/mj/bin/python servo_homing_shift.py --joint wrist_roll --shift -1020 --dry-run

    # 执行
    ~/mj/bin/python servo_homing_shift.py --joint wrist_roll --shift -1020 \
        --backup /tmp/homing_backup.json

    # 回滚
    ~/mj/bin/python servo_homing_shift.py --restore /tmp/homing_backup.json
"""

from __future__ import annotations

from project_paths import default_arm_port

from project_paths import calibration_path

import argparse
import json
import os
import sys
import time

import serial

PORT = default_arm_port()
BAUD = 1_000_000
JOINT_NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
               'wrist_roll', 'gripper']

REG_TORQUE_ENABLE = 40
REG_HOMING_OFFSET = 31
REG_PRESENT_POSITION = 56
REG_LOCK = 55
SIGN_BIT = 11        # lerobot: STS_SMS_SERIES_ENCODINGS_TABLE["Homing_Offset"]


# ---- lerobot 的同名实现（src/lerobot/motors/encoding_utils.py）----------
def encode_sign_magnitude(value: int, sign_bit_index: int) -> int:
    max_magnitude = (1 << sign_bit_index) - 1
    magnitude = abs(value)
    if magnitude > max_magnitude:
        raise ValueError(
            f'magnitude {magnitude} exceeds {max_magnitude} for sign bit '
            f'{sign_bit_index}')
    direction_bit = 1 if value < 0 else 0
    return (direction_bit << sign_bit_index) | magnitude


def decode_twos_complement(value: int, n_bytes: int) -> int:
    bits = n_bytes * 8
    if value & (1 << (bits - 1)):
        value -= 1 << bits
    return value


# ------------------------------------------------------------------------
def checksum(bs):
    return (~sum(bs)) & 0xFF


def read_reg(ser, sid, addr, length, attempts=4):
    body = [sid, 4, 0x02, addr, length]
    pkt = bytes([0xFF, 0xFF] + body + [checksum(body)])
    for _ in range(attempts):
        ser.reset_input_buffer()
        ser.write(pkt)
        ser.flush()
        time.sleep(0.008)
        resp = ser.read(2 + 1 + 1 + 1 + length + 1)
        if len(resp) >= 6 + length and resp[0] == 0xFF and resp[1] == 0xFF:
            if resp[:len(pkt)] == pkt and len(resp) == len(pkt):
                continue        # 回显帧，重读
            return resp[4], resp[5:5 + length]
    return None, None


def write_offset_field(ser, sid, field):
    """写 Homing_Offset：字段本身就是符号+幅值编码后的值。"""
    body = [sid, 5, 0x03, REG_HOMING_OFFSET,
            field & 0xFF, (field >> 8) & 0xFF]
    ser.write(bytes([0xFF, 0xFF] + body + [checksum(body)]))
    ser.flush()
    time.sleep(0.25)            # EEPROM 写入留足时间


def read_offset_semantic(ser, sid, samples=5):
    """回读 offset，按二进制补码解读（本机固件的回读格式）。"""
    values = []
    for _ in range(samples):
        err, data = read_reg(ser, sid, REG_HOMING_OFFSET, 2)
        if data is not None:
            values.append(decode_twos_complement(
                data[0] | (data[1] << 8), 2))
        time.sleep(0.02)
    if not values:
        return None
    values.sort()
    return values[len(values) // 2]


def read_present(ser, sid, samples=9):
    values = []
    for _ in range(samples):
        err, data = read_reg(ser, sid, REG_PRESENT_POSITION, 2)
        if data is not None and not err:
            value = data[0] | (data[1] << 8)
            values.append(decode_twos_complement(value, 2))
        time.sleep(0.012)
    if not values:
        return None
    values.sort()
    return values[len(values) // 2]


def wrap_delta(delta, modulus=4096):
    return (delta + modulus // 2) % modulus - modulus // 2


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=PORT)
    ap.add_argument('--baud', type=int, default=BAUD)
    ap.add_argument('--joint')
    ap.add_argument('--shift', type=float,
                    help='Present_Position 的目标位移（计数）')
    ap.add_argument('--backup', default=calibration_path('homing_backup.json'))
    ap.add_argument('--restore')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2, exclusive=True)
    except Exception as exc:  # noqa: BLE001
        print(f'❌ 打不开 {args.port}（driver 还在占用？）: {exc}')
        return 1
    time.sleep(0.2)

    def torque_states():
        out = []
        for sid in range(1, 7):
            _, data = read_reg(ser, sid, REG_TORQUE_ENABLE, 1)
            out.append(None if data is None else data[0])
        return out

    torque = torque_states()
    print(f'扭矩状态: {torque}')
    if any(v is None for v in torque):
        print('❌ 有舵机无应答，先排查供电/串口')
        ser.close()
        return 1
    if any(v != 0 for v in torque):
        print('❌ 有舵机扭矩使能中 —— 会产生运动，拒绝写 EEPROM。先停掉驱动。')
        ser.close()
        return 1

    # ------------------------- 回滚 -------------------------
    if args.restore:
        with open(args.restore) as fh:
            saved = json.load(fh)
        print(f'↩️  回滚 {args.restore}')
        ok = True
        for name, entry in saved.items():
            sid = JOINT_NAMES.index(name) + 1
            present_before = read_present(ser, sid)
            target = entry['offset_semantic']
            field = encode_sign_magnitude(target, SIGN_BIT)
            write_offset_field(ser, sid, field)
            present_after = read_present(ser, sid)
            back = read_offset_semantic(ser, sid)
            good = (present_after == entry['present_before']
                    and back == target)
            ok &= good
            print(f'  ID{sid} {name:<14} 写 sm({target})=0x{field:03X}  '
                  f'回读 {back}  Present {present_before} → {present_after}  '
                  f'{"✅" if good else "❌ 期望回读 " + str(target) + " / Present " + str(entry["present_before"])}')
        ser.close()
        return 0 if ok else 1

    if not args.joint or args.shift is None:
        print('❌ 需要 --joint 和 --shift（或用 --restore）')
        ser.close()
        return 1
    if args.joint not in JOINT_NAMES:
        print(f'❌ 未知关节 {args.joint}')
        ser.close()
        return 1

    sid = JOINT_NAMES.index(args.joint) + 1
    offset0 = read_offset_semantic(ser, sid)
    present0 = read_present(ser, sid)
    lock = None
    _, data = read_reg(ser, sid, REG_LOCK, 1)
    if data is not None:
        lock = data[0]

    print(f'\n关节              {args.joint} (ID{sid})')
    print(f'Homing_Offset     {offset0}  (按补码回读)')
    print(f'Lock              {lock}')
    print(f'Present_Position  {present0}')
    print(f'目标位移          {args.shift:+.0f} 计数 '
          f'({args.shift * 360.0 / 4096:+.1f}°)')

    if offset0 is None or present0 is None:
        print('❌ 读取失败，放弃')
        ser.close()
        return 1
    if args.dry_run:
        target = int(round(offset0 - args.shift))   # Present = Actual - Offset
        print(f'\n(--dry-run) 将写 sm({target}) = '
              f'0x{encode_sign_magnitude(target, SIGN_BIT):03X}，'
              f'期望 Present = {present0 + args.shift:.0f}')
        ser.close()
        return 0

    backup = {}
    if os.path.exists(args.backup):
        with open(args.backup) as fh:
            backup = json.load(fh)
    backup[args.joint] = {
        'offset_semantic': offset0,
        'present_before': present0,
        'shift_target': args.shift,
        'saved_at': time.strftime('%Y-%m-%d %H:%M:%S'),
    }
    with open(args.backup, 'w') as fh:
        json.dump(backup, fh, indent=2, ensure_ascii=False)
    print(f'   原值已备份到 {args.backup}')

    def restore(reason):
        print(f'↩️  {reason}，正在回滚…')
        field = encode_sign_magnitude(offset0, SIGN_BIT)
        write_offset_field(ser, sid, field)
        back = read_offset_semantic(ser, sid)
        present = read_present(ser, sid)
        good = back == offset0 and present == present0
        print(f'   回滚：offset={back}（原 {offset0}） '
              f'Present={present}（原 {present0}） '
              f'{"✅ 已复原" if good else "❌ 复原失败，请手动核对"}')
        return good

    # Present = Actual - Offset  =>  目标 offset = 原 offset - 目标位移
    target_offset = offset0 - args.shift
    achieved = None
    for attempt in range(1, 4):
        try:
            field = encode_sign_magnitude(int(round(target_offset)), SIGN_BIT)
        except ValueError as exc:
            print(f'❌ {exc}')
            restore('目标 offset 无法编码')
            ser.close()
            return 1
        write_offset_field(ser, sid, field)
        back = read_offset_semantic(ser, sid)
        present = read_present(ser, sid)
        if present is None:
            restore('写入后读不到位置')
            ser.close()
            return 1
        achieved = wrap_delta(present - present0)
        print(f'   第 {attempt} 次：写 sm({int(round(target_offset))})'
              f'=0x{field:03X}  回读 {back}  '
              f'Present {present0} → {present}  实际位移 {achieved:+.0f}')
        if abs(achieved - args.shift) <= 2:
            break
        # 按实测增益修正
        gain = (achieved / (target_offset - offset0)) if target_offset != offset0 else None
        remaining = args.shift - achieved
        if gain and abs(gain) > 1e-6:
            target_offset = target_offset + remaining / gain
        else:
            target_offset = target_offset + remaining
    else:
        if not restore('迭代未收敛'):
            pass
        ser.close()
        return 1

    backup[args.joint].update({
        'new_offset_semantic': int(round(target_offset)),
        'present_after': present,
        'shift_achieved': achieved,
    })
    with open(args.backup, 'w') as fh:
        json.dump(backup, fh, indent=2, ensure_ascii=False)

    print(f'\n✅ 完成：Homing_Offset {offset0} → {int(round(target_offset))}，'
          f'Present {present0} → {present}')
    old_zero = CONFIG_ZERO_HINT.get(args.joint)
    if old_zero is not None:
        print(f'   → 该关节的 zero_raw 要从 {old_zero} 改成 '
              f'{round(old_zero + achieved)}（原零点 + 实测位移 {achieved:+.0f}）')
    print(f'   回滚：~/mj/bin/python {os.path.basename(__file__)} '
          f'--restore {args.backup}')
    ser.close()
    return 0


CONFIG_ZERO_HINT = {
    'shoulder_pan': 2078, 'shoulder_lift': 1980, 'elbow_flex': 3076,
    'wrist_flex': 2035, 'wrist_roll': 3053, 'gripper': 2030,
}


if __name__ == '__main__':
    sys.exit(main())
