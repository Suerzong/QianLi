#!/usr/bin/env python3
"""手动扫行程 → 测出每个关节的真实机械限位（只读，扭矩必须为 0）

为什么必须实测
--------------
真机 `driver_params.yaml` 里的 ``zero_raw/raw_min/raw_max`` 并不是机械行程：

* 2026-09-26 标定留下的 lerobot 文件 ``my_so101_arm.json`` 里，
  shoulder_pan 记的是 ``826~3330`` —— 和 URDF 限位换算值一模一样，
  说明当时是"照着 URDF 摆位"录的，不是推到机械死点录的；
  wrist_roll 直接是 ``0~4095``（没测，取默认）。
* 于是 elbow_flex / wrist_roll 的正向行程还要再被 12 位 ``4095`` 截断。
* 舵机 EEPROM 的 Min/Max_Position_Limit 与当前零位不自洽
  （elbow 当前 4061 远超它自己的 max 3069 也不报错），不能当机械限位用。

所以只剩一个可靠办法：**断电扭矩 = 0，用手把每个关节推到两个机械死点，
同时以高频记录 Present_Position 的极值** —— 这正是 lerobot-calibrate 的做法。

本脚本只发 READ(0x02)，绝不写寄存器、绝不使能力矩。若检测到任何舵机
``torque_enable != 0`` 立即退出（那说明手推不动，也不该由本脚本去卸力）。

用法::

    ~/mj/bin/python joint_range_calibrate.py --duration 120
    ~/mj/bin/python joint_range_calibrate.py --duration 120 --json /tmp/joint_ranges.json
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time

import serial

PORT = '/dev/ttyACM0'
BAUD = 1_000_000
JOINT_NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
               'wrist_roll', 'gripper']
SERVO_IDS = [1, 2, 3, 4, 5, 6]

REG_TORQUE_ENABLE = 40
REG_PRESENT_POSITION = 56
REG_PRESENT_LOAD = 60
REG_PRESENT_VOLTAGE = 62
REG_PRESENT_TEMPERATURE = 63
REG_STATUS = 65

RAW_PER_REV = 4096.0
DEG_PER_COUNT = 360.0 / RAW_PER_REV
RAD_PER_COUNT = math.tau / RAW_PER_REV

# 当前生效的配置（driver_params.yaml，2026-10-06）
CONFIGURED = {
    'zero_raw': [2078, 1980, 3076, 2035, 3053, 2030],
    'raw_min':  [826, 842, 1974, 954, 1264, 1916],
    'raw_max':  [3330, 3118, 4095, 3116, 4095, 3168],
}
# URDF（官方 so101_new_calib.urdf，本仓库 urdf/so101.urdf 完全一致）
URDF_LIMITS = {
    'shoulder_pan':  (-1.91986, 1.91986),
    'shoulder_lift': (-1.74533, 1.74533),
    'elbow_flex':    (-1.69, 1.69),
    'wrist_flex':    (-1.65806, 1.65806),
    'wrist_roll':    (-2.74385, 2.84121),
    'gripper':       (-0.174533, 1.74533),
}
PROGRESS_PATH = '/tmp/joint_range_progress.txt'
STOP_FLAG = '/tmp/joint_range_stop'


def checksum(bs):
    return (~sum(bs)) & 0xFF


def read_reg(ser, sid, addr, length, attempts=2):
    body = [sid, 4, 0x02, addr, length]
    pkt = bytes([0xFF, 0xFF] + body + [checksum(body)])
    for _ in range(attempts):
        ser.reset_input_buffer()
        ser.write(pkt)
        ser.flush()
        time.sleep(0.004)
        resp = ser.read(2 + 1 + 1 + 1 + length + 1)
        if len(resp) >= 6 + length and resp[0] == 0xFF and resp[1] == 0xFF:
            if resp[:len(pkt)] == pkt and len(resp) == len(pkt):
                continue
            return resp[4], resp[5:5 + length]
    return None, None


def read_u16(ser, sid, addr):
    err, data = read_reg(ser, sid, addr, 2)
    if data is None:
        return None, None
    return err, data[0] | (data[1] << 8)


def read_u8(ser, sid, addr):
    err, data = read_reg(ser, sid, addr, 1)
    if data is None:
        return None, None
    return err, data[0]


class Unwrapper:
    """把每圈 4096 计数的读数展开成连续值（处理 0/4095 跨界与多圈）。"""

    def __init__(self):
        self.prev = None
        self.turns = 0
        self.lo = None
        self.hi = None
        self.samples = 0

    def feed(self, raw_u16):
        w = raw_u16 % 4096
        if self.prev is not None:
            delta = w - self.prev
            if delta > 2048:
                self.turns -= 1
            elif delta < -2048:
                self.turns += 1
        self.prev = w
        value = self.turns * 4096 + w
        self.samples += 1
        if self.lo is None or value < self.lo:
            self.lo = value
        if self.hi is None or value > self.hi:
            self.hi = value
        return value

    @property
    def span(self):
        if self.lo is None:
            return 0
        return self.hi - self.lo


def write_progress(lines):
    tmp = PROGRESS_PATH + f'.tmp{os.getpid()}'
    try:
        with open(tmp, 'w') as fh:
            fh.write('\n'.join(lines) + '\n')
        os.replace(tmp, PROGRESS_PATH)
    except OSError:
        pass


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=PORT)
    ap.add_argument('--baud', type=int, default=BAUD)
    ap.add_argument('--duration', type=float, default=120.0,
                    help='采样时长（秒）')
    ap.add_argument('--start-delay', type=float, default=8.0,
                    help='开始采样前的准备时间（秒）')
    ap.add_argument('--json', default='/tmp/joint_ranges.json')
    args = ap.parse_args()

    for path in (PROGRESS_PATH, STOP_FLAG):
        if os.path.exists(path):
            os.remove(path)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2, exclusive=True)
    except Exception as exc:  # noqa: BLE001
        print(f'❌ 打不开 {args.port}（driver 还在占用？）: {exc}')
        return 1
    time.sleep(0.2)

    # ---- 安全闸门 1：扭矩必须全部为 0，否则手推不动 ----
    torque = []
    for sid in SERVO_IDS:
        _, value = read_u8(ser, sid, REG_TORQUE_ENABLE)
        torque.append(value)
    print(f'扭矩状态: {torque}')
    if any(v is None for v in torque):
        print('❌ 有舵机无应答，先排查供电/串口，再重试')
        ser.close()
        return 1
    if any(v != 0 for v in torque):
        print('❌ 检测到转矩已使能 —— 手推不动，且本脚本绝不去卸力。')
        print('   请先 `/arm/stop` 或在驱动里停掉，并支撑好机械臂，再重试。')
        ser.close()
        return 1

    unwrappers = [Unwrapper() for _ in SERVO_IDS]
    errors_seen = {name: [] for name in JOINT_NAMES}
    status_seen = {name: [] for name in JOINT_NAMES}

    print()
    print('=' * 68)
    print('  手动扫行程：把 6 个关节都推到两个机械死点')
    print('=' * 68)
    print(f'  {args.start_delay:.0f} 秒后开始采样，持续 {args.duration:.0f} 秒。')
    print('  · 轻推到"推不动"为止，在每个死点停 ~2 秒')
    print('  · 肩部/肘部请用另一只手托住，避免松手砸到桌面')
    print('  · 顺序随意，但每个关节两个方向都要走到')
    print('  · 中途想停：在虚拟机里执行  touch /tmp/joint_range_stop')
    print('=' * 68)
    print(flush=True)

    deadline = time.monotonic() + args.start_delay
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        print(f'\r  准备中… {remaining:4.1f}s ', end='', flush=True)
        time.sleep(0.2)
    print('\r  开始采样！        ')

    stop_requested = {'value': False}

    def _on_signal(_signum, _frame):
        stop_requested['value'] = True

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    t_end = time.monotonic() + args.duration
    t_next_report = time.monotonic() + 2.0
    samples = 0
    read_failures = 0

    while time.monotonic() < t_end:
        if stop_requested['value'] or os.path.exists(STOP_FLAG):
            break
        values = []
        for sid in SERVO_IDS:
            err, raw = read_u16(ser, sid, REG_PRESENT_POSITION)
            if raw is None:
                read_failures += 1
                values.append(None)
                continue
            values.append(raw)
        for i, raw in enumerate(values):
            if raw is None:
                continue
            unwrappers[i].feed(raw)
            name = JOINT_NAMES[i]
            if err:
                if not errors_seen[name] or errors_seen[name][-1] != err:
                    errors_seen[name].append(err)
        samples += 1

        now = time.monotonic()
        if now >= t_next_report:
            t_next_report = now + 1.0
            lines = [f'采样 {samples} 次  剩余 {t_end - now:5.1f}s  '
                     f'读失败 {read_failures}']
            for name, uw in zip(JOINT_NAMES, unwrappers):
                if uw.lo is None:
                    lines.append(f'  {name:14s} 无数据')
                    continue
                lines.append(
                    f'  {name:14s} 极值 [{uw.lo:6d}, {uw.hi:6d}]  '
                    f'行程 {uw.span:5d} ({uw.span * DEG_PER_COUNT:6.1f}°)')
            write_progress(lines)

    # 收尾：再读一轮状态
    for i, sid in enumerate(SERVO_IDS):
        _, st = read_u8(ser, sid, REG_STATUS)
        if st is not None:
            status_seen[JOINT_NAMES[i]].append(st)
    voltage = {}
    temperature = {}
    load = {}
    for name, sid in zip(JOINT_NAMES, SERVO_IDS):
        _, v = read_u8(ser, sid, REG_PRESENT_VOLTAGE)
        _, t = read_u8(ser, sid, REG_PRESENT_TEMPERATURE)
        _, l = read_reg(ser, sid, REG_PRESENT_LOAD, 2)
        voltage[name] = None if v is None else v / 10.0
        temperature[name] = t
        load[name] = None if l is None else int.from_bytes(l, 'little')
    ser.close()

    write_progress(['采样结束'])
    if os.path.exists(STOP_FLAG):
        os.remove(STOP_FLAG)

    # ------------------------- 汇总 -------------------------
    print()
    print('=' * 78)
    print('  实测结果')
    print('=' * 78)
    header = (f'{"关节":<14}{"实测最负":>9}{"实测最正":>9}{"行程":>8}{"行程°":>8}'
              f'{"中点":>8}{"配置零点":>9}{"零点差":>8}{"零点差°":>9}')
    print(header)
    print('-' * 78)

    report = {'read_at': time.strftime('%Y-%m-%d %H:%M:%S'),
              'port': args.port, 'duration_s': args.duration,
              'samples': samples, 'read_failures': read_failures,
              'joints': {}}

    for i, name in enumerate(JOINT_NAMES):
        uw = unwrappers[i]
        lo, hi = uw.lo, uw.hi
        span = uw.span
        mid = None if lo is None else (lo + hi) / 2.0
        zero = CONFIGURED['zero_raw'][i]
        dmid = None if mid is None else mid - zero
        entry = {
            'measured_min_unwrapped': lo,
            'measured_max_unwrapped': hi,
            'span_counts': span,
            'span_deg': span * DEG_PER_COUNT,
            'midpoint': mid,
            'configured_zero_raw': zero,
            'midpoint_minus_zero': dmid,
            'samples': uw.samples,
            'status_seen': status_seen[name],
            'error_flags_seen': errors_seen[name],
            'voltage': voltage[name],
            'temperature_c': temperature[name],
            'final_load': load[name],
            'urdf_lower_rad': URDF_LIMITS[name][0],
            'urdf_upper_rad': URDF_LIMITS[name][1],
        }
        report['joints'][name] = entry
        if lo is None:
            print(f'{name:<14}{"无数据":>9}')
            continue
        print(f'{name:<14}{lo:>9d}{hi:>9d}{span:>8d}'
              f'{span * DEG_PER_COUNT:>8.1f}{mid:>8.0f}{zero:>9d}'
              f'{dmid:>+8.0f}{dmid * DEG_PER_COUNT:>+9.1f}')

    print()
    print('=' * 78)
    print('  与 URDF / 现有软限位对照')
    print('=' * 78)
    for i, name in enumerate(JOINT_NAMES):
        uw = unwrappers[i]
        if uw.lo is None or uw.span < 8:
            print(f'── {name}: 实测行程仅 {uw.span} 计数 —— 这个关节没被扫到，需重测')
            continue
        urdf_lo, urdf_hi = URDF_LIMITS[name]
        need_lo = urdf_lo * RAW_PER_REV / math.tau
        need_hi = urdf_hi * RAW_PER_REV / math.tau
        cfg_lo = CONFIGURED['raw_min'][i]
        cfg_hi = CONFIGURED['raw_max'][i]
        zero = CONFIGURED['zero_raw'][i]
        cfg_span = cfg_hi - cfg_lo
        print(f'── {name}')
        print(f'   实测行程          {uw.span:5d} 计数 = {uw.span * DEG_PER_COUNT:6.1f}°')
        print(f'   现软限位行程      {cfg_span:5d} 计数 = {cfg_span * DEG_PER_COUNT:6.1f}°'
              f'   → 比实测{"窄" if cfg_span < uw.span else "宽"} '
              f'{abs(cfg_span - uw.span) * DEG_PER_COUNT:5.1f}°')
        print(f'   URDF 要求行程     {need_hi - need_lo:5.0f} 计数 = '
              f'{(need_hi - need_lo) * DEG_PER_COUNT:6.1f}°')
        # URDF 窗口能否装进 12 位
        fit_lo = zero + need_lo
        fit_hi = zero + need_hi
        fits = 0 <= fit_lo and fit_hi <= 4095
        print(f'   以当前零点 {zero} 换算 URDF 窗口 = [{fit_lo:.0f}, {fit_hi:.0f}]'
              f'  → {"✅ 装得下" if fits else "❌ 超出 0..4095，会被静默截断"}')
        if not fits:
            safe_lo = math.ceil(-need_lo)
            safe_hi = math.floor(4095 - need_hi)
            print(f'      要装下，零点必须落在 [{safe_lo}, {safe_hi}] —— '
                  f'当前 {zero} 不在其中')
        # 用实测中点当零点时的建议值
        mid = (uw.lo + uw.hi) / 2.0
        sug_lo = round(mid + need_lo)
        sug_hi = round(mid + need_hi)
        sug_fits = 0 <= sug_lo and sug_hi <= 4095
        print(f'   若把零点改成实测中点 {mid:.0f}: 建议 raw 限位 = '
              f'[{sug_lo}, {sug_hi}]  → {"✅" if sug_fits else "❌ 仍超 12 位"}')
        report['joints'][name].update({
            'suggested_zero_raw': round(mid),
            'suggested_raw_min': sug_lo,
            'suggested_raw_max': sug_hi,
            'urdf_window_with_current_zero': [fit_lo, fit_hi],
            'urdf_window_fits_12bit_with_current_zero': bool(fits),
        })
        print()

    with open(args.json, 'w') as fh:
        json.dump(report, fh, indent=2, ensure_ascii=False)
    print(f'📄 报告已写入 {args.json}')
    print(f'📄 进度文件 {PROGRESS_PATH}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
