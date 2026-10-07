#!/usr/bin/env python3
"""机械臂 USB 串口看门狗：检测持续 Write timeout 并自动 USB 复位

背景：
  SO-101 的串口芯片 CH343 (1a86:55d3) 在 VM 里走通用 cdc_acm 驱动，
  USB CDC 端点会偶发假死（写超时）。偶发时 driver 自己能恢复；
  但持续假死时 driver 的重试循环救不回来，必须 USB 复位。
  已验证：USB 复位 + 驱动重绑后，driver 会**自动重连**（无需重启 launch）。

策略：
  每 poll_s 秒检查 driver 日志新增内容，统计最近 window 内的失败次数；
  >= threshold 判定持续故障 → 执行复位；限流 min_interval 秒。

用法（需要免密 sudo）：
  python3 arm_usb_watchdog.py            # 前台运行
  setsid python3 arm_usb_watchdog.py &   # 后台运行
"""

import argparse
import os
import re
import subprocess
import time
import sys
from pathlib import Path
from project_paths import default_arm_port

LOG = '/tmp/ik_demo.log'
WATCH_LOG = '/tmp/arm_watchdog.log'
PORT = default_arm_port()
USBDEVFS_RESET = 0x5514

POLL_S = 8            # 检查间隔
FAIL_PATTERN = re.compile(r'Write timeout|state read failed|reconnect pending')
FAIL_THRESHOLD = 3    # 连续失败次数阈值
MIN_INTERVAL_S = 60   # 两次复位最小间隔


def discover_usb_device(port=PORT, sys_root=Path('/sys'), dev_root=Path('/dev')):
    """Discover the USB device behind this serial port; never reset another chip."""
    tty = Path(port).resolve().name
    path = (Path(sys_root) / 'class' / 'tty' / tty / 'device').resolve()
    interface = None
    for parent in (path, *path.parents):
        if (parent / 'bInterfaceNumber').is_file():
            interface = parent
        if (parent / 'idVendor').is_file():
            try:
                vendor = (parent / 'idVendor').read_text().strip().lower()
                product = (parent / 'idProduct').read_text().strip().lower()
                if (vendor, product) != ('1a86', '55d3') or interface is None:
                    return None
                bus = int((parent / 'busnum').read_text().strip())
                device = int((parent / 'devnum').read_text().strip())
                if not (1 <= bus <= 999 and 1 <= device <= 999):
                    return None
                driver = (interface / 'driver').resolve()
                expected = (Path(sys_root) / 'bus' / 'usb' / 'drivers').resolve()
                if driver.parent != expected:
                    return None
                return {'node': Path(dev_root) / 'bus' / 'usb' / f'{bus:03d}' / f'{device:03d}',
                        'interface': interface.name, 'driver': driver}
            except (OSError, ValueError):
                return None
    return None


def find_usb_node():
    info = discover_usb_device(PORT)
    return str(info['node']) if info and info['node'].exists() else None


def log(msg):
    line = f'[{time.strftime("%H:%M:%S")}] {msg}'
    print(line, flush=True)
    try:
        with open(WATCH_LOG, 'a') as f:
            f.write(line + '\n')
    except OSError:
        pass


def usb_reset():
    """Reset only the configured CH343P adapter and its discovered interface."""
    info = discover_usb_device(PORT)
    if not info:
        log(f'找不到端口 {PORT} 对应的 CH343P USB 接口，拒绝复位')
        return False
    ok = True
    node = info['node']
    if node.exists():
        try:
            subprocess.run(
                ['sudo', '-n', sys.executable, '-c',
                 "import fcntl,sys; f=open(sys.argv[1],'wb'); fcntl.ioctl(f,0x5514,0); f.close()",
                 str(node)], check=True, capture_output=True, timeout=15)
            log(f'USB 复位已发送 ({node})')
        except (OSError, subprocess.SubprocessError) as exc:
            log(f'USB 复位失败: {exc}')
            ok = False
    for action in ('unbind', 'bind'):
        try:
            subprocess.run(['sudo', '-n', 'tee', str(info['driver'] / action)],
                           input=(info['interface']+'\n').encode(),
                           stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                           check=True, timeout=15)
            log(f"驱动 {action} 完成 ({info['interface']})")
        except (OSError, subprocess.SubprocessError) as exc:
            log(f'驱动 {action} 失败: {exc}')
            ok = False
        time.sleep(1)
    return ok


def main():
    global PORT, LOG
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--log', default=LOG)
    args = parser.parse_args()
    PORT, LOG = args.port, args.log
    log(f'看门狗启动：监控 {LOG}（阈值 {FAIL_THRESHOLD} 次 / '
        f'{POLL_S}s 间隔）')
    pos = 0
    fails = 0
    last_reset = 0.0
    # 从头开始读（只看新增）
    if os.path.exists(LOG):
        pos = os.path.getsize(LOG)

    while True:
        time.sleep(POLL_S)
        try:
            size = os.path.getsize(LOG)
            if size < pos:      # 日志被轮转/重建
                pos = 0
            if size > pos:
                with open(LOG, errors='ignore') as f:
                    f.seek(pos)
                    new = f.read()
                    pos = size
                hits = len(FAIL_PATTERN.findall(new))
                if hits:
                    fails += hits
                    log(f'检测到 {hits} 次串口异常（累计 {fails}）')
                else:
                    fails = max(0, fails - 1)
        except OSError:
            continue

        if fails >= FAIL_THRESHOLD:
            now = time.time()
            if now - last_reset < MIN_INTERVAL_S:
                log('处于复位限流期，等待 driver 自行重连')
                fails = 0
                continue
            log(f'串口持续故障（{fails} 次）→ 执行 USB 复位')
            usb_reset()
            last_reset = time.time()
            fails = 0
            # 给 driver 时间重连
            time.sleep(35)
            tail = ''
            try:
                with open(LOG, errors='ignore') as f:
                    f.seek(max(0, os.path.getsize(LOG) - 3000))
                    tail = f.read()
                pos = os.path.getsize(LOG)
            except OSError:
                pass
            if 'Connected to six servos' in tail:
                log('✅ driver 已自动重连（六舵机）')
            else:
                log('⚠️ 未见重连日志，请检查机械臂电源/USB 线')


if __name__ == '__main__':
    main()
