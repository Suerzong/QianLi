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

import fcntl
import os
import re
import subprocess
import time

LOG = '/tmp/ik_demo.log'
WATCH_LOG = '/tmp/arm_watchdog.log'
USB_INTF = '1-1:1.0'
USBDEVFS_RESET = 0x5514

POLL_S = 8            # 检查间隔
FAIL_PATTERN = re.compile(r'Write timeout|state read failed|reconnect pending')
FAIL_THRESHOLD = 3    # 连续失败次数阈值
MIN_INTERVAL_S = 60   # 两次复位最小间隔


def find_usb_node():
    """动态查找串口设备的 /dev/bus/usb/BBB/DDD（设备号会随复位变化）。"""
    vendor = '/sys/bus/usb/devices/1-1/idVendor'
    try:
        with open(vendor) as f:
            if f.read().strip() != '1a86':
                return None
        bus = int(open('/sys/bus/usb/devices/1-1/busnum').read().strip())
        dev = int(open('/sys/bus/usb/devices/1-1/devnum').read().strip())
        path = f'/dev/bus/usb/{bus:03d}/{dev:03d}'
        return path if os.path.exists(path) else None
    except OSError:
        return None


def log(msg):
    line = f'[{time.strftime("%H:%M:%S")}] {msg}'
    print(line, flush=True)
    try:
        with open(WATCH_LOG, 'a') as f:
            f.write(line + '\n')
    except OSError:
        pass


def usb_reset():
    """USB 复位（可选）+ cdc_acm 驱动重绑（可靠兜底）。返回是否成功。"""
    ok = True
    node = find_usb_node()
    if node:
        try:
            fd = os.open(node, os.O_WRONLY)
            fcntl.ioctl(fd, USBDEVFS_RESET, 0)
            os.close(fd)
            log(f'USB 复位指令已发送 ({node})')
        except Exception as e:
            log(f'USB 复位失败 ({node}): {e}')
            ok = False
    else:
        log('未找到 USB 设备节点，跳过 ioctl 复位，直接重绑驱动')
    time.sleep(2)
    for act in ('unbind', 'bind'):
        try:
            subprocess.run(
                ['sudo', 'tee', f'/sys/bus/usb/drivers/cdc_acm/{act}'],
                input=USB_INTF.encode(), stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL, check=False)
            log(f'驱动 {act} 完成')
        except Exception as e:
            log(f'驱动 {act} 失败: {e}')
            ok = False
        time.sleep(1)
    return ok


def main():
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
