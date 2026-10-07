#!/bin/bash
# USB 掉线诊断 + 尝试软件恢复 cdc_acm 串口
echo "=== 最新 dmesg (USB) ==="
sudo dmesg 2>/dev/null | grep -iE 'usb|acm|video' | tail -15
echo
echo "=== 串口设备现状 ==="
ls -l /dev/ttyACM0 2>&1
echo
echo "=== 尝试 unbind/rebind cdc_acm (1-1:1.0) ==="
DRV=/sys/bus/usb/drivers/cdc_acm
DEV=/sys/bus/usb/devices/1-1:1.0
if [ -e "$DEV" ]; then
  echo "设备存在: $DEV"
  if [ -e "$DEV/driver" ]; then
    sudo sh -c "echo -n '1-1:1.0' > $DRV/unbind" && echo "已 unbind"
  fi
  sudo sh -c "echo -n '1-1:1.0' > $DRV/bind" && echo "已 rebind"
  sleep 2
  ls -l /dev/ttyACM0 2>&1
else
  echo "1-1:1.0 不存在，列出当前 usb 设备："
  ls /sys/bus/usb/devices/ | grep -E '^[0-9]-'
fi
echo
echo "=== 摄像头 ==="
ls /dev/video* 2>&1
exit 0
