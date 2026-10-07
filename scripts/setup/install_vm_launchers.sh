#!/usr/bin/env bash
# User desktop integration. Every automatic ROS launch uses simulation mode.
set -euo pipefail
ROOT="${QI_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)}"
mkdir -p "$HOME/Desktop" "$HOME/.local/share/applications" "$HOME/.config/qianli" "$HOME/.config/autostart"
if [[ ! -f "$HOME/.config/qianli/environment.sh" ]]; then
  cat > "$HOME/.config/qianli/environment.sh" <<'ENV'
export ROS_DOMAIN_ID=42
export QI_ARM_PORT=/dev/qianli_arm
ENV
fi
# Bind the verified external camera profile when it is already attached. Preserve
# user overrides and leave unrelated camera hardware on its own default format.
for camera in /dev/v4l/by-id/*-video-index0; do
  [[ -e "$camera" ]] || continue
  properties="$(udevadm info --query=property --name "$camera")"
  if grep -qx 'ID_VENDOR_ID=05a3' <<< "$properties" && \
      grep -qx 'ID_MODEL_ID=9230' <<< "$properties"; then
    if ! grep -q '^export QI_CAMERA=' "$HOME/.config/qianli/environment.sh"; then
      printf 'export QI_CAMERA=%q\n' "$camera" >> "$HOME/.config/qianli/environment.sh"
    fi
    if ! grep -q '^export QI_CAMERA_FOURCC=' "$HOME/.config/qianli/environment.sh"; then
      printf 'export QI_CAMERA_FOURCC=MJPG\n' >> "$HOME/.config/qianli/environment.sh"
    fi
    break
  fi
done
python3 - "$ROOT" <<'PY'
from pathlib import Path
import sys
root=Path(sys.argv[1])
home=Path.home()
entries=[('qianli-simulation','QianLi 仿真','sim','applications-engineering'),
         ('qianli-check','QianLi 完整自检','check','emblem-ok'),
         ('qianli-training','QianLi 训练自检','training-smoke','applications-science'),
         ('qianli-camera','QianLi 相机','camera','camera-photo'),
         ('qianli-devices','QianLi 只读硬件检查','devices','computer'),
         ('qianli-terminal','QianLi 终端','terminal','utilities-terminal')]
for name,title,action,icon in entries:
    document=(f'[Desktop Entry]\nType=Application\nVersion=1.0\nName={title}\n'
              f'Exec=gnome-terminal -- bash "{root}/scripts/tools/qianli-desktop.sh" {action}\n'
              f'Icon={icon}\nTerminal=false\nCategories=Development;Science;\n')
    for directory in ('Desktop','.local/share/applications'):
        path=home/directory/(name+'.desktop')
        path.write_text(document,encoding='utf-8')
        path.chmod(0o755)
(home/'.config/autostart/qianli-simulation.desktop').write_text(
    '[Desktop Entry]\nType=Application\nName=QianLi Simulation\n'
    f'Exec=gnome-terminal -- bash "{root}/scripts/tools/qianli.sh" sim\n'
    'X-GNOME-Autostart-enabled=true\nX-GNOME-Autostart-Delay=8\n',encoding='utf-8')
PY
if ! grep -q '# QianLi Humble environment' "$HOME/.bashrc"; then
  cat >> "$HOME/.bashrc" <<'SHELL'

# QianLi Humble environment
if [[ -f "$HOME/.config/qianli/environment.sh" ]]; then source "$HOME/.config/qianli/environment.sh"; fi
if [[ -f "$HOME/QianLi/scripts/setup/source_env.sh" ]]; then source "$HOME/QianLi/scripts/setup/source_env.sh" >/dev/null; fi
SHELL
fi
if [[ -S "/run/user/$(id -u)/bus" ]]; then
  for entry in "$HOME"/Desktop/qianli-*.desktop; do
    DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$(id -u)/bus" \
      gio set "$entry" metadata::trusted true
  done
fi
echo '[OK] Desktop, autostart and interactive terminal entries installed.'
