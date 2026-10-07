#!/usr/bin/env bash
# Stable alias for the documented CH343P adapter; no USB reset or motor commands.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
sudo install -m 0644 "$ROOT/scripts/setup/99-qianli-arm.rules" /etc/udev/rules.d/99-qianli-arm.rules
sudo usermod -aG dialout,video "$(id -un)"
sudo udevadm control --reload-rules
echo '[OK] Log out/in for group permissions, then replug the supported arm adapter.'
echo '[OK] Set QI_ARM_PORT=/dev/qianli_arm and QI_CAMERA to a stable V4L2 path.'
