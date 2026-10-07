#!/usr/bin/env bash
# Continue the persistent VM installation after the desktop job.
set -Eeuo pipefail
ROOT="${QI_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)}"
cd "$ROOT"
LOG="$ROOT/migration_assets/vm-install"
mkdir -p "$LOG"
status() {
  python3 - "$LOG/status.json" "$1" <<'PY'
import json,sys,datetime
from pathlib import Path
path=Path(sys.argv[1])
temp=path.with_suffix('.tmp')
temp.write_text(json.dumps(dict(phase=sys.argv[2],time=datetime.datetime.now(datetime.timezone.utc).isoformat())),encoding='utf-8')
temp.replace(path)
PY
  echo "[PHASE] $1"
}
trap 'status failed; echo "[ERROR] Installation failed at line $LINENO" >&2' ERR
export PIP_DEFAULT_TIMEOUT=120 PIP_RETRIES=5 DEBIAN_FRONTEND=noninteractive
resume=false
if [[ $# == 1 && "$1" == --resume ]]; then
  resume=true
  [[ -f /opt/ros/humble/setup.bash ]]
  for profile in ros train; do
    "$ROOT/.venv-$profile/bin/python" -m pip check
  done
elif [[ $# != 0 ]]; then
  echo 'Usage: install_vm_project.sh [--resume]' >&2
  exit 2
fi
if ! "$resume" && [[ -f "$LOG/desktop-install.pid" ]]; then
  desktop_pid="$(cat "$LOG/desktop-install.pid")"
  status waiting-for-desktop
  while [[ -e "/proc/$desktop_pid/status" ]] && ! grep -q '^State:.*Z' "/proc/$desktop_pid/status"; do sleep 5; done
  grep -q '\[OK\] Persistent desktop installed' "$LOG/desktop-install.log"
fi
if ! "$resume"; then
  status installing-humble
  bash scripts/setup/install_ros2_humble.sh > "$LOG/humble-install.log" 2>&1
  status installing-python-environments
  bash scripts/setup/setup_python_envs.sh > "$LOG/python-install.log" 2>&1
fi
status building-workspace
bash scripts/tools/build.sh > "$LOG/build.log" 2>&1
status installing-desktop-entries
bash scripts/setup/install_device_rules.sh
bash scripts/setup/install_vm_launchers.sh
status running-acceptance
bash scripts/tools/qianli.sh check > "$LOG/acceptance.log" 2>&1
status ready-for-reboot
echo '[OK] Persistent QianLi VM installed and acceptance passed.'
