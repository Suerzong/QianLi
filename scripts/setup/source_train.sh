#!/usr/bin/env bash
_qi_root="${QI_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)}"
if [[ -n "${ROS_DISTRO:-}" ]]; then
  echo '[ERROR] Open a fresh shell for the isolated training environment.' >&2
  return 1
fi
if [[ ! -f "$_qi_root/.venv-train/bin/activate" ]]; then
  echo '[ERROR] Run scripts/setup/setup_python_envs.sh first.' >&2
  return 1
fi
source "$_qi_root/.venv-train/bin/activate" || return
export QI_PROJECT_ROOT="$_qi_root"
export QI_CALIB_DIR="${QI_CALIB_DIR:-$_qi_root/calib}"
export QI_SO101_PKG="${QI_SO101_PKG:-$_qi_root/dual_twin}"
export QI_PARTS_DIR="${QI_PARTS_DIR:-$_qi_root/dual_twin/mj_parts}"
export PYTHONPATH="$_qi_root/qianli_ws/src/qianli_vision:$_qi_root/qianli_ws/src/qianli_vision/scripts:$_qi_root/dual_twin/scripts"
echo '[OK] Isolated training environment loaded'
unset _qi_root
