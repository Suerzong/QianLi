#!/usr/bin/env bash
# Source this file in the current Bash shell. Never mix ROS distributions.
_qi_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export QI_PROJECT_ROOT="${QI_PROJECT_ROOT:-$(cd -- "$_qi_script_dir/../.." && pwd)}"
export QI_CALIB_DIR="${QI_CALIB_DIR:-$QI_PROJECT_ROOT/calib}"
_qi_ros_cache="${QI_ROS_INSTALL_CACHE:-$QI_PROJECT_ROOT/migration_assets/ros-install-cache}"
if [[ -z "${ROSDISTRO_INDEX_URL:-}" && -f "$_qi_ros_cache/index-local.yaml" ]]; then
  export ROSDISTRO_INDEX_URL="file://$_qi_ros_cache/index-local.yaml"
fi
_qi_distro="${QI_ROS_DISTRO:-humble}"
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != "$_qi_distro" ]]; then
  echo "[ERROR] Shell already sourced $ROS_DISTRO; open a fresh shell for $_qi_distro." >&2
  return 1
fi
if [[ ! -f "/opt/ros/$_qi_distro/setup.bash" ]]; then
  echo "[ERROR] Missing /opt/ros/$_qi_distro/setup.bash" >&2
  return 1
fi
_qi_source() {
  local _qi_had_nounset=false _qi_result=0
  [[ $- != *u* ]] || _qi_had_nounset=true
  set +u
  source "$1" || _qi_result=$?
  if $_qi_had_nounset; then set -u; fi
  return "$_qi_result"
}
_qi_source "/opt/ros/$_qi_distro/setup.bash" || return
if [[ -f "$QI_PROJECT_ROOT/.venv-ros/bin/activate" ]]; then
  _qi_source "$QI_PROJECT_ROOT/.venv-ros/bin/activate" || return
fi
if [[ -f "$QI_PROJECT_ROOT/qianli_ws/install/setup.bash" ]]; then
  _qi_source "$QI_PROJECT_ROOT/qianli_ws/install/setup.bash" || return
fi
export PYTHONPATH="$QI_PROJECT_ROOT/scripts:$QI_PROJECT_ROOT/qianli_ws/src/qianli_vision/scripts${PYTHONPATH:+:$PYTHONPATH}"
echo "[OK] ROS_DISTRO=$ROS_DISTRO  project=$QI_PROJECT_ROOT"
unset _qi_script_dir _qi_distro _qi_ros_cache
unset -f _qi_source
