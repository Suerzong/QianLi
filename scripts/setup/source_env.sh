#!/usr/bin/env bash
# ============================================================
# source_env.sh — 加载 ROS 2 Humble 与 QianLi 工作区环境
#
# 用法（必须在当前 shell 中 source）：
#   source scripts/setup/source_env.sh
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../qianli_ws" && pwd)"

if [ -d /opt/ros/jazzy ]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "[WARN] 未找到 /opt/ros/jazzy（或 humble）setup.bash，请先安装 ROS 2。"
fi

if [ -f "$WS_DIR/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS_DIR/install/setup.bash"
fi

echo "[OK] ROS_DISTRO=${ROS_DISTRO:-<未设置>}  workspace: $WS_DIR"
