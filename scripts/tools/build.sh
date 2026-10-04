#!/usr/bin/env bash
# ============================================================
# build.sh — colcon build 封装（默认全量构建，--symlink-install）
#
# 用法：
#   bash scripts/tools/build.sh                                  # 构建全部
#   bash scripts/tools/build.sh qianli_description               # 构建指定包
#   bash scripts/tools/build.sh --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../qianli_ws" && pwd)"
cd "$WS_DIR"

if ! command -v colcon >/dev/null 2>&1; then
  echo "[ERROR] 未找到 colcon。请先在 Ubuntu 22.04 安装 ROS 2 Humble"
  echo "        （scripts/setup/install_ros2_humble.sh）并 source 环境。"
  exit 1
fi

echo "[INFO] 构建工作区: $WS_DIR"
exec colcon build --symlink-install "$@"
