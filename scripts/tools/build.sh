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
source "$SCRIPT_DIR/../setup/source_env.sh"

if ! command -v colcon >/dev/null 2>&1; then
  echo "[ERROR] 未找到 colcon。请先在 Ubuntu 22.04 安装 ROS 2 Humble"
  echo "        （scripts/setup/install_ros2_humble.sh）并 source 环境。"
  exit 1
fi

echo "[INFO] 构建工作区: $WS_DIR"
python3 -c 'import sys; assert sys.version_info[:2] == (3,10), "Humble requires Python 3.10"'
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
if [[ $# -gt 0 && "$1" != --* ]]; then set -- --packages-select "$@"; fi
exec python3 -c 'from colcon_core.command import main; raise SystemExit(main())' build --symlink-install "$@"
