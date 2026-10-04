#!/usr/bin/env bash
# ============================================================
# status.sh — 工作区健康检查：git status + 可构建 package 列表
#
# 用法：bash scripts/tools/status.sh
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$ROOT"

echo "=== git status ==="
git status

echo
echo "=== 可构建 ROS 2 packages（含 package.xml） ==="
find qianli_ws/src -name package.xml -printf '%h\n' | sort

echo
echo "=== 提示 ==="
echo "构建: bash scripts/tools/build.sh  （需 Ubuntu 22.04 + ROS 2 Humble 环境）"
echo "清理: bash scripts/tools/clean.sh"
