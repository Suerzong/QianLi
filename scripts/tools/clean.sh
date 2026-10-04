#!/usr/bin/env bash
# ============================================================
# clean.sh — 清理 colcon 构建产物（build / install / log）
#
# 用法：bash scripts/tools/clean.sh
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../qianli_ws" && pwd)"
cd "$WS_DIR"

rm -rf build install log
echo "[OK] 已清理 $WS_DIR 下的 build / install / log"
