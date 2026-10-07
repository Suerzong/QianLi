#!/usr/bin/env bash
# ============================================================
# clean.sh — 清理 colcon 构建产物；保留 log 中的标定和回滚资产
#
# 用法：bash scripts/tools/clean.sh
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../qianli_ws" && pwd)"
cd "$WS_DIR"

[[ -d src && "$(basename "$WS_DIR")" == qianli_ws ]] || { echo 'Invalid workspace' >&2; exit 1; }
rm -rf -- build install
echo "[OK] 已清理 $WS_DIR 下的 build / install；log 已保留。"
