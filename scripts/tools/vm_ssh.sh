#!/usr/bin/env bash
# ============================================================
# vm_ssh.sh — 连接 QianLi 开发虚拟机（一键 SSH）
#
# 用法：
#   bash scripts/tools/vm_ssh.sh                 # 连接（交互式）
#   bash scripts/tools/vm_ssh.sh <cmd...>        # 执行远程命令后返回
#   bash scripts/tools/vm_ssh.sh check           # 只检测连通性，不进入
#
# 说明：优先用主机 SSH 别名 qianli-vm；无别名则回退到
#       ros@192.168.26.128（密码 ros1234）。详见 docs/SSH.md。
# ============================================================
set -uo pipefail

VM_HOST="192.168.26.128"
VM_USER="ros"

# 检测别名是否可用（在 Windows 主机上优先）
if ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new \
    qianli-vm "true" 2>/dev/null; then
  TARGET="qianli-vm"
elif ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new \
    -i "$HOME/.ssh/id_ed25519" "$VM_USER@$VM_HOST" "true" 2>/dev/null; then
  TARGET="-i $HOME/.ssh/id_ed25519 $VM_USER@$VM_HOST"
else
  echo "[WARN] 免密不可用，改用密码登录（密码 ros1234）。" >&2
  TARGET="$VM_USER@$VM_HOST"
fi

if [[ "${1:-}" == "check" ]]; then
  echo "[OK] SSH 连通（$TARGET）"
  exit 0
fi

echo "[INFO] 连接: $TARGET"
if [[ $# -gt 0 ]]; then
  # shellcheck disable=SC2086
  exec ssh $TARGET "$@"
else
  # shellcheck disable=SC2086
  exec ssh $TARGET
fi
