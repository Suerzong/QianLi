#!/usr/bin/env bash
# ============================================================
# vm_check.sh — QianLi 开发虚拟机健康检查（SSH）
#
# 用法：bash scripts/tools/vm_check.sh
# 输出：连通性 / sshd 状态 / 系统信息 / 磁盘 / GPU / ROS / conda
# ============================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 复用 vm_ssh.sh 的连通检测逻辑：确定可用目标
REMOTE=""
if ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new \
    qianli-vm "true" 2>/dev/null; then
  REMOTE="qianli-vm"
elif ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new \
    -i "$HOME/.ssh/id_ed25519" ros@192.168.26.128 "true" 2>/dev/null; then
  REMOTE="-i $HOME/.ssh/id_ed25519 ros@192.168.26.128"
else
  echo "[ERROR] 无法免密连接虚拟机。请检查："
  echo "  - 虚拟机是否开机（VMware）"
  echo "  - sshd 是否运行（docs/SSH.md §4 排查清单）"
  exit 1
fi

echo "=== SSH 连通性 ==="
echo "[OK] $REMOTE"

echo
echo "=== sshd 状态 ==="
# shellcheck disable=SC2086
ssh $REMOTE "systemctl is-enabled ssh 2>&1; systemctl is-active ssh 2>&1"

echo
echo "=== 系统信息 ==="
# shellcheck disable=SC2086
ssh $REMOTE "hostname; lsb_release -ds 2>/dev/null; echo '内存:'; free -h | sed -n '1,2p'"

echo
echo "=== 磁盘 ==="
# shellcheck disable=SC2086
ssh $REMOTE "df -h / | tail -1"

echo
echo "=== GPU ==="
# shellcheck disable=SC2086
ssh $REMOTE "nvidia-smi -L 2>/dev/null || echo '（无 nvidia-smi，虚拟机未直通 GPU）'"

echo
echo "=== ROS 2 ==="
# shellcheck disable=SC2086
ssh $REMOTE "ls /opt/ros 2>/dev/null; ls ~/QianLi/qianli_ws/src 2>/dev/null | head -20"

echo
echo "=== conda ==="
# shellcheck disable=SC2086
ssh $REMOTE "which conda 2>/dev/null || echo '（无 conda）'"

echo
echo "=== 提示 ==="
echo "交互登录: bash scripts/tools/vm_ssh.sh    （详见 docs/SSH.md）"
