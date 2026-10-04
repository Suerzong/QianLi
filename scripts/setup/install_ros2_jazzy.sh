#!/usr/bin/env bash
# ============================================================
# install_ros2_jazzy.sh — 在 Ubuntu 24.04 上安装 ROS 2 Jazzy 基础环境
#
# 范围：locale、ROS 2 apt 源、ros-jazzy-desktop、ros-dev-tools、
#      colcon 扩展、rosdep 初始化。
# 说明：仅安装基础依赖；Nav2 / MoveIt2 / Gazebo 等按 Milestone 需要时再装。
# 用法：bash scripts/setup/install_ros2_jazzy.sh
# 注意：需要 sudo；开发虚拟机一般已装好，本脚本用于新环境复现。
# ============================================================
set -euo pipefail

# 1) 检查发行版是否为 Ubuntu 24.04
if ! grep -q "Ubuntu 24.04" /etc/os-release 2>/dev/null; then
  echo "[ERROR] 本脚本仅支持 Ubuntu 24.04（目标平台），当前环境："
  head -n 2 /etc/os-release 2>/dev/null || echo "无法读取 /etc/os-release"
  exit 1
fi

# 2) 已安装则跳过（幂等）
if [ -f /opt/ros/jazzy/setup.bash ]; then
  echo "[SKIP] 检测到 /opt/ros/jazzy/setup.bash，ROS 2 Jazzy 已安装。"
  exit 0
fi

echo "[1/5] 配置 locale（UTF-8）..."
sudo apt-get update && sudo apt-get install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "[2/5] 添加 ROS 2 apt 源..."
sudo apt-get install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "[3/5] 安装 ROS 2 Jazzy（桌面版 + 开发工具 + colcon + rosdep）..."
sudo apt-get update
sudo apt-get install -y ros-jazzy-desktop ros-dev-tools \
  python3-colcon-common-extensions python3-rosdep

echo "[4/5] rosdep 初始化..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

echo "[5/5] 完成。请在新 shell 中执行：source /opt/ros/jazzy/setup.bash"
echo "      然后：bash scripts/tools/build.sh"
