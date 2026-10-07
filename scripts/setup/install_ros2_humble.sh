#!/usr/bin/env bash
# Ubuntu 22.04 / ROS 2 Humble native installation. Run explicitly on the target.
set -euo pipefail
source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 22.04 ]]; then
  echo '[ERROR] Requires Ubuntu 22.04; no packages were installed.' >&2
  exit 1
fi
[[ "$(dpkg --print-architecture)" == amd64 ]] || { echo 'Requires amd64' >&2; exit 1; }
sudo apt-get update
sudo apt-get install -y locales curl software-properties-common python3-venv
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository -y universe
# Refresh systemd/udev before installing ROS on a fresh Jammy image (ROS docs).
sudo apt-get install --only-upgrade -y systemd udev
if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -q 'install ok installed'; then
  version="${QI_ROS_APT_SOURCE_VERSION:-$(curl -fsSL \
    https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')}"
  [[ "$version" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || { echo 'Invalid ROS apt source version' >&2; exit 1; }
  staging="$(mktemp -d)"
  trap 'rm -f -- "$staging/ros2-apt-source.deb"; rmdir -- "$staging"' EXIT
  curl -fL --retry 3 -o "$staging/ros2-apt-source.deb" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version}.jammy_all.deb"
  sudo dpkg -i "$staging/ros2-apt-source.deb"
fi
sudo apt-get update
sudo apt-get install -y ros-humble-desktop ros-dev-tools \
  python3-colcon-common-extensions python3-rosdep python3-pytest \
  ros-humble-joint-state-publisher-gui ros-humble-interactive-markers \
  python3-serial python3-yaml libgl1 libegl1 libglfw3
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi
rosdep update --rosdistro humble
echo '[OK] Humble installed. Next: bash scripts/setup/setup_python_envs.sh'
