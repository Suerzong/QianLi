#!/usr/bin/env bash
# Persistent VMware desktop; run on the new Jammy guest as its regular user.
set -euo pipefail
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 ]] || exit 1
export DEBIAN_FRONTEND=noninteractive
sudo env DEBIAN_FRONTEND=noninteractive apt-get -o DPkg::Lock::Timeout=180 update
sudo env DEBIAN_FRONTEND=noninteractive apt-get -o DPkg::Lock::Timeout=180 install -y \
  ubuntu-desktop-minimal gnome-terminal open-vm-tools-desktop gedit mesa-utils \
  fonts-noto-cjk language-pack-zh-hans linux-generic-hwe-22.04
sudo locale-gen en_US.UTF-8 zh_CN.UTF-8
sudo update-locale LANG=en_US.UTF-8
sudo install -d /etc/gdm3
sudo tee /etc/gdm3/custom.conf >/dev/null <<'GDM'
[daemon]
WaylandEnable=false
AutomaticLoginEnable=true
AutomaticLogin=ros
GDM
sudo passwd -d ros
mkdir -p "$HOME/.config" "$HOME/Desktop"
touch "$HOME/.config/gnome-initial-setup-done"
sudo systemctl set-default graphical.target
sudo systemctl enable gdm3
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
echo '[OK] Persistent desktop installed; reboot after project setup.'
