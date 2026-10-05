#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$task_root/qianli_ws/install/setup.bash"
set -u
timeout 15 ros2 node list
timeout 15 ros2 topic list
timeout 20 ros2 control list_controllers
timeout 15 ros2 topic info /cmd_vel
timeout 15 ros2 topic info /odom
ros2 run qianli_sim check_system.py "$@" --ros-args -p use_sim_time:=true
