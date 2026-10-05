#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$task_root/qianli_ws/install/setup.bash"
set -u
ros2 node list
ros2 topic list
ros2 control list_controllers
ros2 topic info /cmd_vel
ros2 topic info /odom
ros2 run qianli_sim check_system.py "$@" --ros-args -p use_sim_time:=true
