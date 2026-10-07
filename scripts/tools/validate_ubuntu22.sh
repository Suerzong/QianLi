#!/usr/bin/env bash
# Native target acceptance; simulated ROS only, never enables physical motion.
set -eo pipefail
ROOT="${QI_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)}"
source "$ROOT/scripts/setup/source_env.sh"
python3 "$ROOT/scripts/tools/migration_check.py" --target --ros
python3 "$ROOT/qianli_ws/src/qianli_vision/scripts/block_pipeline.py" --selftest
python3 "$ROOT/qianli_ws/src/qianli_vision/scripts/color_block_detect.py" --selftest
python3 -m pytest "$ROOT/tests/test_migration.py" "$ROOT/tests/test_arm_acceptance.py" -q
cd "$ROOT/qianli_ws"
python3 -c 'from colcon_core.command import main; raise SystemExit(main())' test --event-handlers console_direct+
python3 -c 'from colcon_core.command import main; raise SystemExit(main())' test-result --verbose
export ROS_DOMAIN_ID="${QI_ACCEPTANCE_DOMAIN_ID:-79}"
nodes="$(timeout 15 ros2 node list --no-daemon --spin-time 3)"
if grep -q '^/' <<< "$nodes"; then
  echo '[ERROR] Acceptance ROS domain is occupied; set QI_ACCEPTANCE_DOMAIN_ID to an unused domain.' >&2
  exit 1
fi
setsid ros2 launch so101_bringup ik_demo.launch.py driver_mode:=sim allow_motion:=false use_rviz:=false > "$ROOT/qianli_ws/log/migration-launch.log" 2>&1 &
launch_pid=$!
trap 'kill -TERM -- "-$launch_pid" 2>/dev/null || true; wait "$launch_pid" 2>/dev/null || true' EXIT
sleep 3
kill -0 "$launch_pid"
# DDS discovery can take longer than the CLI's default one-second spin.
ready=false
for attempt in {1..6}; do
  if timeout 12 ros2 param get /so101_driver allow_motion --no-daemon --spin-time 3 \
      2>/dev/null | grep -qx 'Boolean value is: False'; then
    ready=true
    break
  fi
  kill -0 "$launch_pid"
done
if ! "$ready"; then
  cat "$ROOT/qianli_ws/log/migration-launch.log" >&2
  echo '[ERROR] Simulated driver was not discovered with allow_motion=false.' >&2
  exit 1
fi
timeout 20 ros2 topic echo /joint_states sensor_msgs/msg/JointState --once --no-daemon --full-length > "$ROOT/qianli_ws/log/migration-joints.yaml"
timeout 20 ros2 topic echo /tf tf2_msgs/msg/TFMessage --once --no-daemon --full-length > "$ROOT/qianli_ws/log/migration-tf.yaml"
timeout 20 ros2 topic echo /safety_gate_state std_msgs/msg/String --once --no-daemon --full-length > "$ROOT/qianli_ws/log/migration-gate.yaml"
echo '[OK] Humble build/test, vision and simulated joint/TF/gate checks passed.'
