#!/usr/bin/env bash
# Desktop/terminal entry point for the installed QianLi workspace.
set -eo pipefail
ROOT="${QI_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)}"
action="${1:-sim}"
[[ $# == 0 ]] || shift
if [[ -f "$HOME/.config/qianli/environment.sh" ]]; then
  source "$HOME/.config/qianli/environment.sh"
fi
train_shell() {
  # Training must not inherit ROS Python or shared-library paths from a terminal.
  env -i HOME="$HOME" USER="${USER:-ros}" LANG="${LANG:-en_US.UTF-8}" \
    PATH=/usr/local/bin:/usr/bin:/bin QI_PROJECT_ROOT="$ROOT" \
    QI_CALIB_DIR="${QI_CALIB_DIR:-$ROOT/calib}" \
    bash --noprofile --norc -c \
    'cd "$QI_PROJECT_ROOT"; source scripts/setup/source_train.sh; exec "$@"' bash "$@"
}
cd "$ROOT"
case "$action" in
  sim)
    source scripts/setup/source_env.sh
    # Avoid opening a second driver/IK/gate set in the user's simulation domain.
    if timeout 12 ros2 node list --no-daemon --spin-time 3 | grep -qx /so101_driver; then
      echo '[OK] Simulation is already running; opening another view.'
      exec rviz2 -d "$ROOT/qianli_ws/install/so101_bringup/share/so101_bringup/config/ik_demo.rviz"
    fi
    exec ros2 launch so101_bringup ik_demo.launch.py driver_mode:=sim \
      allow_motion:=false calibrated:=false use_rviz:=true "$@"
    ;;
  check)
    mkdir -p migration_assets/vm-acceptance
    bash scripts/tools/validate_ubuntu22.sh 2>&1 | tee migration_assets/vm-acceptance/ros-check.log
    train_shell python -m pytest tests/test_migration.py -q 2>&1 | tee migration_assets/vm-acceptance/simulation-tests.log
    for size in 0.02 0.04; do
      train_shell python scripts/tools/train_smoke.py --device cpu --obj-size "$size" --steps 1024 \
        2>&1 | tee "migration_assets/vm-acceptance/train-${size}.log"
    done
    train_shell env MUJOCO_GL=egl python scripts/tools/migration_check.py --target --render \
      2>&1 | tee migration_assets/vm-acceptance/render.log
    echo '[OK] ROS, vision, both simulation sizes, CPU training and rendering passed.'
    ;;
  train)
    size="${1:-0.04}"
    [[ $# == 0 ]] || shift
    tag="vm_${size}_$(date +%Y%m%d_%H%M%S)"
    train_shell python dual_twin/scripts/train.py --device cpu --obj-size "$size" \
      --n-envs 4 --batch-size 256 --tag "$tag" "$@"
    ;;
  training-smoke)
    for size in 0.02 0.04; do
      train_shell python scripts/tools/train_smoke.py --device cpu --obj-size "$size" --steps 1024 "$@"
    done
    ;;
  camera)
    source scripts/setup/source_env.sh
    python3 qianli_ws/src/qianli_vision/scripts/camera_probe.py "$@"
    if [[ -n "${DISPLAY:-}" ]] && command -v eog >/dev/null; then
      eog /tmp/raw_frame_marked.png
    fi
    ;;
  devices)
    source scripts/setup/source_env.sh
    python3 scripts/tools/migration_check.py --target --devices
    exec python3 scripts/tools/arm_readonly_check.py "$@"
    ;;
  terminal)
    source scripts/setup/source_env.sh
    exec bash --noprofile --rcfile "$HOME/.bashrc" -i
    ;;
  *) echo 'Usage: qianli.sh sim|check|training-smoke|train [0.02|0.04] [arguments]|camera|devices|terminal' >&2; exit 2 ;;
esac
