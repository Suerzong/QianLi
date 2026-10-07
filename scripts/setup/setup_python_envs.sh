#!/usr/bin/env bash
set -euo pipefail
source /etc/os-release
[[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 22.04 ]] || { echo 'Requires Ubuntu 22.04' >&2; exit 1; }
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
python3.10 -c 'import sys; assert sys.version_info[:2] == (3,10)'
profiles='ros train'
if [[ $# -gt 0 ]]; then
  [[ $# == 2 && "$1" == --profile && "$2" =~ ^(ros|train|all)$ ]] || {
    echo 'Usage: setup_python_envs.sh [--profile ros|train|all]' >&2; exit 1;
  }
  [[ "$2" == all ]] || profiles="$2"
fi
for profile in $profiles; do
  env_dir="$ROOT/.venv-$profile"
  if [[ -e "$env_dir" ]]; then
    "$env_dir/bin/python" -c 'import sys; assert sys.version_info[:2] == (3,10)'
    expected=false; [[ "$profile" == ros ]] && expected=true
    grep -q "include-system-site-packages = $expected" "$env_dir/pyvenv.cfg" || {
      echo "[ERROR] $env_dir has the wrong isolation; preserve it and use a clean checkout." >&2; exit 1;
    }
  elif [[ "$profile" == ros ]]; then
    python3.10 -m venv --system-site-packages "$env_dir"
  else
    python3.10 -m venv "$env_dir"
  fi
  "$env_dir/bin/python" -m pip install 'pip==25.1.1' 'setuptools==69.5.1' 'wheel==0.45.1'
  if [[ "$profile" == train ]]; then
    "$env_dir/bin/python" -m pip install 'torch==2.8.0' --index-url https://download.pytorch.org/whl/cu128
  fi
  "$env_dir/bin/python" -m pip install -r "$ROOT/scripts/setup/requirements/$profile.txt"
  "$env_dir/bin/python" -m pip check
  "$env_dir/bin/python" -m pip freeze --local > "$env_dir/resolved-requirements.txt"
done
mkdir -p -- "${QI_CALIB_DIR:-$ROOT/calib}"
echo '[OK] ROS: source scripts/setup/source_env.sh'
echo '[OK] Training: source scripts/setup/source_train.sh'
