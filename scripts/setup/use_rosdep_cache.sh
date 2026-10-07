#!/usr/bin/env bash
# Use fresh host-downloaded official data when guest GitHub TLS is unavailable.
set -euo pipefail
CACHE="$(realpath -- "$1")"
python3 - "$CACHE" <<'PY'
import hashlib,json,sys
from pathlib import Path
import yaml
root=Path(sys.argv[1])
manifest=json.loads((root/'manifest.json').read_text())
for name,entry in manifest['files'].items():
    p=Path(name)
    assert not p.is_absolute() and '..' not in p.parts
    data=(root/p).read_bytes()
    assert len(data)==entry['size'] and hashlib.sha256(data).hexdigest()==entry['sha256'],name
index=yaml.safe_load((root/'index-v4.yaml').read_text())
humble=index['distributions']['humble']
humble['distribution']=[(root/'humble-distribution.yaml').as_uri()]
humble['distribution_cache']=(root/'humble-cache.yaml.gz').as_uri()
index['distributions']={'humble':humble}
(root/'index-local.yaml').write_text(yaml.safe_dump(index),encoding='utf-8')
print('[OK] ROS bootstrap cache checksums passed')
PY
sudo install -d /etc/ros/rosdep/sources.list.d
if [[ -f /etc/ros/rosdep/sources.list.d/20-default.list && ! -f "$CACHE/20-default.list.original" ]]; then
  cp /etc/ros/rosdep/sources.list.d/20-default.list "$CACHE/20-default.list.original"
fi
for name in base python ruby; do printf 'yaml file://%s/%s.yaml\n' "$CACHE" "$name"; done \
  | sudo tee /etc/ros/rosdep/sources.list.d/20-default.list >/dev/null
export ROSDISTRO_INDEX_URL="file://$CACHE/index-local.yaml"
rosdep update --rosdistro humble
