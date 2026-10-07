#!/usr/bin/env python3
"""示教中间位：--release 卸力（便于手拖），--capture 记录当前姿态为 READY。

READY 是"折叠位 -> 工作区"之间的途经姿态，用于绕开关节空间插值下塌。
"""

from project_paths import default_arm_port

from project_paths import arm_source_path, driver_params_path, project_path
import argparse
import json
import math
import os
import sys
import time

import numpy as np
import yaml
from pathlib import Path

sys.path.insert(0, arm_source_path())
from so101_bringup.servo_protocol import FeetechSerialBus

CONFIG = os.path.expanduser(
    driver_params_path())
OUT = os.path.expanduser(project_path('config/ready_pose.json'))

ap = argparse.ArgumentParser()
g = ap.add_mutually_exclusive_group(required=True)
g.add_argument('--release', action='store_true')
g.add_argument('--capture', action='store_true')
g.add_argument('--show', action='store_true')
a = ap.parse_args()

cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])

if a.show:
    if os.path.exists(OUT):
        print(json.dumps(json.load(open(OUT)), indent=2, ensure_ascii=False))
    else:
        print('尚未示教中间位')
    sys.exit(0)

bus = FeetechSerialBus(default_arm_port(), timeout_s=0.15)
if a.release:
    bus.set_torque(False)
    time.sleep(0.6)
    print('已卸力，可以手拖。当前力矩:', bus.read_torque_states())
    bus.close()
    sys.exit(0)

raw = np.array(bus.read_positions())
q = (raw - zero) * direction * 2 * math.pi / 4096
torque = bus.read_torque_states()
bus.close()
lo = np.minimum(np.array(cfg['raw_min']), np.array(cfg['raw_max']))
hi = np.maximum(np.array(cfg['raw_min']), np.array(cfg['raw_max']))
exec_raw = np.clip(raw, lo, hi)
os.makedirs(os.path.dirname(OUT), exist_ok=True)
json.dump({'raw': raw.tolist(), 'raw_exec': exec_raw.tolist(),
           'rad': q.tolist(), 'torque_at_capture': torque,
           'note': '示教中间位（折叠位<->工作区之间的途经姿态）'},
          open(OUT, 'w'), indent=2, ensure_ascii=False)
print(f'中间位已保存 → {OUT}')
print(f'  实测 raw   {raw.tolist()}')
print(f'  可执行 raw {exec_raw.tolist()}')
print(f'  夹位差     {(exec_raw - raw).tolist()}')
print(f'  采集时力矩 {torque}')
