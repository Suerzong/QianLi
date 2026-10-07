#!/usr/bin/env python3
"""把 zero_raw 回滚到旧值（保留已冻结的 raw_min/raw_max）。"""

from project_paths import driver_params_path
import os
import re
import shutil
import time

OLD_ZERO = [2078, 1980, 3076, 2035, 2033, 2030]
YAMLS = [
    os.path.expanduser(driver_params_path()),
    os.path.expanduser(driver_params_path()),
]
stamp = time.strftime('%Y%m%d_%H%M%S')
for p in YAMLS:
    if not os.path.exists(p):
        print(f'缺失 {p}')
        continue
    t = open(p).read()
    before = re.search(r'zero_raw:\s*\[([^\]]*)\]', t).group(1)
    t = re.sub(r'^(\s*)zero_raw:\s*\[[^\]]*\]',
               lambda m: f'{m.group(1)}zero_raw: [{", ".join(map(str, OLD_ZERO))}]',
               t, count=1, flags=re.M)
    shutil.copy2(p, p + f'.revertbak_{stamp}')
    open(p, 'w').write(t)
    z = re.search(r'zero_raw:\s*\[([^\]]*)\]', t).group(1)
    mn = re.search(r'raw_min:\s*\[([^\]]*)\]', t).group(1)
    mx = re.search(r'raw_max:\s*\[([^\]]*)\]', t).group(1)
    print(f'{os.path.basename(os.path.dirname(os.path.dirname(p)))}:')
    print(f'  zero_raw {before.strip()} -> {z.strip()}')
    print(f'  raw_min [{mn.strip()}]')
    print(f'  raw_max [{mx.strip()}]')
