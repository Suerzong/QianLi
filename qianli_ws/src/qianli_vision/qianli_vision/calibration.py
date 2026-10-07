"""Fail closed when a real robot's extrinsic calibration is unknown or invalid."""
import math
from pathlib import Path
from .runtime_paths import calibration_path


def load_extrinsics(path=None):
    path = Path(path or calibration_path('extrinsic.txt'))
    try:
        text=path.read_text(encoding='utf-8')
    except OSError as exc:
        return None, f'外参文件读不了：{path} ({exc})'
    values={}
    for line in text.splitlines():
        line=line.strip()
        if line.startswith('#'):
            if any(mark in line for mark in ('两点法', '废弃', 'DEPRECATED')):
                return None, f'外参是已废弃的旧值，拒绝使用：{path}'
            if '质量' in line and 'FAIL' in line:
                return None, f'外参质量裁决未通过：{path}'
        elif '=' in line:
            key,value=line.split('=',1)
            try: values[key.strip()]=float(value.strip())
            except ValueError: pass
    required=('grid_origin_x','grid_origin_y','grid_origin_z','grid_theta_deg','quality_ok')
    missing=[key for key in required if key not in values]
    if missing:
        return None, f'外参缺字段或质量标记 {missing}：{path}'
    if any(not math.isfinite(values[key]) for key in required):
        return None, f'外参含非有限数值，拒绝使用：{path}'
    if values['quality_ok'] < 0.5:
        return None, f'外参 quality_ok=0，拒绝使用：{path}'
    return values,None
