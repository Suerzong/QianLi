"""Fail closed when a real robot's extrinsic calibration is unknown or invalid."""
import math
from pathlib import Path
from .runtime_paths import calibration_path


CALIBRATION_JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex',
                      'wrist_flex', 'wrist_roll', 'gripper')


def physical_collection_error(status, joints, status_age_s, joints_age_s, max_age_s=2.0):
    """Require current physical feedback before recording a manual touch point."""
    if not isinstance(status, dict) or status.get('mode') != 'direct':
        return '需要 direct 真机驱动；不能使用仿真或未知来源的反馈标定'
    if any(status.get(key) is not False for key in ('allow_motion', 'enabled', 'holding')):
        return '手动采点需要 allow_motion=false、enabled=false、holding=false'
    if status.get('fault') != '':
        return '真机驱动存在故障或缺少故障状态，暂停记录'
    for age in (status_age_s, joints_age_s):
        if age is None or not math.isfinite(age) or not 0 <= age <= max_age_s:
            return '真机状态或关节反馈已过期，暂停记录'
    try:
        if any(not math.isfinite(float(joints[name])) for name in CALIBRATION_JOINTS):
            return '关节反馈含非有限数值，暂停记录'
    except (KeyError, TypeError, ValueError):
        return '需要完整的六关节反馈，暂停记录'
    return None


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
