"""Pure, ROS-independent checks shared by real grasp entry points."""
import math
import os
import time
from pathlib import Path


def down_quat_xyzw(yaw_deg):
    """Rz(yaw) @ Rx(pi), in ROS (x,y,z,w) order."""
    angle = math.radians(yaw_deg)/2
    return math.cos(angle), math.sin(angle), 0., 0.


def require_fresh(path, max_age=2.):
    age = time.time()-Path(path).stat().st_mtime
    if not math.isfinite(age) or age < -.5 or age > max_age:
        raise ValueError(f'{path}: observation age {age:.1f}s exceeds {max_age}s')


def finite_position(values, reach_limit=.36):
    xyz = tuple(float(v) for v in values)
    if len(xyz) != 3 or not all(math.isfinite(v) for v in xyz):
        raise ValueError('target requires three finite coordinates')
    reach = math.hypot(xyz[0], xyz[1])
    if reach > reach_limit:
        raise ValueError(f'target radius {reach*1000:.1f}mm exceeds {reach_limit*1000:.0f}mm')
    return xyz


def read_kv(path):
    values = {}
    for line in Path(path).read_text().splitlines():
        if '=' in line and not line.lstrip().startswith('#'):
            key, value = line.split('=',1)
            values[key.strip()] = value.strip()
    return values


def read_observation(path='/tmp/object_pose.txt', max_age=2.):
    require_fresh(path,max_age)
    pose = read_kv(path)
    if pose.get('valid') != '1':
        raise ValueError(f'visual observation invalid: {pose.get("reason","missing validity metadata")}')
    if abs(float(pose['cell_cm'])-3.3) > .001:
        raise ValueError('checkerboard scale differs from measured 3.3cm')
    x, y = float(pose['X_cm'])/100, float(pose['Y_cm'])/100
    if not all(math.isfinite(v) for v in (x,y)):
        raise ValueError('nonfinite visual coordinates')
    return x,y,pose


def atomic_text(path, content):
    path = Path(path)
    tmp = path.with_name(path.name+f'.tmp{os.getpid()}')
    tmp.write_text(content)
    os.replace(tmp,path)
