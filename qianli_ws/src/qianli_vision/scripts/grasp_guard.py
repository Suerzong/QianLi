"""Pure, ROS-independent checks shared by real grasp entry points."""
import math
import os
import time
from pathlib import Path


def down_quat_xyzw(yaw_deg):
    """Rz(yaw) @ Rx(pi), in ROS (x,y,z,w) order."""
    angle = math.radians(yaw_deg)/2
    return math.cos(angle), math.sin(angle), 0., 0.


def tool_down_error_deg(q):
    """Angle between measured tool Z and base -Z, for a ROS quaternion."""
    values = (q.x,q.y,q.z,q.w)
    norm = math.sqrt(sum(v*v for v in values))
    if not all(math.isfinite(v) for v in values) or norm < 1e-9:
        return float('inf')
    x,y = q.x/norm,q.y/norm
    return math.degrees(math.acos(max(-1.,min(1.,2*(x*x+y*y)-1))))


class FeedbackGuard:
    """Require recent physical joint feedback and direct driver status."""
    def __init__(self):
        self.joints_at = self.status_at = float('-inf')
        self.status = {}
        self.positions = {}
        self._chain = None

    def on_joints(self, msg):
        required = {'shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper'}
        if (len(msg.name) == len(msg.position) and required <= set(msg.name)
                and all(math.isfinite(v) for v in msg.position)):
            self.joints_at = time.monotonic()
            self.positions = dict(zip(msg.name,msg.position))

    def on_status(self, msg):
        import json
        try:
            status = json.loads(msg.data)
            if not isinstance(status,dict):
                raise ValueError('invalid status')
            self.status = status
            self.status_at = time.monotonic()
        except (ValueError,TypeError):
            self.status = {}

    def fresh(self, enabled=False):
        now = time.monotonic()
        return (now-self.joints_at <= 1. and now-self.status_at <= 1.
                and self.status.get('mode') == 'direct'
                and not self.status.get('fault')
                and (not enabled or self.status.get('enabled') is True))

    def tcp_matrix(self):
        """FK from received encoders, avoiding stale/duplicate TF publishers."""
        import numpy as np
        if not self.fresh():
            return None
        if self._chain is None:
            from ikpy.chain import Chain
            package = os.environ.get('QI_SO101_PKG',str(Path.home()/'legacy/arm/arm-final/ros2_ws/install/so101_bringup/share/so101_bringup'))
            self._chain = Chain.from_urdf_file(str(Path(package)/'urdf/so101.urdf'),base_elements=['base_link'])
            self._chain.active_links_mask = [link.name in self.positions for link in self._chain.links]
        q = np.array([self.positions.get(link.name,0.) for link in self._chain.links])
        return self._chain.forward_kinematics(q)


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
    cell = float(pose['cell_cm'])
    if not math.isfinite(cell) or abs(cell-3.3) > .001:
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
