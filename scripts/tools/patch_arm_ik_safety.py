#!/usr/bin/env python3
"""Patch the existing SO101 IK node without launching ROS or touching hardware.

The original file is preserved once as .before_grasp_safety. Re-running is safe.
"""
import argparse
import ast
import textwrap
from pathlib import Path


METHODS = {
'_seed_list': '''
def _seed_list(self, ikpy_sol):
    seeds = []
    if ikpy_sol is not None:
        seeds.append(np.asarray(ikpy_sol,dtype=float))
    seeds.extend([self.q_target.copy(),self.q_current.copy(),np.zeros(len(self.chain.links))])
    middle = np.zeros(len(self.chain.links))
    bent = middle.copy()
    ready = dict(zip(IK_JOINT_NAMES,(0.,-1.,1.5,-.7,-2.2)))
    for i,link in enumerate(self.chain.links):
        if link.name in IK_JOINT_NAMES:
            middle[i] = .5*sum(link.bounds)
            bent[i] = ready[link.name]
    seeds.extend([middle,bent])
    return seeds
''',
'_dls_refine': '''
def _dls_refine(self, q0, target_pos, iters=400, lam=2e-4, step=.8, target_rotation=None):
    """Refine position and the requested orientation together."""
    q = self._clip_bounds(np.asarray(q0, dtype=float).copy())
    weight = .08
    mode = self.orientation_mode if target_rotation is not None else 'none'
    def feature(matrix):
        if mode == 'Z':
            return np.concatenate([matrix[:3,3],weight*matrix[:3,2]])
        if mode == 'all':
            return np.concatenate([matrix[:3,3],weight*matrix[:3,:3].ravel()])
        return matrix[:3,3]
    desired = np.eye(4)
    desired[:3,3] = target_pos
    if target_rotation is not None:
        desired[:3,:3] = target_rotation
    goal = feature(desired)
    for _ in range(iters):
        current = feature(self._tip_matrix(q))
        error = goal-current
        if np.linalg.norm(error[:3]) < 5e-5 and np.linalg.norm(error[3:]) < 4e-5:
            break
        jac = np.zeros((len(error),len(q)))
        for i, active in enumerate(self.chain.active_links_mask):
            if active:
                shifted = q.copy()
                shifted[i] += 2e-4
                jac[:,i] = (feature(self._tip_matrix(shifted))-current)/2e-4
        dq = jac.T @ np.linalg.solve(jac @ jac.T+lam*np.eye(len(error)),error)
        q = self._clip_bounds(q+np.clip(dq*step,-.06,.06))
    return q,float(np.linalg.norm(self._tip_matrix(q)[:3,3]-target_pos))
''',
'_solve_dls': '''
def _solve_dls(self, target_mat):
    """Only accept solutions meeting both position and orientation tolerances."""
    mode = None if self.orientation_mode == 'none' else self.orientation_mode
    initial = None
    try:
        initial = self.chain.inverse_kinematics_frame(target_mat,
                     initial_position=self.q_target,orientation_mode=mode)
    except Exception as exc:
        self._warn_throttled(f'ikpy seed failed: {exc}')
    best_q,best_err,best_score = None,float('inf'),float('inf')
    for seed in self._seed_list(initial):
        q,error = self._dls_refine(seed,target_mat[:3,3],target_rotation=target_mat[:3,:3])
        axis_error = self._orientation_error_deg(q,target_mat)
        score = error/self.max_ik_residual+axis_error/5.
        if score < best_score:
            best_q,best_err,best_score = q,error,score
        if error <= self.max_ik_residual and axis_error <= 5.:
            return q,error
    return best_q,best_err
''',
'_solve_ik': '''
def _solve_ik(self):
    if self.have_driver_status and not self._hardware_limits_ready:
        self._warn_throttled('Waiting for live driver calibration/limits; target rejected')
        return
    sol,error = self._solve_dls(self.target_mat)
    if sol is None or not np.isfinite(sol).all() or not math.isfinite(error) or error > self.max_ik_residual:
        self._warn_throttled(f'Rejected target: position residual {error*1000:.1f}mm')
        return
    angle = self._orientation_error_deg(sol,self.target_mat)
    if not math.isfinite(angle) or angle > 5.:
        self._warn_throttled(f'Rejected target: orientation error {angle:.1f}deg')
        return
    self.q_target = np.asarray(sol,dtype=float)
''',
'_on_target': '''
def _on_target(self, msg):
    if not self._target_input_ready() or not self._frame_is_supported(msg.header.frame_id):
        return
    p,q = msg.pose.position,msg.pose.orientation
    values = (p.x,p.y,p.z,q.x,q.y,q.z,q.w)
    if not all(math.isfinite(v) for v in values) or sum(v*v for v in values[3:]) < 1e-12:
        self._warn_throttled('Rejected nonfinite pose or zero quaternion')
        return
    self._set_cartesian_target(pose_to_matrix(msg.pose))
''',
}

EXTRA = '''
def _orientation_error_deg(self,q,target):
    if self.orientation_mode == 'none':
        return 0.
    actual = self._tip_matrix(q)[:3,:3]
    desired = target[:3,:3]
    if self.orientation_mode == 'Z':
        cosine = np.dot(actual[:,2],desired[:,2])
    else:
        cosine = (np.trace(desired @ actual.T)-1)/2
    return math.degrees(math.acos(float(np.clip(cosine,-1.,1.))))

def _request_hardware_limits(self):
    if self._limits_pending or not self._limits_client.service_is_ready():
        return
    self._limits_pending = True
    request = GetParameters.Request(names=['mode','zero_raw','direction','raw_min','raw_max'])
    future = self._limits_client.call_async(request)
    future.add_done_callback(self._receive_hardware_limits)

def _receive_hardware_limits(self,future):
    self._limits_pending = False
    try:
        values = future.result().values
        mode = values[0].string_value
        arrays = [list(v.integer_array_value) for v in values[1:]]
        if mode not in ('sim','direct','lerobot') or any(len(a) != 6 for a in arrays):
            raise ValueError('driver parameters missing or invalid')
        zero,direction,minimum,maximum = arrays
        bounds = {}
        for name,z,d,lo,hi in zip(JOINT_NAMES,zero,direction,minimum,maximum):
            if d not in (-1,1) or not 0 <= z <= 4095 or not 0 <= lo < hi <= 4095:
                raise ValueError('invalid servo limits')
            ends = [(lo-z)*d*2*math.pi/4096,(hi-z)*d*2*math.pi/4096]
            bounds[name] = (min(ends),max(ends))
        for link in self.chain.links:
            if link.name in self._urdf_joint_bounds:
                original = self._urdf_joint_bounds[link.name]
                if mode == 'sim':
                    link.bounds = original
                else:
                    lo,hi = bounds[link.name]
                    link.bounds = (max(original[0],lo),min(original[1],hi))
                    if link.bounds[0] >= link.bounds[1]:
                        raise ValueError('driver and URDF ranges do not overlap')
        self._hardware_limits_ready = True
    except Exception as exc:
        self._hardware_limits_ready = False
        self._warn_throttled(f'Cannot validate driver limits: {exc}')
'''


def update_method(source,name,replacement):
    tree = ast.parse(source)
    node = next(c for c in tree.body if isinstance(c,ast.ClassDef) and c.name == 'So101IkNode')
    method = next(f for f in node.body if isinstance(f,ast.FunctionDef) and f.name == name)
    lines = source.splitlines(keepends=True)
    lines[method.lineno-1:method.end_lineno] = [textwrap.indent(textwrap.dedent(replacement).strip()+'\n','    ')]
    return ''.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('path',type=Path)
    args = ap.parse_args()
    source = args.path.read_text(encoding='utf-8-sig')
    if '_request_hardware_limits' in source:
        for name,replacement in METHODS.items():
            source = update_method(source,name,replacement)
        compile(source,str(args.path),'exec')
        args.path.write_text(source,encoding='utf-8')
        print('grasp IK safety methods updated; original backup preserved')
        return
    backup = args.path.with_name(args.path.name+'.before_grasp_safety')
    if not backup.exists():
        backup.write_text(source,encoding='utf-8')
    source = source.replace('from std_srvs.srv import SetBool, Trigger',
                            'from std_srvs.srv import SetBool, Trigger\nfrom rcl_interfaces.srv import GetParameters')
    anchor = '        self.have_driver_status = False\n'
    assert anchor in source
    source = source.replace(anchor,anchor+
        '        self._hardware_limits_ready = False\n'
        '        self._limits_pending = False\n'
        "        self._limits_client = self.create_client(GetParameters, '/so101_driver/get_parameters')\n"
        '        self._limits_timer = self.create_timer(1., self._request_hardware_limits)\n')
    anchor = '        self._apply_active_mask()\n'
    assert anchor in source
    source = source.replace(anchor,anchor+
        '        self._urdf_joint_bounds = {link.name:link.bounds for link in self.chain.links if link.name in IK_JOINT_NAMES}\n')
    for name,replacement in METHODS.items():
        source = update_method(source,name,replacement)
    extra = textwrap.indent(textwrap.dedent(EXTRA).strip()+'\n\n','    ')
    source = source.replace('    def _warn_throttled(self, text):',extra+'    def _warn_throttled(self, text):')
    compile(source,str(args.path),'exec')
    args.path.write_text(source,encoding='utf-8')
    print(f'patched {args.path}; backup {backup}')


if __name__ == '__main__':
    main()
