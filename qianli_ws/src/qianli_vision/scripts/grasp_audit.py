#!/usr/bin/env python3
"""Read-only camera replay, quaternion regression and offline IK/geometry audit.

No Node instances, ROS publishers/services or serial ports are created.
"""
import argparse
import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace
import sys
import time
import contextlib
import io
import os
import tempfile

import cv2
import mujoco
import numpy as np
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from qianli_vision import object_localizer as OL
import sim_grasp as S
import sim_mesh_gripper as MG
import real_grasp_ok as REAL
import vision_snapshot as SNAP
import auto_grasp as AUTO
from grasp_guard import require_fresh, finite_position, FeedbackGuard, tool_down_error_deg, read_observation


def rotation(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


class Replay:
    _calibrate = OL.ObjectLocalizer._calibrate
    _detect_object = OL.ObjectLocalizer._detect_object
    _load_bg = OL.ObjectLocalizer._load_bg
    _filter_contours = OL.ObjectLocalizer._filter_contours
    _warn_throttled = OL.ObjectLocalizer._warn_throttled

    def __init__(self,bg):
        tree = ast.parse(Path(OL.__file__).read_text(encoding='utf-8-sig'))
        self.params = {}
        for call in ast.walk(tree):
            if (isinstance(call,ast.Call) and isinstance(call.func,ast.Attribute)
                    and call.func.attr == 'declare_parameter' and len(call.args) > 1):
                self.params[call.args[0].value] = eval(compile(ast.Expression(call.args[1]),'<parameter>','eval'),vars(OL))
        self.params['bg_file'] = str(bg)
        self.H = self.corners = self.origin_px = self.reproj_err = None
        self.reference_corners = self.calibrated_at = None
        self.calibration_moved = False
        self._last_warn = 0.
        self.logs = []

    def get_parameter(self,key):
        return SimpleNamespace(value=self.params[key])

    def get_logger(self):
        return SimpleNamespace(info=self.logs.append,warn=self.logs.append,error=self.logs.append)


def offline_ik(source):
    methods = {'_tip_matrix','_numeric_jacobian','_clip_bounds','_dls_refine','_seed_list','_solve_dls','_orientation_error_deg','_solve_ik','_receive_hardware_limits'}
    tree = ast.parse(Path(source).read_text(encoding='utf-8-sig'))
    cls = next(c for c in tree.body if isinstance(c,ast.ClassDef) and c.name == 'So101IkNode')
    funcs = [f for f in cls.body if isinstance(f,ast.FunctionDef) and f.name in methods]
    namespace = dict(np=np,math=math,JOINT_NAMES=S.ARM_JOINTS+['gripper'],IK_JOINT_NAMES=S.ARM_JOINTS)
    module = ast.Module(body=[ast.ClassDef(name='OfflineIK',bases=[],keywords=[],body=funcs,decorator_list=[])],type_ignores=[])
    exec(compile(ast.fix_missing_locations(module),'<offline native IK>','exec'),namespace)
    node = namespace['OfflineIK']()
    node.chain,names = S.make_chain()
    node.q_target = np.zeros(len(node.chain.links))
    node.q_current = node.q_target.copy()
    node.orientation_mode = 'Z'
    node.max_ik_residual = .003
    node._warn_throttled = lambda message: None
    node.have_driver_status = True
    node._hardware_limits_ready = True
    node._urdf_joint_bounds = {link.name:link.bounds for link in node.chain.links if link.name in S.ARM_JOINTS}
    return node,names


def lowest_jaw(model,data):
    bodies = {mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,n)
              for n in ('gripper_link','moving_jaw_so101_v1_link')}
    bottom = float('inf')
    for i in range(model.ngeom):
        if model.geom_bodyid[i] not in bodies or model.geom_type[i] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mesh = model.geom_dataid[i]
        start,n = model.mesh_vertadr[mesh],model.mesh_vertnum[mesh]
        verts = model.mesh_vert[start:start+n]
        world = verts @ data.geom_xmat[i].reshape(3,3).T + data.geom_xpos[i]
        bottom = min(bottom,float(world[:,2].min()))
    return bottom


def regressions():
    tests = []
    for yaw in (-180,-90,0,90,180):
        for name,fn,order in [('real',REAL.quat_from_RzRx,'wxyz'),
                              ('snapshot',SNAP.quat,'wxyz'),('auto',AUTO.tool_down_quat,'xyzw')]:
            q = fn(yaw)
            xyzw = (q[1],q[2],q[3],q[0]) if order == 'wxyz' else q
            assert np.linalg.norm(rotation(*xyzw)[:,2]-[0,0,-1]) < 1e-10
            tests.append(f'{name}:yaw={yaw}:tool_down')
    for values in ([float('nan'),0,0],[.39,0,0]):
        try:
            finite_position(values)
            raise AssertionError('invalid target accepted')
        except ValueError:
            pass
    tests.append('nonfinite_and_out_of_reach_rejected')
    guard = FeedbackGuard()
    assert not guard.fresh(enabled=True)
    guard.on_joints(SimpleNamespace(name=S.ARM_JOINTS+['gripper'],position=[0.]*6))
    guard.on_status(SimpleNamespace(data='{"mode":"direct","enabled":true,"fault":null}'))
    assert guard.fresh(enabled=True)
    matrix = guard.tcp_matrix()
    chain,_ = S.make_chain()
    assert np.allclose(matrix,chain.forward_kinematics(np.zeros(len(chain.links))))
    tests.append('TCP_derived_from_encoder_feedback_without_TF')
    guard.joints_at -= 2.
    assert not guard.fresh(enabled=True)
    assert guard.tcp_matrix() is None
    guard.on_status(SimpleNamespace(data='{"mode":"sim","enabled":true}'))
    assert not guard.fresh(enabled=True)
    tests.append('physical_feedback_rejects_missing_stale_or_simulated_state')
    assert tool_down_error_deg(SimpleNamespace(x=1.,y=0.,z=0.,w=0.)) < 1e-6
    assert tool_down_error_deg(SimpleNamespace(x=0.,y=0.,z=0.,w=1.)) > 179.
    tests.append('measured_tool_orientation_checked')
    from sim_grasp_validate import Trial,SimulationGuardError
    model = MG.build(.020)
    trial = Trial(model)
    joint = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,'wrist_roll')
    trial.d.qpos[trial.qadr['wrist_roll']] = model.jnt_range[joint,1]+.04
    mujoco.mj_forward(model,trial.d)
    try:
        trial.step()
        raise AssertionError('physics joint-limit guard failed')
    except SimulationGuardError:
        pass
    tests.append('simulation_stops_on_actual_joint_limit_violation')
    with tempfile.TemporaryDirectory(prefix='grasp_readonly_regression_') as tmp:
        observation = Path(tmp)/'pose.txt'
        for valid,cell,age in [('0','3.3',0),('1','nan',0),('1','3.25',0),('1','3.3',10),('1','3.3',-10)]:
            observation.write_text(f'valid={valid}\ncell_cm={cell}\nX_cm=1\nY_cm=2\n')
            os.utime(observation,(time.time()-age,time.time()-age))
            try:
                read_observation(observation)
                raise AssertionError('invalid camera sample accepted')
            except ValueError:
                pass
        observation.write_text('valid=1\ncell_cm=3.3\nX_cm=1\nY_cm=2\n')
        x,y,_ = read_observation(observation)
        assert (x,y) == (.01,.02)
    tests.append('vision_rejects_invalid_scale_stale_and_future_samples')
    # Exercise the real sequence with fake operations, never a real ROS Node.
    saved_node,saved_ros,saved_argv = REAL.Grasp,REAL.rclpy,sys.argv[:]
    try:
        for failed in ('预抓取','抓取点'):
            calls = []
            fake = SimpleNamespace(tcp=lambda:np.array([.3,0,.15]),
                                   enable=lambda enabled:True,wait=lambda sec:None,
                                   grip=lambda value:calls.append(('grip',value)),
                                   goto=lambda target,label:calls.append(('goto',label)) or label != failed,
                                   close=lambda:None)
            REAL.Grasp = lambda dry_run:fake
            REAL.rclpy = SimpleNamespace(init=lambda:None,spin_once=lambda *a,**kw:None,shutdown=lambda:None)
            sys.argv = ['real_grasp_ok.py','--obj','.30','0','-.0394','--board-z','-.0494']
            try:
                with contextlib.redirect_stdout(io.StringIO()):
                    REAL.main()
                raise AssertionError('failed motion continued')
            except RuntimeError:
                assert ('grip',0.) not in calls
                assert ('goto','抬起') not in calls
            tests.append(f'real:{failed}:aborts_before_closure')
        calls.clear()
        sys.argv = ['real_grasp_ok.py','--obj','.30','0','-.0394','--dry-run']
        with contextlib.redirect_stdout(io.StringIO()):
            REAL.main()
        assert not calls
        tests.append('real:dry_run_sends_no_commands')
        sys.argv = ['real_grasp_ok.py','--obj','.30','0','-.0394']
        try:
            REAL.main()
            raise AssertionError('unverified real board height accepted')
        except ValueError:
            assert not calls
        tests.append('real:measured_board_height_required_before_enable')
    finally:
        REAL.Grasp,REAL.rclpy,sys.argv = saved_node,saved_ros,saved_argv
    return tests


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--image',type=Path,required=True)
    ap.add_argument('--background',type=Path,default=Path('/tmp/board_bg.png'))
    ap.add_argument('--report-dir',type=Path,required=True)
    ap.add_argument('--native-ik',type=Path,default=Path.home()/'legacy/arm/arm-final/ros2_ws/src/so101_bringup/so101_bringup/ik_node.py')
    args = ap.parse_args()
    args.report_dir.mkdir(parents=True,exist_ok=True)
    report = dict(read_only=True,regressions=regressions())
    frame = cv2.imread(str(args.image))
    if frame is None:
        raise RuntimeError('input image unreadable')
    replay = Replay(args.background)
    calibrated = replay._calibrate(frame)
    detection = replay._detect_object(frame) if calibrated else None
    report['vision'] = dict(calibrated=calibrated,cell_cm=replay.params['cell_cm'],
                           candidate_found=detection is not None,
                           reason=getattr(replay,'_detection_reason',None),logs=replay.logs)
    if detection is not None:
        x,y,bw,bh,area,contour = detection
        gx,gy = cv2.perspectiveTransform(np.array([[[x,y]]],dtype=float),replay.H)[0,0]
        from grasp_guard import read_kv
        ext = read_kv('/tmp/extrinsic.txt')
        th = math.radians(float(ext['grid_theta_deg']))
        bx = float(ext['grid_origin_x'])+math.cos(th)*gx/100-math.sin(th)*gy/100
        by = float(ext['grid_origin_y'])+math.sin(th)*gx/100+math.cos(th)*gy/100
        report['vision'].update(grid_cm=[float(gx),float(gy)],base_xy_m=[bx,by],
                                reach_mm=1000*math.hypot(bx,by),bbox_px=[bw,bh],area_px=area,
                                mean_reprojection_mm=replay.reproj_err[0]*10)
        cv2.rectangle(frame,(x-bw//2,y-bh//2),(x+bw//2,y+bh//2),(0,255,0),2)
        cv2.putText(frame,f'grid=({gx:.2f},{gy:.2f})cm; cell=33mm',(15,25),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,220,255),2)
        cv2.imwrite(str(args.report_dir/'detection_replay.png'),frame)
        node,names = offline_ik(args.native_ik)
        model = MG.build(.020)
        qadr = S.attach_handles(model)[0]
        cases = []
        for xy in ([.29,-.05],[.33,-.05],[bx,by]):
            target = np.array([xy[0]+.008,xy[1]-.004,-.0394+.002])
            matrix = np.eye(4)
            matrix[:3,:3] = rotation(*AUTO.tool_down_quat(-90))
            matrix[:3,3] = target
            q,err = node._solve_dls(matrix)
            fk = node._tip_matrix(q)
            tilt = math.degrees(math.acos(np.clip(np.dot(fk[:3,2],[0,0,-1]),-1,1)))
            data = mujoco.MjData(model)
            for joint in S.ARM_JOINTS:
                data.qpos[qadr[joint]] = q[names.index(joint)]
            clearances = []
            for angle in (.6,0.):
                data.qpos[qadr['gripper']] = angle
                mujoco.mj_forward(model,data)
                clearances.append((lowest_jaw(model,data)-(S.TABLE_Z+.003))*1000)
            cases.append(dict(target_m=target.tolist(),position_error_mm=err*1000,
                              tool_tilt_deg=tilt,jaw_clearance_open_closed_mm=clearances,
                              accepted=bool(err <= .003 and tilt <= 5.)))
            if hasattr(node,'_orientation_error_deg'):
                original = node.q_target.copy()
                node.target_mat = matrix
                node._solve_ik()
                accepted = not np.array_equal(original,node.q_target)
                assert accepted == cases[-1]['accepted']
                report['regressions'].append(f'native_ik:radius={math.hypot(*target[:2])*1000:.0f}mm:accepted={accepted}')
        report['native_ik_geometry'] = cases
        if hasattr(node,'_receive_hardware_limits'):
            config = yaml.safe_load((Path.home()/'legacy/arm/arm-final/ros2_ws/src/so101_bringup/config/driver_params.yaml').read_text())['so101_driver']['ros__parameters']
            values = [SimpleNamespace(string_value='direct')]+[SimpleNamespace(integer_array_value=config[k]) for k in ('zero_raw','direction','raw_min','raw_max')]
            node._receive_hardware_limits(SimpleNamespace(result=lambda:SimpleNamespace(values=values)))
            assert node._hardware_limits_ready
            report['effective_driver_bounds_rad'] = {link.name:list(link.bounds) for link in node.chain.links if link.name in S.ARM_JOINTS}
            expected = (4095-3053)*2*math.pi/4096
            assert abs(report['effective_driver_bounds_rad']['wrist_roll'][1]-expected) < 1e-9
            report['regressions'].append('live_driver_raw_limits_intersect_urdf')
            hardware_cases = []
            for case in cases:
                node.q_target[:] = 0.
                matrix = np.eye(4)
                matrix[:3,:3] = rotation(*AUTO.tool_down_quat(-90))
                matrix[:3,3] = case['target_m']
                q,error = node._solve_dls(matrix)
                angle = node._orientation_error_deg(q,matrix)
                hardware_cases.append(dict(target_m=case['target_m'],position_error_mm=error*1000,
                                           tool_tilt_deg=angle,accepted=bool(error <= .003 and angle <= 5.)))
                assert all(link.bounds[0]-1e-9 <= q[i] <= link.bounds[1]+1e-9 for i,link in enumerate(node.chain.links))
            report['driver_limited_ik'] = hardware_cases
            report['regressions'].append('hardware_ik_solutions_respect_effective_bounds')
    try:
        require_fresh('/tmp/obj_base.txt')
        report['old_snapshot']='fresh'
    except (OSError,ValueError) as exc:
        report['old_snapshot']=str(exc)
    report['original_bad_tool_z_at_yaw_minus90'] = rotation(math.sqrt(.5),0,-math.sqrt(.5),0)[:,2].tolist()
    report['board_geometry'] = dict(width_mm=S.BOARD_W*1000,height_mm=S.BOARD_H*1000,
                                   grid_origin='first inner corner',outer_grid_min_mm=-33.)
    (args.report_dir/'audit.json').write_text(json.dumps(report,indent=2,ensure_ascii=False))
    print(json.dumps(report,ensure_ascii=False),flush=True)


if __name__ == '__main__':
    main()
