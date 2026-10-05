#!/usr/bin/env python3
"""MuJoCo grasp acceptance with environment collision and sustained holding.

Pure simulation: no ROS imports, publishers, or servo/serial connections.
"""
import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np

import sim_grasp as S
import sim_ik_dls as IK
import sim_mesh_gripper as MG


def build(size, mass=.008, friction=1., timestep=.002):
    spec = MG.make_spec(size)
    spec.geom('cube').mass = mass
    spec.option.timestep = timestep
    for geom in spec.geoms:
        if geom.name == 'cube' or geom.name.startswith('pg'):
            geom.friction = [friction, .005, .0001]
    return spec.compile()


class Trial:
    def __init__(self, model, data=None, on_step=None, on_stage=None):
        self.m = model
        self.d = data if data is not None else mujoco.MjData(model)
        self.on_step = on_step
        self.on_stage = on_stage
        self.qadr, self.aadr, self.obj, self.oq = S.attach_handles(model)
        self.cube = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
        self.env = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n)
                    for n in ('table', 'board', 'pedestal')}
        self.jaws = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
                     for n in ('gripper_link', 'moving_jaw_so101_v1_link')}
        self.penetration = 0.
        self.peak_speed = 0.
        self.peak_arm_speed = 0.
        self.max_limit_error = 0.
        self.hinge = np.flatnonzero(model.jnt_type == mujoco.mjtJoint.mjJNT_HINGE)
        self.hq = model.jnt_qposadr[self.hinge]
        self.hv = model.jnt_dofadr[self.hinge]
        self.qlo, self.qhi = model.jnt_range[self.hinge].T

    def tcp(self):
        body = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, 'gripper_link')
        return self.d.xpos[body] + self.d.xmat[body].reshape(3, 3) @ S.FRAME_IN_GRIPPER

    def step(self):
        mujoco.mj_step(self.m, self.d)
        if not np.isfinite(self.d.qpos).all() or not np.isfinite(self.d.qvel).all():
            raise RuntimeError('nonfinite physics state')
        self.peak_speed = max(self.peak_speed, float(np.abs(self.d.qvel[self.hv]).max()))
        self.peak_arm_speed = max(self.peak_arm_speed, float(np.abs(self.d.qvel[self.hv[:-1]]).max()))
        q = self.d.qpos[self.hq]
        self.max_limit_error = max(self.max_limit_error,
                                  float(np.maximum(self.qlo-q, q-self.qhi).max()))
        for con in self.d.contact:
            g1, g2 = int(con.geom1), int(con.geom2)
            if ((g1 in self.env and self.m.geom_bodyid[g2] != 0 and g2 != self.cube)
                    or (g2 in self.env and self.m.geom_bodyid[g1] != 0 and g1 != self.cube)):
                self.penetration = max(self.penetration, -float(con.dist))
        if self.on_step is not None:
            self.on_step(self)

    def stage(self, label):
        if self.on_stage is not None:
            self.on_stage(label, self)

    def contacts(self):
        sides = set()
        force = np.zeros(6)
        for i, con in enumerate(self.d.contact):
            if self.cube not in (con.geom1, con.geom2):
                continue
            other = con.geom2 if con.geom1 == self.cube else con.geom1
            body = int(self.m.geom_bodyid[other])
            mujoco.mj_contactForce(self.m, self.d, i, force)
            if body in self.jaws and force[0] > 0.02:
                sides.add(body)
        return len(sides) == 2

    def move(self, target, grip, speed=0.3, yaw=-90.):
        q, residual = IK.solve_only(self.m, self.qadr, target, self.d.qpos.copy(), yaw)
        if residual > .002:
            return float(residual)
        goals = np.asarray(q + [grip])
        idx = np.array([self.aadr[j] for j in S.ALL_JOINTS])
        initial = self.d.ctrl[idx].copy()
        steps = max(1, int(np.ceil(np.max(np.abs(goals-initial)) / speed / self.m.opt.timestep)))
        for i in range(steps):
            self.d.ctrl[idx] = initial + (goals-initial) * ((i+1) / steps)
            self.step()
        for _ in range(600):
            self.step()
        return float(np.linalg.norm(self.tcp()-target))

    def run(self, offset, approach=.6, hold=2., obj_offset=(0., 0.), yaw=0.,
            perception_error=(0., 0.), reset=True):
        if reset:
            mujoco.mj_resetData(self.m, self.d)
            op = S.obj_world_pos().copy()
            op[:2] += np.asarray(obj_offset)
            self.d.qpos[self.oq:self.oq+3] = op
            self.d.qpos[self.oq+3:self.oq+7] = [math.cos(yaw/2), 0., 0., math.sin(yaw/2)]
            mujoco.mj_forward(self.m, self.d)
        else:
            # Retry on the actual resulting physical state. Never reset the
            # object or rewind physics after a failed grasp.
            op = self.d.xpos[self.obj].copy()
        target = op + np.asarray(offset) / 1000
        target[:2] += perception_error
        e1 = self.move(target + [0, 0, .06], approach)
        # Initial setup only: baseline settled object position before descent.
        obj0 = self.d.xpos[self.obj].copy()
        self.stage('pregrasp')
        if e1 >= .004:
            return dict(success=False, failed_stage='pregrasp', offset_mm=list(offset), tcp_error_mm=e1*1000)
        e2 = self.move(target, approach)
        self.stage('descend')
        if e2 >= .004:
            return dict(success=False, failed_stage='descend', offset_mm=list(offset), tcp_error_mm=e2*1000)
        grip_id = self.aadr['gripper']
        initial_grip = float(self.d.ctrl[grip_id])
        nclose = max(1, math.ceil(abs(initial_grip)/(.3*self.m.opt.timestep)))
        for i in range(nclose):
            self.d.ctrl[grip_id] = initial_grip*(1-(i+1)/nclose)
            self.step()
        for _ in range(1000):
            self.step()
        self.stage('closed')
        e3 = self.move(target + [0, 0, .12], 0.)
        if e3 >= .004:
            return dict(success=False, failed_stage='lift', offset_mm=list(offset),
                        tcp_error_mm=e3*1000, penetration_mm=self.penetration*1000,
                        joint_limit_error_rad=self.max_limit_error)
        minimum_lift = float('inf')
        held = 0
        n = max(1, int(hold/self.m.opt.timestep))
        for _ in range(n):
            self.step()
            minimum_lift = min(minimum_lift, float(self.d.xpos[self.obj][2]-obj0[2]))
            held += int(self.contacts())
        up = float(self.d.xpos[self.obj][2]-obj0[2])
        self.stage('held')
        ok = (minimum_lift >= .05 and held/n >= .95 and max(e1,e2,e3) < .004
              and self.penetration < .001 and self.max_limit_error < .01)
        return dict(success=bool(ok), offset_mm=list(offset), approach=approach,
                    obj_offset_mm=(np.asarray(obj_offset)*1000).tolist(), yaw_deg=math.degrees(yaw),
                    perception_error_mm=(np.asarray(perception_error)*1000).tolist(),
                    lift_mm=up*1000, minimum_hold_lift_mm=minimum_lift*1000,
                    hold_contact_fraction=held/n, tcp_error_mm=(np.array([e1,e2,e3])*1000).tolist(),
                    penetration_mm=self.penetration*1000, peak_joint_speed=self.peak_speed,
                    peak_arm_speed=self.peak_arm_speed,
                    joint_limit_error_rad=self.max_limit_error)

    def release(self, approach):
        """Open in place with a limited command ramp, then let the object settle."""
        aid = self.aadr['gripper']
        start = float(self.d.ctrl[aid])
        count = max(1, math.ceil(abs(approach-start)/(.3*self.m.opt.timestep)))
        for i in range(count):
            self.d.ctrl[aid] = start + (approach-start)*(i+1)/count
            self.step()
        for _ in range(math.ceil(1.5/self.m.opt.timestep)):
            self.step()

    def execute(self, offset, approach=.6, retries=2, **settings):
        """Bounded feedback retries using simulated object observations.

        Initial pose randomization happens once. Retry observations use the
        actual object pose plus the same configured perception error.
        """
        attempts = []
        candidates = [np.asarray(offset, dtype=float),
                      np.asarray(offset, dtype=float) + [0., 0., -2.],
                      np.asarray(offset, dtype=float) + [2., -2., 0.]]
        for i, candidate in enumerate(candidates[:retries+1]):
            if i:
                self.release(approach)
            result = self.run(candidate.tolist(), approach, reset=(i == 0), **settings)
            attempts.append(result)
            if result['success'] or result.get('failed_stage'):
                break
            if self.penetration >= .001 or self.max_limit_error >= .01:
                break
        output = dict(attempts[-1])
        output['attempts'] = attempts
        output['attempt_count'] = len(attempts)
        return output


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--obj-size', type=float, default=.020)
    ap.add_argument('--mass-g', type=float, default=8.)
    ap.add_argument('--friction', type=float, default=1.)
    ap.add_argument('--timestep', type=float, default=.002)
    ap.add_argument('--offset', type=float, nargs=3, default=[8., -4., 8.])
    ap.add_argument('--approach', type=float, default=.6)
    ap.add_argument('--object-grid-mm', type=float, nargs=2, default=[111., 20.],
                    help='Object center in board coordinates; keep the whole cube supported.')
    ap.add_argument('--repeat', type=int, default=1)
    ap.add_argument('--sweep', action='store_true')
    ap.add_argument('--robust', type=int, default=0,
                    help='Seeded trials: position +/-5mm, yaw 0..90deg, perception +/-2mm.')
    ap.add_argument('--seed', type=int, default=101)
    ap.add_argument('--retries', type=int, choices=(0,1,2), default=2,
                    help='Feedback retries on the same physical state; no object resets.')
    ap.add_argument('--render-dir', type=Path, help='Save physical states as PNGs (requires GL).')
    ap.add_argument('--report', type=Path)
    args = ap.parse_args()
    if (not np.isfinite(args.offset).all() or args.obj_size <= 0 or args.repeat < 1
            or args.robust < 0 or args.mass_g <= 0 or args.friction < 0
            or not 0 < args.timestep <= .002 or not 0 < args.approach < 1.745):
        ap.error('finite offsets, positive size/mass/repeat, valid timestep/angle required')
    half = args.obj_size*1000/2
    grid = np.asarray(args.object_grid_mm)
    if np.any(grid < half) or np.any(grid > np.array([S.BOARD_W,S.BOARD_H])*1000-half):
        ap.error('object must start completely supported by the board')
    S.OBJ_GRID = tuple(np.asarray(args.object_grid_mm)/1000.)
    model = build(args.obj_size, args.mass_g/1000, args.friction, args.timestep)
    offsets = ([x,y,z] for z in (2,4,6,8) for x in (4,8,12) for y in (-8,-4,0)) if args.sweep else [args.offset]*(args.robust or args.repeat)
    rows = []
    rng = np.random.default_rng(args.seed)
    renderer = None
    capture = None
    if args.render_dir:
        import cv2
        args.render_dir.mkdir(parents=True, exist_ok=True)
        renderer = mujoco.Renderer(model, 480, 640)
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [.24, -.03, .04]
        camera.distance, camera.azimuth, camera.elevation = .63, 125, -18
        def capture(label, trial):
            renderer.update_scene(trial.d, camera)
            frame = renderer.render()
            cv2.putText(frame, label, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, .9, (230,230,230), 2)
            cv2.imwrite(str(args.render_dir / f'{label}.png'), frame[:, :, ::-1])
    for offset in offsets:
        settings = (dict(obj_offset=rng.uniform(-.005,.005,2),
                         yaw=rng.uniform(0,math.pi/2),
                         perception_error=rng.uniform(-.002,.002,2)) if args.robust else {})
        row = Trial(model, on_stage=capture).execute(offset, args.approach, args.retries, **settings)
        rows.append(row)
        print(json.dumps(row), flush=True)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(dict(mujoco_version=mujoco.__version__,
                                              criteria=dict(minimum_lift_mm=50, hold_seconds=2,
                                                            two_sided_contact_fraction=.95,
                                                            tcp_tolerance_mm=4, penetration_tolerance_mm=1),
                                              physics=dict(mass_g=args.mass_g, friction=args.friction,
                                                           timestep=args.timestep, torque_arm_nm=3,
                                                           torque_gripper_nm=1.5),
                                              seed=args.seed, retries=args.retries,
                                              object_grid_mm=args.object_grid_mm,
                                              trials=rows, successes=sum(r['success'] for r in rows)), indent=2))
    if renderer is not None:
        renderer.close()
    return 0 if all(r['success'] for r in rows) else 1


if __name__ == '__main__':
    raise SystemExit(main())
