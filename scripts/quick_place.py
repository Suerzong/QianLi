#!/usr/bin/env python3
"""Short relative carry motions from the current held pose; no extrinsics.

Default plans only. --execute carries and holds. --open opens the jaws only.
"""

from project_paths import open_video_capture

from project_paths import default_arm_port

from project_paths import default_camera
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.optimize import least_squares
import yaml
from quick_taught_grasp import FeetechSerialBus, GripperModel, JOINTS, CONFIG


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--delta-mm', nargs=3, type=float, default=[80, -10, 40])
    ap.add_argument('--execute', action='store_true')
    ap.add_argument('--open', action='store_true')
    ap.add_argument('--report', default='/tmp/quick_place.json')
    a = ap.parse_args()
    delta = np.array(a.delta_mm)/1000
    if np.linalg.norm(delta) > .18 or delta[2] < 0:
        raise ValueError('Carry must be a short motion with nonnegative lift')
    cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
    zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
    lo = (np.array(cfg['raw_min'])-zero)*direction*2*math.pi/4096
    hi = (np.array(cfg['raw_max'])-zero)*direction*2*math.pi/4096
    lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)
    bus = FeetechSerialBus(default_arm_port(), timeout_s=.08)
    result = {'mode': 'execute' if a.execute else 'plan'}
    def read():
        return (np.array(bus.read_positions())-zero)*direction*2*math.pi/4096
    def write(q):
        if np.any(q < lo-1e-6) or np.any(q > hi+1e-6):
            raise ValueError('Measured limits exceeded')
        bus.write_positions(np.rint(zero+q*direction*4096/(2*math.pi)).astype(int).tolist())
    def picture(tag):
        import cv2
        cap = open_video_capture(default_camera())
        frames = [cap.read() for _ in range(5)]
        cap.release()
        ok, img = frames[-1]
        if ok:
            path = str(Path(a.report).with_suffix(''))+'_'+tag+'.jpg'
            cv2.imwrite(path, img)
            result.setdefault('images', {})[tag] = path
    try:
        q0 = read()
        if bus.read_torque_states() != [1]*6:
            raise ValueError('Carry requires the existing held pose')
        if a.open:
            for g in np.linspace(q0[5], max(q0[5], .65), 30):
                q = q0.copy(); q[5] = g
                write(q); time.sleep(.09)
            picture('opened')
            result['jaws_opened'] = True
            print('Jaws opened; arm stays held', flush=True)
            return
        model = GripperModel(stride=12)
        def frame(q):
            return model.solve(dict(zip(JOINTS, q)))['gripper_frame_link']
        f0 = frame(q0)
        p0, axis = f0[:3, 3].copy(), f0[:3, 2].copy()
        def solve(target, seed):
            def residual(q):
                f = frame(np.r_[q, seed[5]])
                return np.r_[f[:3, 3]-target, .04*(f[:3, 2]-axis), .002*(q[4]-q0[4])]
            sol = least_squares(residual, seed[:5], bounds=(lo[:5], hi[:5]), max_nfev=100)
            q = np.r_[sol.x, seed[5]]
            if np.linalg.norm(frame(q)[:3, 3]-target) > .004:
                raise ValueError('Carry position is outside local reach')
            lowest, link = model.lowest_over_all(dict(zip(JOINTS, q)))
            if lowest[2] < -.06909+.002:
                raise ValueError(f'Carry too low: {link}')
            return q
        up = p0+[0,0,delta[2]]
        targets = [p0+(up-p0)*s for s in np.linspace(.1,1,10)]
        targets += [up+np.array([delta[0],delta[1],0])*s for s in np.linspace(.1,1,10)]
        seed = q0.copy()
        for target in targets:
            seed = solve(target, seed)
        result.update(start_m=p0.tolist(), target_m=(p0+delta).tolist())
        print(json.dumps(result), flush=True)
        if not a.execute:
            print('CARRY PLAN READY: no writes', flush=True)
            return
        # Restore a small squeeze; stopping a motor at its encoder position
        # after contact would remove its position error and gripping force.
        grip = max(lo[5], q0[5]-.025)
        q = q0.copy(); q[5] = grip
        write(q); time.sleep(.5)
        load = bus.read_gripper_load()[0]
        result['gripper_load_pct'] = load
        if load < 3:
            raise ValueError('No sustained gripping load; object may be absent')
        print(f'Holding object, gripper load {load:.1f}%', flush=True)
        for i,target in enumerate(targets):
            actual = read(); actual[5] = grip
            goal = solve(target, actual); goal[5] = grip
            # Brief slow interpolation, then encoder-based correction for sag.
            start = read()
            steps = max(4, math.ceil(float(np.max(np.abs(goal-start)))/(.18*.08)))
            for k in range(1,steps+1):
                write(start+(goal-start)*k/steps); time.sleep(.08)
            time.sleep(.2)
            for _ in range(3):
                measured = read()
                err = float(np.linalg.norm(frame(measured)[:3,3]-target))
                if err < .008:
                    break
                correction = np.clip((goal-measured)*.6, -.025, .025)
                correction[5] = 0
                goal = np.clip(goal+correction,lo,hi)
                write(goal); time.sleep(.25)
            if i == 9:
                picture('raised')
        result['actual_m'] = frame(read())[:3,3].tolist()
        picture('carried')
        result['carried'] = True
        print('Carry completed; jaws remain closed', flush=True)
    except BaseException as exc:
        result['error'] = str(exc)
        try:
            picture('stopped')
        except Exception:
            pass
        print(f'STOP: {exc}', flush=True)
        raise
    finally:
        Path(a.report).write_text(json.dumps(result, indent=2))
        bus.close()


if __name__ == '__main__':
    main()
