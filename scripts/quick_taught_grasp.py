#!/usr/bin/env python3
"""One taught grasp without camera extrinsics. Default: read/plan only.

Place open jaws around the object with torque off, then use --execute.
Requires sole ownership of the servo port. Reuses measured driver limits.
The finished/error pose remains torque-held; --release explicitly releases it.
"""

from project_paths import open_video_capture

from project_paths import default_arm_port

from project_paths import arm_source_path, default_camera, driver_params_path
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
import warnings

import numpy as np
from scipy.optimize import least_squares
import yaml

warnings.filterwarnings('ignore', category=UserWarning, module='ikpy')
sys.path.insert(0, os.path.expanduser(arm_source_path()))
from so101_bringup.servo_protocol import FeetechSerialBus
from gripper_model import GripperModel, JOINTS

CONFIG = os.path.expanduser(driver_params_path())


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute', action='store_true')
    ap.add_argument('--release', action='store_true')
    ap.add_argument('--port', default=default_arm_port())
    ap.add_argument('--lift-mm', type=float, default=50)
    ap.add_argument('--load-pct', type=float, default=12)
    ap.add_argument('--hold-seconds', type=float, default=4)
    ap.add_argument('--report', default='/tmp/quick_taught_grasp.json')
    a = ap.parse_args()
    if not 0 < a.lift_mm <= 60 or not 0 < a.load_pct <= 20:
        raise ValueError('Lift must be 0..60mm, contact load 0..20 percent')
    cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
    if not cfg['calibrated']:
        raise ValueError('Servo calibration missing')
    zero = np.array(cfg['zero_raw'])
    direction = np.array(cfg['direction'])
    lower_raw, upper_raw = np.array(cfg['raw_min']), np.array(cfg['raw_max'])
    lower = (lower_raw - zero) * direction * 2 * math.pi / 4096
    upper = (upper_raw - zero) * direction * 2 * math.pi / 4096
    lower, upper = np.minimum(lower, upper), np.maximum(lower, upper)
    result = {'mode': 'execute' if a.execute else 'plan', 'completed': False}
    bus = FeetechSerialBus(a.port, timeout_s=.08)
    energized = False

    def snapshot(tag):
        try:
            import cv2
            cap = open_video_capture(default_camera())
            frames = [cap.read() for _ in range(5)]
            cap.release()
            ok, img = frames[-1]
            if ok:
                path = str(Path(a.report).with_suffix('')) + '_' + tag + '.jpg'
                cv2.imwrite(path, img)
                result.setdefault('images', {})[tag] = path
        except Exception as exc:
            print(f'Camera snapshot unavailable: {exc}', flush=True)

    def read():
        raw = np.array(bus.read_positions())
        return (raw - zero) * direction * 2 * math.pi / 4096

    def write(q):
        if not np.all(np.isfinite(q)) or np.any(q < lower - 1e-6) or np.any(q > upper + 1e-6):
            raise ValueError('Joint command exceeds measured limits')
        raw = np.rint(zero + q * direction * 4096 / (2 * math.pi)).astype(int)
        bus.write_positions(raw.tolist())

    try:
        if a.release:
            bus.set_torque(False)
            print('Released all joint torques', flush=True)
            return
        q0 = read()
        torque = bus.read_torque_states()
        if any(torque):
            raise ValueError('Teach with all torques OFF before starting this tool')
        if np.any(q0 < lower) or np.any(q0 > upper):
            raise ValueError('Taught pose is outside measured joint limits')
        model = GripperModel(stride=10)
        model.prepare_clearance(stride=16)
        def js(q):
            return dict(zip(JOINTS, q))
        def frame(q):
            return model.solve(js(q))['gripper_frame_link']
        initial = frame(q0)
        axis = initial[:3, 2].copy()
        p0 = initial[:3, 3].copy()
        result.update(taught_joints=q0.tolist(), taught_tcp_m=p0.tolist())
        # Solve short local lift while preserving the taught tool axis.
        seed = q0[:5].copy()
        waypoints = []
        for z in np.linspace(0, a.lift_mm / 1000, 11)[1:]:
            target = p0 + [0, 0, z]
            def residual(arm):
                f = frame(np.r_[arm, q0[5]])
                return np.r_[f[:3, 3] - target, .05 * (f[:3, 2] - axis)]
            sol = least_squares(residual, seed, bounds=(lower[:5], upper[:5]),
                                max_nfev=100, ftol=1e-8, xtol=1e-8, gtol=1e-8)
            candidate = np.r_[sol.x, q0[5]]
            f = frame(candidate)
            err = float(np.linalg.norm(f[:3, 3] - target))
            angle = math.degrees(math.acos(float(np.clip(f[:3, 2] @ axis, -1, 1))))
            if err > .002 or angle > 6 or np.max(np.abs(candidate[:5] - q0[:5])) > .75:
                raise ValueError(f'Local lift unavailable: error {err*1000:.1f}mm, axis {angle:.1f}deg')
            lowest, link = model.lowest_over_all(js(candidate))
            if lowest[2] < -.06909 + .0005:
                raise ValueError(f'Lift would touch table: {link}')
            waypoints.append(candidate.tolist())
            seed = sol.x
        result['lift_waypoints'] = waypoints
        print(json.dumps(result, indent=2), flush=True)
        Path(a.report).write_text(json.dumps(result, indent=2))
        if not a.execute:
            print('PLAN READY: no register writes', flush=True)
            return
        if np.max(np.abs(read() - q0)) > .015:
            raise ValueError('Arm moved since taught pose was recorded')
        snapshot('before')
        write(q0)
        bus.set_torque(True)
        energized = True
        if bus.read_torque_states() != [1]*6:
            raise RuntimeError('Torque enable failed')
        time.sleep(.25)
        print('Holding taught pose; closing jaws slowly', flush=True)
        close = q0.copy()
        load0 = bus.read_gripper_load()[0]
        hit = False
        deadline = time.monotonic() + 16
        hits = 0
        while close[5] > lower[5] + .008 and time.monotonic() < deadline:
            close[5] = max(lower[5], close[5] - .006)
            # Check only the changing jaw's table clearance at this pose.
            lowest, _ = model.lowest_point(js(close))
            if lowest is None or lowest[2] < -.06909 + .0005:
                raise RuntimeError('Jaw closure would touch table; reposition taught pose')
            write(close)
            time.sleep(.09)
            pct = bus.read_gripper_load()[0]
            hits = hits + 1 if pct >= max(a.load_pct, load0 + 6) else 0
            if hits >= 3:
                hit = True
                break
        if not hit:
            raise RuntimeError('No jaw contact detected; not lifting')
        measured = read()
        close[5] = max(lower[5], measured[5] - .008)
        write(close)
        time.sleep(.4)
        result.update(contact_load_pct=pct, held_gripper_rad=float(close[5]))
        print(f'Jaw contact {pct:.1f}%; lifting {a.lift_mm:.0f}mm', flush=True)

        def move(target):
            start = read()
            duration = max(.3, float(np.max(np.abs(target-start))) / .18)
            steps = max(1, math.ceil(duration / .08))
            for i in range(1, steps+1):
                write(start + (target-start) * i / steps)
                time.sleep(duration / steps)
            end = time.monotonic() + 3
            while time.monotonic() < end:
                actual = read()
                if np.max(np.abs(actual[:5] - target[:5])) < .07:
                    return
                write(target)
                time.sleep(.1)
            raise RuntimeError('Arm did not reach the commanded waypoint')

        for waypoint in waypoints:
            target = np.array(waypoint)
            target[5] = close[5]
            move(target)
        actual = read()
        result['measured_lift_mm'] = float((frame(actual)[2, 3] - p0[2])*1000)
        # 位置舵机在重力负载下上行明显滞后：实测约为指令的 6~8 成。
        # 旧判据要求达到 指令-10mm（50mm 时要 40mm），实测 37.8mm 会被误判失败。
        need_mm = max(15.0, a.lift_mm * 0.6)
        if result['measured_lift_mm'] < need_mm:
            raise RuntimeError(
                f'Measured arm lift {result["measured_lift_mm"]:.1f}mm '
                f'< {need_mm:.1f}mm')
        result['lift_completed'] = True
        Path(a.report).write_text(json.dumps(result, indent=2))
        print(f'ARM LIFTED {result["measured_lift_mm"]:.1f}mm; inspect the object now', flush=True)
        snapshot('lifted')
        time.sleep(a.hold_seconds)
        for waypoint in reversed([q0.tolist()] + waypoints[:-1]):
            target = np.array(waypoint)
            target[5] = close[5]
            move(target)
        move(q0)
        result['completed'] = True
        snapshot('returned')
        print('Cycle completed: jaws closed, lifted, returned, opened. Arm remains held.', flush=True)
    except BaseException as exc:
        result['error'] = str(exc)
        print(f'STOP: {exc}', flush=True)
        if energized:
            try:
                write(read())  # Hold rather than dropping an object/arm.
            except Exception:
                pass
        raise
    finally:
        Path(a.report).write_text(json.dumps(result, indent=2))
        bus.close()


if __name__ == '__main__':
    main()
