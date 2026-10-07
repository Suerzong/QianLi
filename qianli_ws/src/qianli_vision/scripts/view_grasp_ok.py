#!/usr/bin/env python3
"""Display the exact pipeline used by headless grasp acceptance.
DISPLAY=:0 MUJOCO_GL=glfw ~/mj/bin/python view_grasp_ok.py
Left drag rotates, right drag pans, scroll zooms. Pure simulation.
"""
import argparse
import json
import time
import threading
import mujoco
import mujoco.viewer
import numpy as np
import sim_grasp as S
from sim_grasp_validate import Trial, build, DEFAULT_OFFSET, DEFAULT_GRID, DEFAULT_APPROACH


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--obj-size', type=float, default=.020)
    ap.add_argument('--offset', type=float, nargs=3, default=DEFAULT_OFFSET)
    ap.add_argument('--approach', type=float, default=DEFAULT_APPROACH)
    ap.add_argument('--retries', type=int, choices=(0,1,2), default=2)
    ap.add_argument('--object-grid-mm', type=float, nargs=2, default=DEFAULT_GRID)
    ap.add_argument('--loops', type=int, default=0, help='0 loops until window closes.')
    args = ap.parse_args()
    S.OBJ_GRID = tuple(np.asarray(args.object_grid_mm)/1000.)
    model = build(args.obj_size)
    data = mujoco.MjData(model)
    existing_threads = set(threading.enumerate())
    with mujoco.viewer.launch_passive(model, data) as viewer:
        with viewer.lock():
            viewer.cam.lookat[:] = [.24, -.03, .04]
            viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = .63, 125, -18
        frame_steps = max(1, round(1/(60*model.opt.timestep)))
        step_count = 0
        deadline = time.monotonic()
        def step(trial):
            nonlocal step_count, deadline
            if not viewer.is_running():
                raise KeyboardInterrupt
            step_count += 1
            if step_count % frame_steps == 0:
                viewer.sync()
                deadline += frame_steps*model.opt.timestep
                time.sleep(max(0., deadline-time.monotonic()))
        count = 0
        try:
            while viewer.is_running() and (not args.loops or count < args.loops):
                result = Trial(model, data, on_step=step).execute(args.offset, args.approach, args.retries)
                print(json.dumps(result), flush=True)
                count += 1
        except KeyboardInterrupt:
            pass
    # Handle.close() requests exit asynchronously. Wait for this launch's
    # daemon renderer to release GL resources before Python shuts down.
    for thread in threading.enumerate():
        if thread not in existing_threads and thread is not threading.current_thread():
            thread.join(timeout=10.)


if __name__ == '__main__':
    main()
