#!/usr/bin/env python3
"""Inspect the existing SO101 mesh at the last recorded calibration pose.

No ROS publisher, serial access, IK, or physical stepping is used.
Run on the VM: ~/mj/bin/python view_arm_model.py --board
Keys: 1 overview, 2 jaws, 3 top. Mouse: rotate/pan/zoom.
"""
import argparse
import json
import math
import os
from pathlib import Path
import threading
import time

import mujoco
import numpy as np

JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
          'wrist_flex', 'wrist_roll', 'gripper']
URDF = os.path.expanduser('~/legacy/arm/arm-final/ros2_ws/install/'
                         'so101_bringup/share/so101_bringup/urdf/so101.urdf')
TABLE_Z = -0.06909


def build(args):
    marks = json.loads(Path(args.marks).read_text())
    spec = mujoco.MjSpec.from_file(URDF)
    spec.visual.global_.offwidth = 1400
    spec.visual.global_.offheight = 1000
    wb = spec.worldbody

    def box(name, pos, size, color, quat=(1, 0, 0, 0)):
        g = wb.add_geom()
        g.name, g.type = name, mujoco.mjtGeom.mjGEOM_BOX
        g.pos, g.size, g.rgba, g.quat = pos, size, color, quat
        return g

    box('inspection_table', [0.22, 0, TABLE_Z - .025], [.36, .29, .025],
        [.48, .49, .52, 1])
    box('inspection_pedestal', [0, 0, (TABLE_Z - .0024) / 2],
        [.045, .05, (-.0024 - TABLE_Z) / 2], [.24, .25, .28, 1])
    ext = {}
    if args.board:
        for line in Path(args.extrinsic).read_text().splitlines():
            if '=' in line and not line.startswith('#'):
                k, v = line.split('=', 1)
                ext[k] = float(v)
        yaw = math.radians(ext['grid_theta_deg'])
        rot = np.array([[math.cos(yaw), -math.sin(yaw)],
                        [math.sin(yaw), math.cos(yaw)]])
        origin = np.array([ext['grid_origin_x'], ext['grid_origin_y']])
        cell = ext['cell_cm'] / 100
        quat = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        # First INNER corner is the origin; physical board has 8 x 6 cells.
        for row in range(6):
            for col in range(8):
                p = origin + rot @ np.array([(col - .5) * cell, (row - .5) * cell])
                color = [.85, .83, .76, 1] if (row + col) % 2 else [.10, .11, .13, 1]
                box(f'inspection_cell_{row}_{col}', [*p, TABLE_Z + .00025],
                    [cell / 2, cell / 2, .00025], color, quat)
        for i, m in enumerate(marks):
            g = wb.add_geom()
            g.name, g.type = f'inspection_mark_{i}', mujoco.mjtGeom.mjGEOM_SPHERE
            g.pos, g.size, g.rgba = m['contact_m'], [.003, 0, 0], [1, .12, .1, 1]
    model = spec.compile()
    data = mujoco.MjData(model)
    for name in JOINTS:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise ValueError(f'Missing joint {name}')
        data.qpos[model.jnt_qposadr[jid]] = marks[-1]['joints'][name]
    mujoco.mj_forward(model, data)
    jaw_center = np.array(marks[-1]['contact_m']) + [0, 0, .035]
    print(json.dumps({'source': URDF, 'pose': 'last saved calibration mark',
                      'joints': marks[-1]['joints'], 'board_shown': args.board,
                      'extrinsic_quality_ok': ext.get('quality_ok'),
                      'extrinsic_rms_mm': ext.get('rms_mm')}, indent=2), flush=True)
    return model, data, jaw_center


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--marks', default='/tmp/extrinsic_marks_merged.json')
    ap.add_argument('--extrinsic', default='/tmp/extrinsic.txt')
    ap.add_argument('--board', action='store_true', help='Show unverified board placement')
    ap.add_argument('--render-dir')
    args = ap.parse_args()
    model, data, jaw_center = build(args)
    views = {'overview': ([.15, -.02, .025], .72, 135, -28),
             'jaws': (jaw_center, .24, 125, -15),
             'top': ([.16, -.01, -.025], .75, 90, -89)}

    def camera(cam, name):
        lookat, distance, azimuth, elevation = views[name]
        cam.lookat[:], cam.distance = lookat, distance
        cam.azimuth, cam.elevation = azimuth, elevation

    if args.render_dir:
        from PIL import Image
        out = Path(args.render_dir)
        out.mkdir(parents=True, exist_ok=True)
        with mujoco.Renderer(model, height=900, width=1200) as renderer:
            for name in views:
                cam = mujoco.MjvCamera()
                camera(cam, name)
                renderer.update_scene(data, camera=cam)
                Image.fromarray(renderer.render()).save(out / f'{name}.png')
                print(f'Rendered {out / (name + ".png")}', flush=True)
        return

    from mujoco import viewer as mjviewer
    state = {'view': 'overview'}
    def key(code):
        if code in (49, 50, 51):
            state['view'] = {49: 'overview', 50: 'jaws', 51: 'top'}[code]
    prior = set(threading.enumerate())
    with mjviewer.launch_passive(model, data, key_callback=key) as viewer:
        active = None
        print('MODEL VIEWER OPEN: 1 overview / 2 jaws / 3 top', flush=True)
        while viewer.is_running():
            if active != state['view']:
                with viewer.lock():
                    camera(viewer.cam, state['view'])
                active = state['view']
            viewer.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_150,
                               mujoco.mjtGridPos.mjGRID_TOPLEFT,
                               'SO101 MODEL INSPECTION\nLast saved pose\n1: overview  2: jaws  3: top',
                               'BOARD EXTRINSIC: UNVERIFIED' if args.board else '')])
            viewer.sync()
            time.sleep(1 / 30)
    for thread in threading.enumerate():
        if thread not in prior and thread is not threading.current_thread():
            thread.join(timeout=10)


if __name__ == '__main__':
    main()
