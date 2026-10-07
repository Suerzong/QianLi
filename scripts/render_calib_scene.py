#!/usr/bin/env python3
"""渲染标定场景：机械臂(当前姿态) + 桌面 + 底座 + 棋盘(按外参) + 打点球。

自包含版本：不依赖 digital_twin（VM 上是旧版）。把"打点是否落在棋盘
格点上"直接画出来，让用户对照真实世界判断标定是否符合物理。

用法:
  ~/mj/bin/python render_calib_scene.py \
      --marks /tmp/extrinsic_marks_merged.json --out /tmp/calib_scene.png
"""
import argparse
import json
import math
import os

import numpy as np

import mujoco

URDF = os.path.expanduser(
    '~/legacy/arm/arm-final/ros2_ws/install/so101_bringup/share/so101_bringup'
    '/urdf/so101.urdf')
JOINT_ORDER = ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
               'wrist_flex', 'wrist_roll', 'gripper']

# ---- 场景常量（2026-10-06 实测）----
TABLE_Z = -0.06909          # 桌面（夹爪最低点碰桌、6 点拟合）
BASE_BOTTOM = -0.0024       # 机械臂底座最低点（URDF）
CELL_M = 0.033
BOARD_COLS, BOARD_ROWS = 7, 5   # 内角点
BOARD_W = (BOARD_COLS - 1) * CELL_M   # 0.198 m
BOARD_H = (BOARD_ROWS - 1) * CELL_M   # 0.132 m

# ---- 当前标定结果（merged marks 求解, 2026-10-06 16:51）----
CALIB = {
    'grid_origin_x': 0.2246,
    'grid_origin_y': 0.0218,
    'grid_theta_deg': -83.499,
    'cell_cm': 3.3,
}


def build_scene_with_marks(marks, origin, yaw_deg):
    spec = mujoco.MjSpec.from_file(URDF)
    wb = spec.worldbody

    # 桌面
    gt = wb.add_geom()
    gt.name = 'table'
    gt.type = mujoco.mjtGeom.mjGEOM_BOX
    gt.size = [0.4, 0.4, 0.15]
    gt.pos = [0.3, 0.0, TABLE_Z - 0.15]
    gt.rgba = [0.55, 0.55, 0.58, 1]

    # 底座垫台
    gp = wb.add_geom()
    gp.name = 'pedestal'
    gp.type = mujoco.mjtGeom.mjGEOM_BOX
    gp.size = [0.045, 0.05, (BASE_BOTTOM - TABLE_Z) / 2]
    gp.pos = [0.0, 0.0, (TABLE_Z + BASE_BOTTOM) / 2]
    gp.rgba = [0.3, 0.3, 0.32, 1]

    # 棋盘薄板（按外参摆放）
    yaw = math.radians(yaw_deg)
    cy, sy = math.cos(yaw), math.sin(yaw)
    cx = origin[0] + cy * (BOARD_W / 2) + (-sy) * (BOARD_H / 2)
    cyy = origin[1] + sy * (BOARD_W / 2) + cy * (BOARD_H / 2)
    gb = wb.add_geom()
    gb.name = 'board'
    gb.type = mujoco.mjtGeom.mjGEOM_BOX
    gb.size = [BOARD_W / 2, BOARD_H / 2, 0.0005]
    gb.pos = [cx, cyy, TABLE_Z + 0.0005]
    gb.quat = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    gb.rgba = [0.92, 0.92, 0.90, 1]

    # 棋盘内角点网格（黑点）
    for r in range(BOARD_ROWS):
        for c in range(BOARD_COLS):
            gx = c * CELL_M
            gy = r * CELL_M
            wx = origin[0] + cy * gx - sy * gy
            wy = origin[1] + sy * gx + cy * gy
            gk = wb.add_geom()
            gk.name = f'grid_{r}_{c}'
            gk.type = mujoco.mjtGeom.mjGEOM_SPHERE
            gk.size = [0.001, 0, 0]
            gk.pos = [wx, wy, TABLE_Z + 0.001]
            gk.rgba = ([1.0, 0.5, 0.0, 1] if (r == 0 and c == 0)
                       else [0.2, 0.2, 0.2, 0.8])

    # 打点球（用户实际接触位置，红色，大一点）
    for i, m in enumerate(marks):
        c = m['contact_m']
        g = wb.add_geom()
        g.name = f'mark_{i}'
        g.type = mujoco.mjtGeom.mjGEOM_SPHERE
        g.size = [0.005, 0, 0]
        g.pos = [c[0], c[1], TABLE_Z + 0.003]
        g.rgba = [1.0, 0.1, 0.1, 1]

    return spec.compile()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--marks', default='/tmp/extrinsic_marks_merged.json')
    ap.add_argument('--out', default='/tmp/calib_scene.png')
    ap.add_argument('--lookat', nargs=3, type=float, default=[0.26, -0.02, -0.05])
    ap.add_argument('--distance', type=float, default=0.75)
    a = ap.parse_args()

    marks = json.load(open(a.marks))
    print(f'加载 {len(marks)} 个打点')
    for i, m in enumerate(marks):
        c = m['contact_m']
        print(f'  mark{i} grid{m["grid_cm"]}: ({c[0]:.4f}, {c[1]:.4f})')

    origin = (CALIB['grid_origin_x'], CALIB['grid_origin_y'])
    yaw_deg = CALIB['grid_theta_deg']
    print(f'棋盘原点 ({origin[0]:.4f}, {origin[1]:.4f}) '
          f'yaw {yaw_deg:+.1f}°（当前标定结果）')

    model = build_scene_with_marks(marks, origin, yaw_deg)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    # 机械臂摆到最近打点姿态
    last = marks[-1]
    q = [float(last['joints'][k]) for k in JOINT_ORDER]
    for i, v in enumerate(q):
        data.qpos[i] = v
    mujoco.mj_forward(model, data)
    print(f'机械臂姿态 = 最近打点 ({q[0]:.2f}, {q[1]:.2f}, {q[2]:.2f}, '
          f'{q[3]:.2f}, {q[4]:.2f}, {q[5]:.2f})')

    r = mujoco.Renderer(model, 480, 640)
    for tag, cam_kw in [('iso', dict(azimuth=60, elevation=-40)),
                        ('top', dict(azimuth=0, elevation=-88)),
                        ('side', dict(azimuth=0, elevation=-20))]:
        cam = mujoco.MjvCamera()
        cam.lookat[:] = a.lookat
        cam.distance = a.distance
        cam.azimuth = cam_kw['azimuth']
        cam.elevation = cam_kw['elevation']
        r.update_scene(data, cam)
        img = r.render()
        out = a.out.replace('.png', f'_{tag}.png')
        try:
            import cv2
            cv2.imwrite(out, img[:, :, ::-1])
        except ImportError:
            from PIL import Image
            Image.fromarray(img[:, :, ::-1]).save(out)
        print(f'已渲染 {out}')

    # 量化报告：每个打点离其应占格点的水平偏差
    print()
    print('=== 打点 vs 棋盘格点 偏差（水平）===')
    yaw = math.radians(yaw_deg)
    cy, sy = math.cos(yaw), math.sin(yaw)
    for i, m in enumerate(marks):
        c = np.array(m['contact_m'][:2])
        gx, gy = m['grid_cm']
        expect = np.array([
            origin[0] + cy * (gx / 100.0) - sy * (gy / 100.0),
            origin[1] + sy * (gx / 100.0) + cy * (gy / 100.0)])
        d = np.linalg.norm(c - expect) * 1000
        print(f'  grid({gx:.1f},{gy:.1f}): 打点({c[0]:.4f},{c[1]:.4f}) '
              f'应({expect[0]:.4f},{expect[1]:.4f})  偏差 {d:5.1f} mm')


if __name__ == '__main__':
    main()
