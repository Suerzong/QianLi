#!/usr/bin/env python3
"""数字孪生场景：SO-101 + 桌面 + 底座 + 棋盘 + 物块（MuJoCo）

为什么需要底座：
  URDF 里机械臂最低点是 base_link - 0.0024 m；
  实测"机械臂最下端离桌面 5 cm" → 桌面在 base_link - 0.0524 m。
  两者差 5cm，说明机械臂下面有个约 5cm 高的底座/垫块。
  这是之前所有高度矛盾的根源（我把桌面当成了 5cm 更低）。

场景参数（全部来自实测）：
  桌面高度      z = -0.0524 m（base_link 坐标系）
  底座          z 从 -0.0524 到 -0.0024（5cm）
  棋盘中心      base (0.3420, 0.0584) 为棋盘原点(左上内角)
                棋盘 22.8 × 16.2 cm，绕 z 转 -97.75°
  物块          2cm 立方体，立在棋盘上（中心 z = 桌面 + 0.01）

用法：
  ~/mj/bin/python digital_twin.py --check          # 场景自检
  ~/mj/bin/python digital_twin.py --compare        # 与真机关节角对比验证
  ~/mj/bin/python digital_twin.py --render out.png # 渲染一张图
"""

import argparse
import math
import os

import numpy as np

import mujoco

URDF = os.path.expanduser(
    '~/legacy/arm/arm-final/ros2_ws/install/so101_bringup/share/so101_bringup'
    '/urdf/so101.urdf')

# ---- 实测场景参数 ----
TABLE_Z = -0.0524        # 桌面上表面（base_link 坐标系）
BASE_BOTTOM = -0.0024    # 机械臂底座最低点
BOARD_ORIGIN = (0.3420, 0.0584)   # 棋盘原点（左上内角）在 base 平面位置
BOARD_YAW_DEG = -97.75
BOARD_W, BOARD_H = 0.228, 0.162   # 8×6 格 × 3.25cm
OBJECT_SIZE = 0.02
OBJECT_GRID = (0.111, 0.0)        # 物块在棋盘坐标 (11.1cm, 0) → 可达区内


def build_scene(object_grid=OBJECT_GRID):
    """组合场景：机械臂 URDF + 桌面/底座/棋盘（worldbody 静态 geom）+ 物块。

    注意：MjSpec 从 URDF 载入后，用 add_body 加的**无关节静态 body 会被
    compile 丢弃**（实测），所以环境几何体直接挂到 worldbody 上。
    """
    spec = mujoco.MjSpec.from_file(URDF)
    wb = spec.worldbody

    # 桌面（上表面在 TABLE_Z）
    gt = wb.add_geom()
    gt.name = 'table'
    gt.type = mujoco.mjtGeom.mjGEOM_BOX
    gt.size = [0.4, 0.4, 0.15]
    gt.pos = [0.3, 0.0, TABLE_Z - 0.15]
    gt.rgba = [0.55, 0.55, 0.58, 1]

    # 底座（撑起机械臂 5cm）
    gp = wb.add_geom()
    gp.name = 'pedestal'
    gp.type = mujoco.mjtGeom.mjGEOM_BOX
    gp.size = [0.045, 0.05, (BASE_BOTTOM - TABLE_Z) / 2]
    gp.pos = [0.0, 0.0, (TABLE_Z + BASE_BOTTOM) / 2]
    gp.rgba = [0.3, 0.3, 0.32, 1]

    # 棋盘（薄板，按实测位置与朝向）
    yaw = math.radians(BOARD_YAW_DEG)
    cy, sy = math.cos(yaw), math.sin(yaw)
    cx = BOARD_ORIGIN[0] + cy * (BOARD_W / 2) + (-sy) * (BOARD_H / 2)
    cyy = BOARD_ORIGIN[1] + sy * (BOARD_W / 2) + cy * (BOARD_H / 2)
    gb = wb.add_geom()
    gb.name = 'board'
    gb.type = mujoco.mjtGeom.mjGEOM_BOX
    gb.size = [BOARD_W / 2, BOARD_H / 2, 0.0015]
    gb.pos = [cx, cyy, TABLE_Z + 0.0015]
    gb.quat = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    gb.rgba = [0.9, 0.9, 0.9, 1]

    # 物块（动态，可被抓/掉落）
    ox = BOARD_ORIGIN[0] + cy * object_grid[0] - sy * object_grid[1]
    oy = BOARD_ORIGIN[1] + sy * object_grid[0] + cy * object_grid[1]
    ob = wb.add_body(name='object')
    ob.pos = [ox, oy, TABLE_Z + 0.003 + OBJECT_SIZE / 2]
    ob.add_freejoint()
    go = ob.add_geom()
    go.name = 'cube'
    go.type = mujoco.mjtGeom.mjGEOM_BOX
    go.size = [OBJECT_SIZE / 2] * 3
    go.rgba = [0.75, 0.75, 0.78, 1]
    go.mass = 0.008          # 8g（2cm 塑料块）

    return spec.compile(), (ox, oy, TABLE_Z + 0.003 + OBJECT_SIZE / 2)


# gripper_frame_joint 在 gripper_link 下的固定变换（来自 URDF）
#   origin xyz=(-0.0079, -0.000218121, -0.0981274), rpy=(0, π, 0)
FRAME_IN_GRIPPER = np.array([-0.0079, -0.000218121, -0.0981274])


def tcp_pos(model, data):
    """gripper_frame_link 在世界(=base_link)坐标下的位置。

    MuJoCo 会把固定关节合并进父 body，所以 gripper_frame_link 不存在独立
    body —— 用 gripper_link 的位置 + 固定偏移算出来。
    """
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                            'gripper_link')
    if bid < 0:
        return None
    xpos = data.xpos[bid]
    xmat = data.xmat[bid].reshape(3, 3)
    return xpos + xmat @ FRAME_IN_GRIPPER


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true')
    ap.add_argument('--compare', action='store_true')
    ap.add_argument('--render')
    ap.add_argument('--q', nargs=6, type=float,
                    help='设置 6 个关节角（rad），顺序同 URDF')
    a = ap.parse_args()

    model, obj_pos = build_scene()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    if a.check or not any([a.compare, a.render, a.q]):
        print(f'场景: nbody={model.nbody} njnt={model.njnt} '
              f'ngeom={model.ngeom} nq={model.nq}')
        print('\nBodies:')
        for i in range(model.nbody):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
            print(f'  {i}: {name}')
        print('\nGeoms:')
        for i in range(model.ngeom):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
            print(f'  {i}: {name}')
        print(f'\n物块初始位置 = ({obj_pos[0]:.4f}, {obj_pos[1]:.4f}, '
              f'{obj_pos[2]:.4f})')
        print(f'桌面 z = {TABLE_Z:.4f}，底座顶 z = {BASE_BOTTOM:.4f}')

    if a.q:
        for i, v in enumerate(a.q):
            data.qpos[i] = v
        mujoco.mj_forward(model, data)
        p = tcp_pos(model, data)
        print(f'\n关节角 {a.q} → TCP (base_link) = '
              f'({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})')

    if a.compare:
        # 用真机读到的关节角，对比 MuJoCo 与真机 TF 的 TCP 位置
        q_real = [-0.1396, 0.9127, -0.1120, 1.3790, 1.5999, -0.0077]
        for i, v in enumerate(q_real):
            data.qpos[i] = v
        mujoco.mj_forward(model, data)
        p = tcp_pos(model, data)
        print(f'\nMuJoCo 用真机关节角 {q_real}')
        print(f'  → TCP = ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})')
        print('  真机 TF 当时读到 (0.2010, 0.0040, -0.0600)'
              '（下垂后位置，仅供参考）')

    if a.render:
        r = mujoco.Renderer(model, 480, 640)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [0.3, 0.0, -0.03]
        cam.distance = 0.6
        cam.azimuth = 130
        cam.elevation = -35
        r.update_scene(data, cam)
        img = r.render()
        try:
            import cv2
            cv2.imwrite(a.render, img[:, :, ::-1])
        except ImportError:
            from PIL import Image
            Image.fromarray(img).save(a.render)
        print(f'\n已渲染 {a.render}')


if __name__ == '__main__':
    main()
