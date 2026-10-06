#!/usr/bin/env python3
"""数字孪生场景：SO-101 + 桌面 + 底座 + 棋盘 + 物块（MuJoCo）

场景参数（2026-10-06 全部重新实测）
-----------------------------------
  桌面高度    z = -0.06909 m   夹爪最低点碰桌、6 点拟合平面，残差 RMS 0.469mm
  平面倾角      0.202°         实测法向 (0.0030, -0.0019, 0.99999)
  底座        z 从 -0.06909 到 -0.0024  = 66.69 mm
              （base_link 安装面 -2.40mm 与实测桌面之差）
  棋盘        7x5 内角点 ⇒ 图案跨 6x4 格，格边长 33mm
              位姿**从外参文件读**，不再内置
  物块        4cm EVA 泡棉方块，立在棋盘上

三条被推翻的旧结论（留着是为了别再犯）
--------------------------------------
1. **桌面不是 -0.0524**。旧值是从"机械臂最下端离桌面约 5cm"这句估话 +
   URDF 底座 -0.0024 反推的，实测比它低 16.7mm —— 那个垫台实际是 66.7mm
   不是 50mm。
2. **碰到桌面的是"夹爪几何最低点"，不是 TCP**。TCP 相对最低点的高度随
   姿态从 +5.1mm 变到 +100.6mm，所以用 TCP 的 Z 当桌面高度会随姿态漂移。
3. **棋盘位姿不能用那轮两点法标定的值**（0.3420, 0.0584 / -97.75°）：
   两个内角点 Z 差 9.5mm、反推格宽 34.6mm，它连自己都不自洽
   （见 docs/GRASP_REAL_AUDIT.md）。现在必须从带 quality_ok 的外参文件读。

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
# TABLE_Z：2026-10-06 实测（夹爪最低点碰桌、6 点拟合平面，残差 RMS 0.469mm）。
# 旧值 -0.0524 是**推算**的（"最下端离桌约 5cm" + URDF 底座 -0.0024），
# 实测比它低 16.7mm —— 那个垫台实际不是 5cm。
# 注意：碰到桌面的是"夹爪几何最低点"，不是 TCP；TCP 相对最低点的高度
# 随姿态从 +5.1mm 变到 +100.6mm，所以不能用 TCP 的 Z 当桌面高度。
TABLE_Z = -0.06909
TABLE_TILT_DEG = 0.202   # 实测平面倾角，建模按水平处理
BASE_BOTTOM = -0.0024    # 机械臂底座最低点（URDF）

# 棋盘：7x5 个**内角点** ⇒ 图案跨 6x4 **格**；格边长 33mm（用户确认）。
# 旧值 (0.228, 0.162) 配注释"8x6 格 x 3.25cm"是自相矛盾的：
# 22.8cm 其实是 7x3.257，16.2cm 是 5x3.24 —— 把"角点数"当成了"格数"，
# 而且格宽用了 32.5mm。两处都错。
CELL_M = 0.033
BOARD_COLS, BOARD_ROWS = 7, 5          # 内角点
BOARD_W = (BOARD_COLS - 1) * CELL_M    # 0.198 m
BOARD_H = (BOARD_ROWS - 1) * CELL_M    # 0.132 m

# 棋盘在 base_link 下的位姿 **必须来自外参标定**。
# 旧的 (0.3420, 0.0584) / -97.75° 出自那轮已被判定不可信的两点法标定
# （见 docs/GRASP_REAL_AUDIT.md：两个内角点 Z 差 9.5mm、反推格宽 34.6mm），
# 所以这里**不再内置旧值**，改成读外参文件；读不到就明确报警并跳过棋盘。
EXTRINSIC_PATH = '/tmp/extrinsic.txt'
BOARD_ORIGIN = None      # 由外参填充；None = 未知，不画棋盘
BOARD_YAW_DEG = None

# 物块：现在是 **4cm EVA 泡棉方块**（旧值 2cm 是上一轮的塑料小方块）
OBJECT_SIZE = 0.04
OBJECT_GRID = (0.111, 0.0)        # 物块在棋盘坐标 (11.1cm, 0) → 可达区内
# 5 种颜色（2026-10-06 现场实测色相，OpenCV H 0~179）
OBJECT_COLORS = {
    'red':    (0.85, 0.15, 0.15),
    'yellow': (0.95, 0.85, 0.20),
    'green':  (0.25, 0.75, 0.30),
    'blue':   (0.20, 0.35, 0.85),
    'purple': (0.55, 0.30, 0.80),
}


def load_extrinsics(path=EXTRINSIC_PATH):
    """从外参文件读棋盘位姿，带质量标记校验。

    为什么要校验 quality_ok：外参文件被多个脚本消费，而旧的写入方
    （extrinsic_calib.py 两点法）会往同一路径写。手滑跑一次旧的就会把
    标好的值覆盖掉，下游全静默用错值 —— 所以没有质量标记的一律拒绝。
    """
    if not os.path.exists(path):
        return None, f'外参文件不存在（{path}）—— 棋盘位姿未知'
    kv = {}
    for line in open(path, encoding='utf-8'):
        s = line.strip()
        if not s or s.startswith('#') or '=' not in s:
            continue
        k, v = s.split('=', 1)
        kv[k.strip()] = v.strip()
    if 'quality_ok' not in kv or float(kv.get('quality_ok', 0)) < 0.5:
        return None, (f'外参未通过质量裁决（quality_ok='
                      f'{kv.get("quality_ok", "缺失")}）—— 拒绝使用')
    try:
        return (float(kv['grid_origin_x']), float(kv['grid_origin_y']),
                float(kv['grid_theta_deg'])), None
    except (KeyError, ValueError) as exc:
        return None, f'外参字段不完整: {exc}'


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

    # 底座（垫台）：高度由 实测桌面 与 base_link 安装面 之差反推
    #   base_link 最低点（base_so101_v2.stl）= -2.40 mm
    #   实测桌面 = -69.09 mm  →  垫台 = 66.69 mm
    # （旧注释写"5cm"是估算，实测差 16.7 mm）
    gp = wb.add_geom()
    gp.name = 'pedestal'
    gp.type = mujoco.mjtGeom.mjGEOM_BOX
    gp.size = [0.045, 0.05, (BASE_BOTTOM - TABLE_Z) / 2]
    gp.pos = [0.0, 0.0, (TABLE_Z + BASE_BOTTOM) / 2]
    gp.rgba = [0.3, 0.3, 0.32, 1]

    # 棋盘（薄板）。位姿**必须来自外参**；外参不可用时明确报警并跳过，
    # 而不是拿旧值糊上去 —— 旧值来自已废弃的两点法，画出来只会误导。
    board_xy = None
    if BOARD_ORIGIN is not None and BOARD_YAW_DEG is not None:
        yaw = math.radians(BOARD_YAW_DEG)
        cy, sy = math.cos(yaw), math.sin(yaw)
        cx = BOARD_ORIGIN[0] + cy * (BOARD_W / 2) + (-sy) * (BOARD_H / 2)
        cyy = BOARD_ORIGIN[1] + sy * (BOARD_W / 2) + cy * (BOARD_H / 2)
        gb = wb.add_geom()
        gb.name = 'board'
        gb.type = mujoco.mjtGeom.mjGEOM_BOX
        gb.size = [BOARD_W / 2, BOARD_H / 2, 0.0005]
        gb.pos = [cx, cyy, TABLE_Z + 0.0005]
        gb.quat = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        gb.rgba = [0.92, 0.92, 0.90, 1]
        board_xy = (cy, sy)
    else:
        print('⚠️ 棋盘位姿未知（外参不可用），场景里不画棋盘。')

    # 物块（动态，可被抓/掉落）。4cm EVA 泡棉方块，密度按 EVA 约 60kg/m^3。
    if board_xy is not None:
        cy, sy = board_xy
        ox = BOARD_ORIGIN[0] + cy * object_grid[0] - sy * object_grid[1]
        oy = BOARD_ORIGIN[1] + sy * object_grid[0] + cy * object_grid[1]
    else:
        ox, oy = 0.26, 0.0            # 棋盘未知时放在臂前的默认可达位
    ob = wb.add_body(name='object')
    ob.pos = [ox, oy, TABLE_Z + 0.003 + OBJECT_SIZE / 2]
    ob.add_freejoint()
    go = ob.add_geom()
    go.name = 'cube'
    go.type = mujoco.mjtGeom.mjGEOM_BOX
    go.size = [OBJECT_SIZE / 2] * 3
    # EVA 密度 ~60 kg/m^3 → 4cm 立方体约 3.8g（旧值 8g 是按 2cm 塑料块给的）
    go.mass = 60.0 * OBJECT_SIZE ** 3
    go.rgba = list(OBJECT_COLORS.get('yellow', (0.9, 0.8, 0.2))) + [1.0]

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
    global BOARD_ORIGIN, BOARD_YAW_DEG
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true')
    ap.add_argument('--compare', action='store_true')
    ap.add_argument('--render')
    ap.add_argument('--q', nargs=6, type=float,
                    help='设置 6 个关节角（rad），顺序同 URDF')
    ap.add_argument('--lookat', nargs=3, type=float,
                    default=[0.3, 0.0, -0.03])
    ap.add_argument('--distance', type=float, default=0.6)
    ap.add_argument('--azimuth', type=float, default=130.0)
    ap.add_argument('--elevation', type=float, default=-35.0)
    a = ap.parse_args()

    # 棋盘位姿只能来自外参（内置旧值是废弃的两点法结果，不能用）
    ext, why = load_extrinsics()
    if ext is None:
        print(f'⚠️ 外参不可用：{why}')
        print('   → 场景里不画棋盘，物块放在臂前默认位。')
        print('     标完外参后重跑即可自动对齐。')
    else:
        BOARD_ORIGIN = (ext[0], ext[1])
        BOARD_YAW_DEG = ext[2]
        print(f'✅ 外参已加载：棋盘原点 ({ext[0]:.4f}, {ext[1]:.4f}) m  '
              f'yaw {ext[2]:+.3f}°')

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
        cam.lookat[:] = a.lookat
        cam.distance = a.distance
        cam.azimuth = a.azimuth
        cam.elevation = a.elevation
        r.update_scene(data, cam)
        img = r.render()
        try:
            import cv2
            cv2.imwrite(a.render, img[:, :, ::-1])
        except ImportError:
            from PIL import Image
            Image.fromarray(img).save(a.render)
        print(f'\n已渲染 {a.render}  '
              f'(azimuth={a.azimuth} elevation={a.elevation} '
              f'distance={a.distance})')
        # 顺便报告夹爪最低点与桌面的关系
        gjid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                 'moving_jaw_so101_v1_link')
        glid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                 'gripper_link')
        lows = []
        for bid in (gjid, glid):
            if bid < 0:
                continue
            for g in range(model.body_geomadr[bid],
                           model.body_geomadr[bid] + model.body_geomnum[bid]):
                lows.append(data.geom_xpos[g][2] - model.geom_size[g][2])
        if lows:
            print(f'夹爪最低点 z ≈ {min(lows)*1000:+.1f} mm，'
                  f'桌面 z = {TABLE_Z*1000:+.1f} mm，'
                  f'间隙 {(min(lows)-TABLE_Z)*1000:+.1f} mm')


if __name__ == '__main__':
    main()
