#!/usr/bin/env python3
"""合成外参：T_base_cam = T_base_board · T_board_cam

两半是**互相独立**测出来的，这是本方法最大的优点：

    相机侧  board_pose_cam.py        solvePnP（35 个角点、亚像素）
            → T_board_cam            只用到相机和内参，**不知道机械臂存在**
    机械臂侧 extrinsic_calib_multi.py 触点 + Umeyama
            → T_base_board           只用到机械臂和棋盘几何，**不知道相机存在**

所以任何一半出错，都不会被另一半"吸收"掉 —— 这点和手眼标定（AX=XB）不同，
后者两侧耦合，一侧错会污染整个解。

验证手段（都不循环论证）
------------------------
1. **两侧各自的残差**：solvePnP 重投影误差（相机侧）、触点拟合残差（机械臂侧）
2. **留一灵敏度**：逐个去掉一个触点重解，看 T_base_cam 平移/旋转抖动多少。
   这直接回答"外参有多稳"，比任何单一 RMS 都实在。
3. **桌面交叉验证**：触点解出的板面 z 必须等于独立实测的桌面高度 + 纸厚。
   这条**完全独立于相机**（桌面是另一套 6 点拟合测出来的）。
4. **反投影一致性**：把棋盘的角点经 T_base_board 抬到 base_link、再经
   T_base_cam⁻¹ 拉回相机、投影回图像 —— 应当与 solvePnP 的重投影一致。
   ⚠️ 注意这条是**恒等式**（合成本身就保证它成立），所以它只能用来查
   "实现有没有写错"，**不能**当作精度证据。这里写出来是为了自检实现。

用法::

    ~/mj/bin/python compose_extrinsic.py \
        --board-cam ~/QianLi/calib/board_cam.npz \
        --marks /tmp/extrinsic_marks.json \
        --out ~/QianLi/calib/camera_extrinsic.yaml
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

TABLE_Z_MEASURED = -0.06909      # 独立实测（夹爪碰桌、6 点拟合，RMS 0.469mm）
TABLE_FIT_RMS_MM = 0.469


# ---------------------------------------------------------------- 基础工具
def rvec_tvec_to_T(rvec, tvec):
    import cv2
    R, _ = cv2.Rodrigues(np.asarray(rvec, float).reshape(3, 1))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(tvec, float).ravel()
    return T


def umeyama(src, dst, with_scale=False):
    """标准 Umeyama 相似/刚体变换：求 T 使 T·src ≈ dst。

    这里用**刚体**（with_scale=False）—— 棋盘格边长 33mm 是印出来的已知量，
    没有理由让拟合去缩放它。允许缩放反而会把打点噪声吸收成假的比例误差
    （之前那轮就是这样得出 18% 假各向异性的）。
    """
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)
    mu_s, mu_d = src.mean(0), dst.mean(0)
    Sc, Dc = src - mu_s, dst - mu_d
    H = Sc.T @ Dc
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    if with_scale:
        s = S.sum() / (Sc ** 2).sum()
    else:
        s = 1.0
    t = mu_d - s * (R @ mu_s)
    T = np.eye(4)
    T[:3, :3] = s * R
    T[:3, 3] = t
    return T, R, t


def load_marks(path):
    """读触点。支持新格式（contact_m = 爪子最低点）和旧格式（base_m = TCP）。"""
    marks = json.load(open(path))
    if isinstance(marks, dict):
        marks = marks.get('marks', marks)
    pts, grids, using_contact = [], [], True
    for m in marks:
        g = np.array(m['grid_cm'], float) / 100.0
        if 'contact_m' in m:
            p = np.array(m['contact_m'], float)
        else:
            # 旧格式：base_m 是 TCP，z 要扣掉"TCP 高于接触点"才是板面
            using_contact = False
            p = np.array(m['base_m'], float).copy()
            p[2] -= float(m.get('tcp_above_lowest_m', 0.0))
        grids.append(g)
        pts.append(p)
    return np.array(grids), np.array(pts), using_contact


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--board-cam', default=os.path.expanduser(
        '~/QianLi/calib/board_cam.npz'))
    ap.add_argument('--marks', default='/tmp/extrinsic_marks.json')
    ap.add_argument('--out', default=os.path.expanduser(
        '~/QianLi/calib/camera_extrinsic.yaml'))
    ap.add_argument('--cell-cm', type=float, default=3.3)
    ap.add_argument('--board-mm', type=float, default=0.5)
    args = ap.parse_args()

    # ---------------------------------------------------- 相机侧
    z = np.load(args.board_cam)
    T_board_cam = rvec_tvec_to_T(z['rvec'], z['tvec'])
    T_cam_board = np.linalg.inv(T_board_cam)
    print('=' * 74)
    print('  输入')
    print('=' * 74)
    print(f'  相机侧  {args.board_cam}')
    print(f'          {int(z["n_frames"])} 帧中值，棋盘原点在相机系 '
          f'({z["tvec"][0]:+.4f}, {z["tvec"][1]:+.4f}, {z["tvec"][2]:+.4f}) m')
    print(f'          帧间离散 (x,y,z) = ({z["tvec_std"][0]*1000:.3f}, '
          f'{z["tvec_std"][1]*1000:.3f}, {z["tvec_std"][2]*1000:.3f}) mm')

    grids, pts, using_contact = load_marks(args.marks)
    print(f'  机械臂侧 {args.marks}  {len(pts)} 点'
          f'{"（contact_m 接触点）" if using_contact else "（旧格式 base_m，已扣 TCP 偏移）"}')

    # ---------------------------------------------------- 机械臂侧
    # 棋盘平面上的格点 (x, y, 0) -> base_link
    src = np.hstack([grids, np.zeros((len(grids), 1))])
    T_base_board, R, t = umeyama(src, pts, with_scale=False)
    fit = (T_base_board[:3, :3] @ src.T).T + T_base_board[:3, 3]
    res = np.linalg.norm(fit - pts, axis=1)
    rms = float(np.sqrt((res ** 2).mean()))

    # 板面高度：拟合出来的棋盘原点 z（取全部触点板面 z 的均值更稳）
    plane_z = float(pts[:, 2].mean())
    print()
    print('=' * 74)
    print('  合成结果  T_base_cam')
    print('=' * 74)
    print(f'  触点拟合残差 RMS  {rms*1000:.2f} mm   最大 {res.max()*1000:.2f} mm')
    print(f'  单点残差(mm)      {np.round(res*1000, 2).tolist()}')

    T_base_cam = T_base_board @ T_cam_board
    Rb = T_base_cam[:3, :3]
    tb = T_base_cam[:3, 3]
    # 相机在 base_link 下的朝向：光轴 = R 的第三列
    axis = Rb[:, 2]
    print(f'  相机位置          ({tb[0]:+.4f}, {tb[1]:+.4f}, {tb[2]:+.4f}) m')
    print(f'  相机光轴(朝前)    ({axis[0]:+.4f}, {axis[1]:+.4f}, {axis[2]:+.4f})')
    print(f'    俯角            {math.degrees(math.asin(max(-1,min(1,-axis[2])))):.2f}° '
          f'(向下看为正)')
    print(f'  相机到棋盘原点距离 {np.linalg.norm(tb - T_base_board[:3,3]):.4f} m')

    # ---------------------------------------------------- 验证 1：桌面交叉验证
    print()
    print('  ── 交叉验证（独立于相机，桌面是另一套 6 点拟合测的）──')
    expect = TABLE_Z_MEASURED + args.board_mm / 1000.0
    print(f'  触点解出的板面 z   {plane_z:+.5f} m')
    print(f'  期望（桌面+纸厚）  {expect:+.5f} m  '
          f'(桌面 {TABLE_Z_MEASURED:+.5f} + {args.board_mm}mm)')
    dz = (plane_z - expect) * 1000
    print(f'  偏差              {dz:+.2f} mm   '
          f'{"✅ 通过" if abs(dz) < 2.0 else "❌ 超出 ±2mm"}')

    # ---------------------------------------------------- 验证 2：留一灵敏度
    print()
    print('  ── 留一灵敏度（逐个去掉一个触点重解，看结果抖多少）──')
    dt, dr = [], []
    for k in range(len(pts)):
        m = [i for i in range(len(pts)) if i != k]
        Tk, _, _ = umeyama(src[m], pts[m], with_scale=False)
        Tk = Tk @ T_cam_board
        dt.append((Tk[:3, 3] - tb) * 1000)
        dR = Tk[:3, :3] @ Rb.T
        ang = math.degrees(math.acos(max(-1, min(1, (np.trace(dR) - 1) / 2))))
        dr.append(ang)
    dt = np.array(dt)
    print(f'  平移抖动          {np.abs(dt).max():.2f} mm (各分量最大)  '
          f'模长最大 {np.linalg.norm(dt, axis=1).max():.2f} mm')
    print(f'  旋转抖动          最大 {max(dr):.3f}°')
    loo_ok = np.linalg.norm(dt, axis=1).max() < 3.0 and max(dr) < 0.5

    # ---------------------------------------------------- 验证 3：反投影自检
    # 这一步是**恒等式检查**：如果实现没写错，误差应等于 solvePnP 的重投影误差。
    import cv2
    K = z['K']
    D = z['D']
    # 棋盘角点 -> base -> 相机
    obj = np.zeros((int(z['rows']) * int(z['cols']), 3), np.float32)
    obj[:, :2] = np.array([[c, r] for r in range(int(z['rows']))
                           for c in range(int(z['cols']))]) * float(z['cell'])
    obj_base = (T_base_board[:3, :3] @ obj.T).T + T_base_board[:3, 3]
    obj_cam = (T_base_cam[:3, :3].T @ (obj_base - tb).T).T
    proj, _ = cv2.projectPoints(obj_cam.astype(np.float64), np.zeros(3),
                                np.zeros(3), K, D)
    # 与相机侧的原始 tvec/rvec 反投影比较
    proj0, _ = cv2.projectPoints(obj.astype(np.float64), z['rvec'], z['tvec'], K, D)
    id_err = np.linalg.norm(proj.reshape(-1, 2) - proj0.reshape(-1, 2), axis=1)
    print()
    print('  ── 反投影自检（恒等式，只查实现有没有写错）──')
    print(f'  合成链路 vs 直接投影 最大差 {id_err.max():.4f} px  '
          f'{"✅ 实现一致" if id_err.max() < 0.01 else "❌ 实现有问题"}')

    # ---------------------------------------------------- 汇总
    ok = (rms * 1000 < 3.0) and abs(dz) < 2.0 and loo_ok and id_err.max() < 0.01
    print()
    print('=' * 74)
    print(f'  质量裁决  {"✅ 通过" if ok else "⚠️ 不完全通过"}')
    print(f'    触点拟合 RMS < 3mm      : {rms*1000:.2f} mm   '
          f'{"✅" if rms*1000 < 3 else "❌"}')
    print(f'    桌面交叉验证 < 2mm      : {abs(dz):.2f} mm   '
          f'{"✅" if abs(dz) < 2 else "❌"}')
    print(f'    留一抖动 < 3mm / 0.5°   : '
          f'{np.linalg.norm(dt, axis=1).max():.2f} mm / {max(dr):.3f}°   '
          f'{"✅" if loo_ok else "❌"}')
    print(f'    反投影实现自检          : {id_err.max():.4f} px   '
          f'{"✅" if id_err.max() < 0.01 else "❌"}')
    print(f'    ⚠️ 相机侧 solvePnP 重投影 ~1.05px ≈ {0.4836/485.46*1.05*1000:.2f} mm '
          f'（这是本方法的主要误差来源，不在上面四项里）')
    print('=' * 74)

    # ---------------------------------------------------- 写出
    with open(args.out, 'w') as f:
        f.write('# 相机外参：base_link <- camera\n')
        f.write('# 由 compose_extrinsic.py 生成（纯棋盘路线：solvePnP + 触点）\n')
        f.write(f'# 触点拟合 RMS {rms*1000:.2f} mm，桌面交叉验证 {dz:+.2f} mm，'
                f'留一平移抖动 {np.linalg.norm(dt, axis=1).max():.2f} mm\n')
        f.write(f'quality_ok={1 if ok else 0}\n')
        f.write('parent_frame=base_link\n')
        f.write('child_frame=camera_optical_frame\n')
        f.write(f'x={tb[0]:.10f}\ny={tb[1]:.10f}\nz={tb[2]:.10f}\n')
        q = _rot_to_quat(Rb)
        f.write(f'qx={q[0]:.10f}\nqy={q[1]:.10f}\nqz={q[2]:.10f}\nqw={q[3]:.10f}\n')
        f.write('# 4x4 行主序\n')
        for row in T_base_cam:
            f.write('T_row=' + ','.join(f'{v:.10f}' for v in row) + '\n')
        f.write('# legacy grid->base（给老消费者用）\n')
        th = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        f.write(f'grid_theta_deg={th:.6f}\n')
        f.write(f'grid_origin_x={t[0]:.10f}\ngrid_origin_y={t[1]:.10f}\n')
    print(f'\n📄 已写 {args.out}')
    with open('/tmp/extrinsic.txt', 'w') as f:
        f.write('# 由 compose_extrinsic.py 生成（含 quality_ok）\n')
        f.write(f'quality_ok={1 if ok else 0}\n')
        f.write(f'grid_theta_deg={th:.6f}\n')
        f.write(f'grid_origin_x={t[0]:.10f}\ngrid_origin_y={t[1]:.10f}\n')
    print('📄 同时写了 /tmp/extrinsic.txt（向后兼容老消费者）')
    return 0 if ok else 2


def _rot_to_quat(R):
    tr = np.trace(R)
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / S, (R[0, 2] - R[2, 0]) / S,
                (R[1, 0] - R[0, 1]) / S, 0.25 * S)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    if i == 0:
        S = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * S, (R[0, 1] + R[1, 0]) / S,
                (R[0, 2] + R[2, 0]) / S, (R[2, 1] - R[1, 2]) / S)
    if i == 1:
        S = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / S, 0.25 * S,
                (R[1, 2] + R[2, 1]) / S, (R[0, 2] - R[2, 0]) / S)
    S = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / S, (R[1, 2] + R[2, 1]) / S,
            0.25 * S, (R[1, 0] - R[0, 1]) / S)


if __name__ == '__main__':
    sys.exit(main())
