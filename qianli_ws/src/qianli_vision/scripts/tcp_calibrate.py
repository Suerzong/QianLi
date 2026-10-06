#!/usr/bin/env python3
"""TCP 标定：用"爪口触碰同一平面"求真实抓取点相对 gripper_link 的偏移

为什么这样做
------------
`gripper_frame_link` 只是 URDF 里的虚拟工具坐标系。真正决定"能不能夹到物块"
的是**两爪内侧面形成的抓取点**，它相对 gripper_link 的实际位置受打印件、
装配和舵机零位影响，只能实测。

方法（工业上常用的"平面约束 TCP 标定"）
--------------------------------------
把夹爪以**若干不同倾角**去碰同一张**平坦桌面**。设 TCP 在 gripper_link 下的
偏移为 t（未知），第 i 次触碰时 gripper_link 在 base_link 下为 (R_i, p_i)，
则接触点 c_i = p_i + R_i·t 必须落在同一平面上：

    n·c_i = d          （n, d 为该平面，未知）

* 固定平面、解 t：线性最小二乘；
* 平面也未知：交替迭代（先假设水平 → 解 t → 用 c_i 拟合平面 → 再解 t），
  SVD 拟合平面，通常 3~5 轮收敛。

**不需要使能扭矩**：松开扭矩后用手把爪子放到桌面上即可，零运动风险。
这也正是 `lerobot-calibrate` 的手动风格。

用法::

    # 终端 A（读取 TF 并等待打点）
    ros2 launch so101_bringup ik_demo.launch.py driver_mode:=direct use_rviz:=false
    ~/mj/bin/python tcp_calibrate.py --marks 6
    # 每次摆好一个倾角、爪口轻触桌面后：
    touch /tmp/tcp_mark

    # 只差最后一步时也可以手工喂数据
    ~/mj/bin/python tcp_calibrate.py --solve-only /tmp/tcp_marks.json
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np

TRIGGER = '/tmp/tcp_mark'
OUT_JSON = '/tmp/tcp_marks.json'
OUT_TXT = '/tmp/tcp_calib.txt'
FLANGE = 'gripper_link'
BASE = 'base_link'


# ------------------------- 纯数学部分（可离线测试） -------------------------
def quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        raise ValueError('zero quaternion')
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def fit_plane(points):
    """SVD 拟合平面，返回 (单位法向 n, 距离 d)，使 n·p = d。"""
    centroid = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - centroid)
    n = vt[2]
    if n[2] < 0:
        n = -n                      # 让法向朝上，便于读表
    return n, float(n @ centroid)


def solve_tcp_point(poses):
    """经典"定点法"：TCP 始终触碰同一个固定点 P。

    p_i + R_i·t = P   →   [R_i | −I]·[t; P] = −p_i
    6 个未知数，3N 个方程；姿态一变就有很好的杠杆臂，
    比"平面法"条件数好得多（实测同样噪声下误差小一个量级）。

    返回 (t, P, residuals_mm)
    """
    rows, rhs = [], []
    for R, p in poses:
        rows.append(np.hstack([R, -np.eye(3)]))
        rhs.append(-p)
    A = np.vstack(rows)
    b = np.concatenate(rhs)
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    t, P = x[:3], x[3:]
    residuals = np.array([np.linalg.norm(p + R @ t - P) for R, p in poses]) * 1000.0
    return t, P, residuals


def solve_tcp_horizontal(poses):
    """假设桌面水平 (n=z)，线性最小二乘解 t 与桌面高度 d。

    (p_i + R_i t)·ẑ = d  →  R_i[2,:]·t + (−1)·d = −p_i[2]
    未知数 4 个，姿态足够分散时可解。这个解是下面通用解的初值。
    """
    R_list = [R for R, _ in poses]
    p_list = [p for _, p in poses]
    A = np.array([np.append(R[2, :], -1.0) for R in R_list])
    b = np.array([-p[2] for p in p_list])
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    t = x[:3]
    d = float(x[3])
    residual = np.array([(p + R @ t)[2] - d for R, p in poses])
    return t, np.array([0.0, 0.0, 1.0]), d, residual * 1000.0


def _coplanarity_sigma(t, poses):
    """给定 t，返回"接触点共面程度" = 居中后点云的最小奇异值。

    接触点 c_i = p_i + R_i t。它们共面 ⇔ 居中后秩 ≤ 2 ⇔ 最小奇异值 = 0。
    """
    R_list = [R for R, _ in poses]
    p_list = [p for _, p in poses]
    C = np.array([p + R @ t for R, p in poses])
    C = C - C.mean(axis=0)
    return float(np.linalg.svd(C, compute_uv=False)[-1])


def _nelder_mead(func, x0, step=0.004, tol=1e-12, max_iter=4000):
    """极简 Nelder-Mead（3 维），避免依赖 scipy。"""
    x0 = np.asarray(x0, dtype=float)
    n = len(x0)
    simplex = [x0.copy()]
    for i in range(n):
        point = x0.copy()
        point[i] += step
        simplex.append(point)
    simplex = np.array(simplex)
    values = np.array([func(p) for p in simplex])

    for _ in range(max_iter):
        order = np.argsort(values)
        simplex, values = simplex[order], values[order]
        if np.max(np.abs(simplex[1:] - simplex[0])) < tol:
            break
        centroid = simplex[:-1].mean(axis=0)
        worst = simplex[-1]
        reflected = centroid + (centroid - worst)
        f_reflected = func(reflected)
        if f_reflected < values[0]:
            expanded = centroid + 2.0 * (centroid - worst)
            f_expanded = func(expanded)
            simplex[-1], values[-1] = ((expanded, f_expanded)
                                       if f_expanded < f_reflected
                                       else (reflected, f_reflected))
        elif f_reflected < values[-2]:
            simplex[-1], values[-1] = reflected, f_reflected
        else:
            contracted = centroid + 0.5 * (worst - centroid)
            f_contracted = func(contracted)
            if f_contracted < values[-1]:
                simplex[-1], values[-1] = contracted, f_contracted
            else:
                simplex[1:] = simplex[0] + 0.5 * (simplex[1:] - simplex[0])
                values[1:] = np.array([func(p) for p in simplex[1:]])
    best = int(np.argmin(values))
    return simplex[best], float(values[best])


def solve_tcp(poses, iterations=None, plane=None):
    """两段式求解，返回 (t, (n, d), residuals_mm, info)。

    1. 先用"桌面水平"假设做线性最小二乘 —— 有闭式解，稳定；
    2. 再放开水平假设，最小化接触点的共面残差（最小奇异值），
       以第 1 步结果为初值做 Nelder-Mead 精修。

    只用第 1 步会在桌面倾斜时产生偏差；只用交替迭代会掉进局部极小
    （实测会给出 25mm 的错误解，残差却看起来很小）。
    """
    t_h, n_h, d_h, res_h = solve_tcp_horizontal(poses)

    t_refined, sigma = _nelder_mead(
        lambda t: _coplanarity_sigma(t, poses), t_h)

    # 用精修后的 t 拟合平面，得到最终法向与残差
    contacts = np.array([p + R @ t_refined for R, p in poses])
    n, d = fit_plane(contacts)
    residuals = contacts @ n - d

    info = {
        't_horizontal_assumption_mm': (t_h * 1000.0).round(3).tolist(),
        'sigma_after_refine': sigma,
        'horizontal_vs_general_mm': float(np.linalg.norm(t_refined - t_h) * 1000),
        'plane_normal_deg_from_vertical': math.degrees(
            math.acos(min(1.0, abs(n[2])))),
    }
    return t_refined, (n, d), residuals * 1000.0, info


# ------------------------------- ROS 采集端 -------------------------------
def collect(args):
    import rclpy
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener

    class Collector(Node):
        def __init__(self):
            super().__init__('tcp_calibrate')
            self.buf = Buffer()
            self.listener = TransformListener(self.buf, self)
            self.marks = []

        def current(self):
            try:
                tr = self.buf.lookup_transform(BASE, FLANGE, rclpy.time.Time())
            except Exception:
                return None
            t = tr.transform.translation
            q = tr.transform.rotation
            return quat_to_R(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])

        def tick(self):
            if os.path.exists(TRIGGER):
                try:
                    os.remove(TRIGGER)
                except OSError:
                    pass
                pose = self.current()
                if pose is None:
                    self.get_logger().warning('读不到 TF，本次打点忽略')
                    return
                R, p = pose
                self.marks.append({'R': R.tolist(), 'p': p.tolist(),
                                   'at': time.strftime('%H:%M:%S')})
                self.get_logger().info(
                    f'📍 打点 {len(self.marks)}/{args.marks}  '
                    f'{FLANGE} @ ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})  '
                    f'工具轴 = {np.round(R @ [0,0,1], 2)}')

    rclpy.init()
    node = Collector()
    print(f'采集 TCP 标定点：需要 {args.marks} 个不同的工具姿态（{args.mode} 法）')
    if getattr(args, 'mode', 'point') == 'point':
        print('  1) 桌上固定一个**尖点**（针尖 / 立起来的螺丝 / 笔尖）')
        print('  2) 松开扭矩，用手把**爪口抓取点**（两爪之间、想夹物块的位置）')
        print('     对准这个尖点')
        print('  3) 每次换一个明显不同的姿态（转 wrist_flex ±40°、再转 wrist_roll）')
    else:
        print('  1) 松开扭矩，用手把爪口两片爪子**同时轻触桌面**')
        print('  2) 每次换一个明显不同的倾角（±40° 以上）')
    print(f'  4) 摆好后执行： touch {TRIGGER}')
    print('  做完 Ctrl+C 结束\n')
    try:
        while rclpy.ok() and len(node.marks) < args.marks:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.tick()
    except KeyboardInterrupt:
        pass
    marks = node.marks
    node.destroy_node()
    if rclpy.ok():
        rclpy.try_shutdown()

    with open(args.json, 'w') as fh:
        json.dump(marks, fh, indent=2)
    print(f'\n📄 已保存 {len(marks)} 个打点到 {args.json}')
    return marks


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--marks', type=int, default=8)
    ap.add_argument('--mode', default='point', choices=['point', 'plane'],
                    help='point=定点法(准, 需一个固定尖点); '
                         'plane=平面法(易, 只需桌面, 精度约 2~4mm)')
    ap.add_argument('--json', default=OUT_JSON)
    ap.add_argument('--out', default=OUT_TXT)
    ap.add_argument('--solve-only', help='只用已有打点文件求解，不连 ROS')
    args = ap.parse_args()

    if args.solve_only:
        with open(args.solve_only) as fh:
            raw = json.load(fh)
    else:
        raw = collect(args)

    poses = [(np.array(m['R'], dtype=float), np.array(m['p'], dtype=float))
             for m in raw]

    # 姿态多样性检查
    axes = np.array([R @ np.array([0.0, 0.0, 1.0]) for R, _ in poses])
    spread = float(np.degrees(np.arccos(np.clip(
        axes @ axes.T, -1, 1))).max())

    print()
    print('=' * 74)
    print(f'  TCP 标定结果（{args.mode} 法）')
    print('=' * 74)
    print(f'  打点数            {len(poses)}')
    print(f'  工具轴最大夹角    {spread:.1f}°'
          f'   {"✅ 姿态足够分散" if spread > 25 else "⚠️ 姿态太接近，建议加大倾角差异"}')
    print()

    if args.mode == 'point':
        if len(poses) < 4:
            print('❌ 定点法至少需要 4 个不同姿态')
            return 1
        t, P, residuals = solve_tcp_point(poses)
        method = 'point'
        plane_note = []
        extra = {'fixed_point': np.round(P, 5).tolist()}
    else:
        if len(poses) < 8:
            print('⚠️ 平面法在 8 个点以下会过拟合噪声（自检实测 6 点时误差可达 10mm）')
        t, (n, d), residuals, info = solve_tcp(poses)
        method = 'plane'
        plane_note = [
            f'  拟合桌面法向      {np.round(n, 4)}  '
            f'(偏离竖直 {info["plane_normal_deg_from_vertical"]:.2f}°)',
            f'  拟合桌面高度      {d:+.4f} m (base_link)',
            f'  水平假设解        '
            f'{np.round(info["t_horizontal_assumption_mm"], 2).tolist()} mm  '
            f'(与通用解差 {info["horizontal_vs_general_mm"]:.2f} mm)',
        ]
        extra = info

    for line in plane_note:
        print(line)
    print(f'  TCP 偏移（{FLANGE} 坐标系）= '
          f'({t[0]*1000:+.2f}, {t[1]*1000:+.2f}, {t[2]*1000:+.2f}) mm')
    print(f'    沿工具轴的偏移  {t[2]*1000:+.2f} mm')
    print(f'    横向偏移        ({t[0]*1000:+.2f}, {t[1]*1000:+.2f}) mm')
    print(f'    |t|             {np.linalg.norm(t)*1000:.2f} mm')
    rms = float(np.sqrt(np.mean(residuals ** 2)))
    print()
    print(f'  残差             RMS {rms:.2f} mm   最大 {float(np.max(np.abs(residuals))):.2f} mm')
    print(f'  单点残差(mm)     {np.round(residuals, 2).tolist()}')
    threshold = 1.0 if method == 'point' else 2.5
    ok = rms < threshold
    print(f'  质量            {"✅ 可信" if ok else "⚠️ 残差偏大，重摆打点"}'
          f'   (阈值 {threshold} mm)')

    lines = [
        f'# TCP 标定（{"定点法" if method == "point" else "平面约束法"}实测）',
        f'# 打点数 {len(poses)}  工具轴最大夹角 {spread:.1f}°',
        f'# 残差 RMS {rms:.3f} mm  最大 {float(np.max(np.abs(residuals))):.3f} mm',
        f'flange={FLANGE}',
        f'offset_x={t[0]:.6f}',
        f'offset_y={t[1]:.6f}',
        f'offset_z={t[2]:.6f}',
    ]
    if method == 'plane':
        lines += [
            f'table_z={d:.6f}',
            f'table_normal_x={n[0]:.6f}',
            f'table_normal_y={n[1]:.6f}',
            f'table_normal_z={n[2]:.6f}',
        ]
    with open(args.out, 'w') as fh:
        fh.write('\n'.join(lines) + '\n')
    print(f'\n📄 结果已写入 {args.out}')
    return 0 if ok else 2


if __name__ == '__main__':
    sys.exit(main())
