#!/usr/bin/env python3
"""对比刚性(4DOF) vs 仿射(6DOF) 拟合 5 个打点。

仿射能吸收标定中的非刚性误差（尺度/剪切），对桌面抓取更实用。
"""
import json
import math

import numpy as np

marks = json.load(open('/tmp/extrinsic_marks_merged.json'))
G = np.array([m['grid_cm'] for m in marks], float) / 100.0
B = np.array([m['contact_m'][:2] for m in marks], float)


def rms(res):
    return float(np.sqrt(np.mean(np.sum(res ** 2, axis=1)))) * 1000


# --- 刚性 4DOF (Umeyama, scale=1) ---
gc, bc = G.mean(axis=0), B.mean(axis=0)
Gd, Bd = G - gc, B - bc
num = float(np.sum(Gd[:, 0] * Bd[:, 1] - Gd[:, 1] * Bd[:, 0]))
den = float(np.sum(Gd[:, 0] * Bd[:, 0] + Gd[:, 1] * Bd[:, 1]))
th = math.atan2(num, den)
c, s = math.cos(th), math.sin(th)
R = np.array([[c, -s], [s, c]])
t = bc - R @ gc
res_rigid = (R @ G.T).T + t - B
print(f'刚性 4DOF: theta={math.degrees(th):+.3f}deg  '
      f'origin=({t[0]:.4f},{t[1]:.4f})  RMS={rms(res_rigid):.2f} mm')

# --- 仿射 6DOF: B = A @ G + b ---
X = np.hstack([G, np.ones((len(G), 1))])       # N x 3
coef, *_ = np.linalg.lstsq(X, B, rcond=None)   # 3 x 2
pred_aff = X @ coef
res_aff = pred_aff - B
print(f'仿射 6DOF: 残差 RMS={rms(res_aff):.2f} mm')
print(f'  矩阵 A=\n{coef[:2].T}')
print(f'  平移 b={coef[2]}')
print()
print('逐点残差(mm):')
print(f'{"grid":>12}{"刚性":>18}{"仿射":>18}')
for m, r1, r2 in zip(marks, res_rigid, res_aff):
    print(f'{str(m["grid_cm"]):>12}'
          f'({r1[0]*1000:+7.2f},{r1[1]*1000:+7.2f})'
          f'({r2[0]*1000:+7.2f},{r2[1]*1000:+7.2f})')

# 仿射的几何健康检查: A 的两列应大致正交等长
A = coef[:2].T
col0, col1 = A[:, 0], A[:, 1]
print()
print(f'仿射 x 轴列 {col0}, 长度 {np.linalg.norm(col0):.4f}')
print(f'仿射 y 轴列 {col1}, 长度 {np.linalg.norm(col1):.4f}')
print(f'  夹角 {math.degrees(math.acos(np.clip(np.dot(col0, col1) / (np.linalg.norm(col0) * np.linalg.norm(col1)), -1, 1))):.2f}deg (理想 90)')
print(f'  (格宽 0.033m, 长度应≈1.0)')

json.dump({'rigid_theta_deg': math.degrees(th),
           'rigid_origin': list(t),
           'affine_A': coef[:2].tolist(),
           'affine_b': coef[2].tolist(),
           'rms_rigid_mm': rms(res_rigid),
           'rms_affine_mm': rms(res_aff)},
          open('/tmp/fit_compare.json', 'w'), indent=2)
print('\n已写 /tmp/fit_compare.json')
