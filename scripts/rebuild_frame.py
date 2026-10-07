#!/usr/bin/env python3
"""用拖拽实测重建 grid->base 映射，并写入长期坐标系。

依据
----
· 横走 6 格 = 187.0mm（直线度 2.4mm） -> 方向 ex、格边长 31.16mm
· 竖走 3 格 =  94.0mm                -> 同格边长 31.33mm（=> 正方形格）
· 原点：实测 (0,0) 内角点的 base 坐标

映射: base = origin00 + (格数) * CELL * [ex | ey]，其中 ey ⊥ ex（棋盘方格，
      台面内正交），符号按旧仿射的 y 列取向。
标签是"33mm 格"单位，换算：格数 = 标签mm / 33
"""

from project_paths import project_path
import json
import math
import os

import numpy as np

CELL = (187.0 / 6 + 94.0 / 3) / 2 / 1000.0     # 31.245 mm
ex = np.array([0.10980, -0.99395])
ex /= np.linalg.norm(ex)
ey = np.array([-ex[1], ex[0]])
# 与旧仿射 y 列同向
AFF_OLD = np.array([[0.07298559870136852, -1.1941346564477962],
                    [-0.9239523684165104, -0.15094334179736751]])
ay = AFF_OLD[:, 1] / np.linalg.norm(AFF_OLD[:, 1])
if np.dot(ey, ay) < 0:
    ey = -ey
ORIGIN = np.array([0.30270909465110973, 0.03176689026538958])   # (0,0) 实测

# 标签("33mm格") -> base。方向与格宽由拖拽实测固定，原点用 5 个实测点
# 最小二乘定（纯平移拟合：origin = mean(b - A@g)）
A = (CELL / 0.033) * np.column_stack([ex, ey])
MARKS_TMP = [((0.0, 0.0), (0.30270909465110973, 0.03176689026538958)),
             ((9.9, 0.0), (0.3047753911280389, -0.0615763312629367)),
             ((0.0, 6.6), (0.2189, 0.0198)),
             ((9.9, 6.6), (0.2309, -0.0696)),
             ((3.3, 3.3), (0.2582, -0.0086))]
resid_vec = [np.array(bm) - A @ np.array([gx / 100.0, gy / 100.0])
             for (gx, gy), bm in MARKS_TMP]
b = np.mean(resid_vec, axis=0)
print(f'格边长 CELL = {CELL*1000:.2f} mm（拖拽实测）')
print(f'ex = {np.round(ex,5).tolist()}   ey = {np.round(ey,5).tolist()}')
print(f'两轴夹角 {math.degrees(math.acos(np.clip(ex@ey,-1,1))):.2f}° (正交)')
print(f'A =\n{np.round(A,6)}')
print(f'b = {np.round(b,6).tolist()}')

MARKS = [((0.0, 0.0), (0.30270909465110973, 0.03176689026538958)),
         ((9.9, 0.0), (0.3047753911280389, -0.0615763312629367)),
         ((0.0, 6.6), (0.2189, 0.0198)),
         ((9.9, 6.6), (0.2309, -0.0696)),
         ((3.3, 3.3), (0.2582, -0.0086))]
print('\n对照 5 个旧标定点（残差 = 实测 − 新映射预测）：')
tot = 0.0
for (gx, gy), bm in MARKS:
    pred = A @ np.array([gx / 100.0, gy / 100.0]) + b
    r = np.array(bm) - pred
    tot += float(np.dot(r, r))
    print(f'  grid {str((gx,gy)):>12} 残差 ({r[0]*1000:+7.1f},{r[1]*1000:+7.1f}) mm')
print(f'  RMS {math.sqrt(tot/len(MARKS))*1000:.2f} mm')

FRAME = os.path.expanduser(project_path('config/board_frame.json'))
fr = json.load(open(FRAME))
fr['affine'] = [A[0].tolist(), A[1].tolist(), b.tolist()]
fr['affine_source'] = 'drag-measured (2 lines) + (0,0) mark'
fr['cell_mm'] = CELL * 1000
fr['affine_rms_mm'] = None
fr['note'] = ('由拖拽实测重建：格边长 31.2mm（正方形格），方向实测；'
              '原点取 (0,0) 实测点。旧仿射(fit 5点)的格宽各向异性是假象。')
json.dump(fr, open(FRAME, 'w'), indent=2, ensure_ascii=False)
print(f'\n✅ 已写入 {FRAME}')
for name, g in (('黄方块', (12.49, 8.76)),):
    p = A @ np.array([g[0] / 100, g[1] / 100]) + b
    print(f'  {name} 棋盘({g[0]},{g[1]})cm -> base ({p[0]:.4f},{p[1]:.4f})  '
          f'半径 {np.linalg.norm(p)*1000:.0f}mm')
p_old = AFF_OLD @ np.array([0.1249, 0.0876]) + np.array([0.2998701389309053,
                                                        0.030965679822895026])
p_new = A @ np.array([0.1249, 0.0876]) + b
print(f'  旧仿射给 ({p_old[0]:.4f},{p_old[1]:.4f})  →  '
      f'相差 {np.linalg.norm(p_old-p_new)*1000:.1f} mm')
