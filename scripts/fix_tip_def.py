#!/usr/bin/env python3
"""纠正"固定爪尖端"定义：用示教抓取位交叉验证。

判据：在示教抓取位（已知夹爪正环住方块）下，合格的"爪尖"必须是
      gripper_link 网格的**世界最低点**，且在 TCP 下方约 6mm。
"""

from project_paths import project_path
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.expanduser(
    project_path('qianli_ws/src/qianli_vision/scripts')))
from gripper_model import GripperModel, JOINTS, FLANGE_LINK

TEACH = np.array([-0.11658253987930872, 0.9480001269133262,
                  -0.4586602555778067, 1.2363885150358267,
                  -1.5508545765523831, 0.3098641191528995])
model = GripperModel(stride=6)
T0 = model.solve(dict(zip(JOINTS, np.zeros(6))))
F0, G0 = T0['gripper_frame_link'], T0[FLANGE_LINK]
w0 = (G0[:3, :3] @ model.parts[FLANGE_LINK].T).T + G0[:3, 3]
pf = (F0[:3, :3].T @ (w0 - F0[:3, 3]).T).T      # gripper_link 网格，工具系
print(f'gripper_link 网格在工具系: '
      f'x {pf[:,0].min()*1000:+.1f}..{pf[:,0].max()*1000:+.1f}  '
      f'y {pf[:,1].min()*1000:+.1f}..{pf[:,1].max()*1000:+.1f}  '
      f'z {pf[:,2].min()*1000:+.1f}..{pf[:,2].max()*1000:+.1f} mm')

T = model.solve(dict(zip(JOINTS, TEACH)))
F, G = T['gripper_frame_link'], T[FLANGE_LINK]
w = (G[:3, :3] @ model.parts[FLANGE_LINK].T).T + G[:3, 3]
k = int(np.argmin(w[:, 2]))
print(f'\n示教抓取位: TCP z = {F[2,3]*1000:+.1f}mm')
print(f'  gripper_link 世界最低点 ({w[k,0]:.4f},{w[k,1]:.4f},'
      f'{w[k,2]*1000:+.2f}mm)  比 TCP 低 {(F[2,3]-w[k,2])*1000:.1f}mm')

for tag, cand in (('取 z 最小(我之前的错误做法)', pf[int(np.argmin(pf[:,2]))]),
                  ('取 z 最大', pf[int(np.argmax(pf[:,2]))])):
    p = F[:3, 3] + F[:3, :3] @ cand
    print(f'\n{tag}: 工具系 {np.round(cand*1000,1).tolist()} mm')
    print(f'  -> 示教位世界 ({p[0]:.4f},{p[1]:.4f},{p[2]*1000:+.2f}mm)  '
          f'比 TCP {"高" if p[2] > F[2,3] else "低"} '
          f'{abs(p[2]-F[2,3])*1000:.1f}mm')
    err = np.linalg.norm(p - w[k]) * 1000
    print(f'  与世界最低点差 {err:.1f}mm  {"✅ 对" if err < 3 else "❌ 错"}')
