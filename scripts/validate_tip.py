#!/usr/bin/env python3
"""用"成功抓过物块"的示教位姿校验爪尖定义。

示教数据（quick_taught_grasp.json）：jaws 环住物块时
  taught_tcp_m = (0.27146, 0.03502, -0.05662)
若爪尖定义正确，爪尖应落在该点附近（物块中心），且不低于桌面。
"""

from project_paths import project_path
import os
import sys

import numpy as np

sys.path.insert(0, os.path.expanduser(
    project_path('qianli_ws/src/qianli_vision/scripts')))
from gripper_model import GripperModel, JOINTS

TAUGHT = [-0.11658253987930872, 0.9480001269133262, -0.4586602555778067,
          1.2363885150358267, -1.5508545765523831, 0.3098641191528995]
TAUGHT_TCP = (0.27145880900300084, 0.03502343513643272, -0.05662023769794756)

model = GripperModel(stride=8)
q = dict(zip(JOINTS, TAUGHT))
T = model.solve(q)
print('示教位姿下各坐标系原点:')
for name in ('gripper_link', 'gripper_frame_link', 'tcp_link',
             'moving_jaw_so101_v1_link'):
    M = T.get(name)
    if M is None:
        print(f'  {name}: 无')
        continue
    p = M[:3, 3]
    print(f'  {name:>28} = ({p[0]:.4f}, {p[1]:.4f}, {p[2]*1000:+8.1f} mm)')

print(f'\n记录的 taught_tcp_m (gripper_frame) = '
      f'({TAUGHT_TCP[0]:.4f}, {TAUGHT_TCP[1]:.4f}, {TAUGHT_TCP[2]*1000:+.1f} mm)')

# 各候选"爪尖"定义在该位姿下的世界位置
cands = {}
pts = model.parts['gripper_link']
F = T['gripper_frame_link']
GL = T['gripper_link']
w = (GL[:3, :3] @ pts.T).T + GL[:3, 3]

# 定义1：gripper_link 网格中最深点（工具系常向量）
p_frame = (F[:3, :3].T @ (w - F[:3, 3]).T).T
cands['网格最深点(工具系)'] = p_frame[int(np.argmin(p_frame[:, 2]))]
# 定义2：同一点但直接用世界最低
cands['网格世界最低点'] = None  # 特殊处理
# 定义3：网格中"离 TCP 最远且在下半部"的点
lowmask = p_frame[:, 2] < np.percentile(p_frame[:, 2], 20)
sub = p_frame[lowmask]
cands['下半部最深点'] = sub[int(np.argmin(sub[:, 2]))]

for name, c in cands.items():
    if c is None:
        k = int(np.argmin(w[:, 2]))
        tip = w[k]
        print(f'\n{name}: 世界 ({tip[0]:.4f}, {tip[1]:.4f}, {tip[2]*1000:+.1f}mm)')
        continue
    tip = F[:3, 3] + F[:3, :3] @ c
    dxy = np.hypot(tip[0] - TAUGHT_TCP[0], tip[1] - TAUGHT_TCP[1]) * 1000
    print(f'\n{name}: 工具系 {np.round(c*1000,1).tolist()} mm')
    print(f'  世界 ({tip[0]:.4f}, {tip[1]:.4f}, {tip[2]*1000:+.1f}mm)')
    print(f'  与 taught_tcp 水平距离 {dxy:.1f}mm  '
          f'垂直 {(tip[2]-TAUGHT_TCP[2])*1000:+.1f}mm')

low_all, link_all = model.lowest_over_all(q)
print(f'\n整臂最低点: {link_all} z={low_all[2]*1000:+.1f}mm (桌面 -69.1mm)')
