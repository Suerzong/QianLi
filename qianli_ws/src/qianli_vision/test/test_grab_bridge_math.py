#!/usr/bin/env python3
"""grab_bridge 坐标变换的数学验证（不需要相机/机械臂）

验证：grid 系点 → 旋转 θ + 平移 (x0,y0) → base_link 系

测试用例：
  外参：x0=0.20, y0=0.0, z0=0.02, θ=0°
  物块 grid=(0.26, 0.25) m（即 26cm, 25cm）
  期望：base_link = (0.20+0.26, 0+0.25, 0.02+0.10) = (0.46, 0.25, 0.12)

  再测旋转：θ=90° 时 grid 的 x 轴指向 base_link 的 +y
"""

import math
import sys

sys.path.insert(0, 'qianli_vision')


def grid_to_base(gx, gy, x0, y0, z0, theta_deg, approach_z):
    """复现 grab_bridge.on_object_pose 的变换逻辑。"""
    theta = math.radians(theta_deg)
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    bx = cos_t * gx - sin_t * gy + x0
    by = sin_t * gx + cos_t * gy + y0
    bz = z0 + approach_z
    return bx, by, bz


def check(name, got, expected, tol=1e-6):
    ok = all(abs(g - e) < tol for g, e in zip(got, expected))
    print(f'{"✅" if ok else "❌"} {name}: got={tuple(round(v,4) for v in got)} '
          f'expected={expected}')
    return ok


ok_all = True

# 用例1：θ=0°，纯平移
ok_all &= check('θ=0° 平移', grid_to_base(0.26, 0.25, 0.20, 0.0, 0.02, 0, 0.10),
                (0.46, 0.25, 0.12))

# 用例2：θ=90°，grid x 轴 → base_link +y，grid y 轴 → base_link -x
# grid=(0.1, 0) → base = (0.20, 0.10) 附近
bx, by, bz = grid_to_base(0.1, 0.0, 0.20, 0.0, 0.02, 90, 0.10)
ok_all &= check('θ=90° 旋转', (bx, by, bz), (0.20, 0.10, 0.12))

# 用例3：θ=45°，混合旋转平移
bx, by, bz = grid_to_base(0.1, 0.0, 0.20, 0.05, 0.02, 45, 0.10)
exp_bx = 0.20 + 0.1 * math.cos(math.radians(45))
exp_by = 0.05 + 0.1 * math.sin(math.radians(45))
ok_all &= check('θ=45° 混合', (bx, by, bz), (exp_bx, exp_by, 0.12))

print()
print('全部通过 ✅' if ok_all else '存在失败 ❌')
sys.exit(0 if ok_all else 1)
