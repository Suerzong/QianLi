#!/usr/bin/env python3
"""数值验证梯形剖面 ramp(s)：单调、连续、端值正确、速度平滑。"""
import numpy as np

f = 0.13


def ramp(s):
    if s < f:
        return (s / 2 - f / (2 * np.pi) * np.sin(np.pi * s / f)) / (1 - f)
    if s <= 1 - f:
        return (s - f / 2) / (1 - f)
    u = s - (1 - f)
    return (1 - 1.5 * f + u / 2 + f / (2 * np.pi) * np.sin(np.pi * u / f)) / (1 - f)


ss = np.linspace(0.0, 1.0, 2001)
p = np.array([ramp(s) for s in ss])
v = np.diff(p) / (ss[1] - ss[0])
a = np.diff(v) / (ss[1] - ss[0])
print(f'ramp(0)={ramp(0):.6f}  ramp(1)={ramp(1):.6f}')
print(f'位移单调递增: {np.all(np.diff(p) >= 0)}')
print(f'巡航中点 ramp(0.5)={ramp(0.5):.6f} '
      f'(期望 {(0.5-0.13/2)/(1-0.13):.6f})')
print(f'速度范围: [{v.min():.4f}, {v.max():.4f}]  (期望巡航≈1, 端≈0)')
i_f = int(f * 2000)
print(f'速度在 f 处连续性: v(f-)={v[i_f]:.4f} v(f+)={v[i_f+1]:.4f}')
print(f'速度在 1-f 处连续性: v(1-f-)={v[-int(f*2000)-2]:.4f} '
      f'v(1-f+)={v[-int(f*2000)-1]:.4f}')
print('OK' if np.all(np.diff(p) >= 0) and abs(ramp(1) - 1) < 1e-9 else 'FAIL')
