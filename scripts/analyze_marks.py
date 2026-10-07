
from project_paths import calibration_path
#!/usr/bin/env python3
import json
import numpy as np

marks = json.load(open(calibration_path('extrinsic_marks.json')))
B = -0.06859
print('点 | grid(cm)    | contact xy          | z-板面mm | TCP-接触水平mm')
for i, m in enumerate(marks, 1):
    c = np.array(m['contact_m'])
    t = np.array(m['base_m'])
    dz = (c[2] - B) * 1000
    dh = np.hypot(c[0] - t[0], c[1] - t[1]) * 1000
    print(f'{i:2d} | {str(m["grid_cm"]):>11} | ({c[0]:.4f},{c[1]:.4f}) '
          f'| {dz:+6.2f} | {dh:6.2f}')
