#!/usr/bin/env python3
"""从 /state 快照重新生成内参 yaml

为什么需要单独一个工具
----------------------
标定前端的 25 张视图只在**内存**里。一旦前端重启（改代码、崩了、机器重启），
视图就没了，`solve()` 也就重跑不出来。而 `/tmp/s.json` 这种 `/state` 快照里
**同时含 result 和 coverage**，足以把 yaml 完整重建。

所以：标定结果一定要及时落盘；这个脚本就是"没来得及落盘"时的补救路径。

用法::

    ~/mj/bin/python intrinsics_from_state.py --state /tmp/s.json \
        --out /tmp/camera_intrinsics.yaml
"""

from __future__ import annotations

from project_paths import calibration_path

import argparse
import json
import sys
import time

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--state', default='/tmp/s.json')
    ap.add_argument('--out', default=calibration_path('camera_intrinsics.yaml'))
    ap.add_argument('--cols', type=int, default=7)
    ap.add_argument('--rows', type=int, default=5)
    ap.add_argument('--cell-mm', type=float, default=33.0)
    ap.add_argument('--cov-rows', type=int, default=4)
    ap.add_argument('--cov-cols', type=int, default=5)
    args = ap.parse_args()

    s = json.load(open(args.state))
    r = s.get('result')
    if not r:
        print('❌ 快照里没有 result，无法重建')
        return 1

    ok = str(r.get('verdict', '')).startswith('✅')
    verdict = r.get('verdict', '').replace('<b>', '').replace('</b>', '')
    sb, sa = r['straight_before'], r['straight_after']

    lines = [
        f'# SO-101 相机内参标定（由 {args.state} 快照重建）'
        f'  {time.strftime("%Y-%m-%d %H:%M:%S")}',
        f'# 棋盘 {args.cols}x{args.rows} 内角点，格边长 {args.cell_mm}mm，'
        f'{r["n_views"]} 张',
        f'# calibrateCamera RMS = {r["rms"]:.4f} px',
        f'# 直线度 RMS(px)  去畸变前 行{sb["row"]:.3f} 列{sb["col"]:.3f} '
        f'最大{sb["max"]:.3f}',
        f'#                去畸变后 行{sa["row"]:.3f} 列{sa["col"]:.3f} '
        f'最大{sa["max"]:.3f}',
        f'# 边缘角点数 {r.get("edge_corners")}   直线度改善 '
        f'{r.get("straight_improve", 0)*100:.1f}%',
        f'quality_ok={1 if ok else 0}',
        f'# 质量裁决: {verdict}',
        f'image_width: {r["image_size"][0]}',
        f'image_height: {r["image_size"][1]}',
        'camera_matrix:',
    ]
    for row in r['camera_matrix']:
        lines.append('  - [' + ', '.join(f'{v:.8f}' for v in row) + ']')
    lines.append('distortion_coefficients:')
    lines.append('  - [' + ', '.join(f'{v:.10f}' for v in r['dist_coeffs'])
                  + ']')

    cov = s.get('coverage')
    if cov:
        c = np.asarray(cov)
        cnz = np.nonzero(c.sum(axis=0))[0]
        rnz = np.nonzero(c.sum(axis=1))[0]
        if len(cnz) and len(rnz):
            lines += [
                '# 可信区域：标定时角点实际覆盖到的画面范围（归一化 0~1）。',
                '# 畸变在边缘最大，覆盖之外的区域去畸变属于外推，结果不可信。',
                f'trusted_x0={cnz[0]/args.cov_cols:.4f}',
                f'trusted_x1={(cnz[-1]+1)/args.cov_cols:.4f}',
                f'trusted_y0={rnz[0]/args.cov_rows:.4f}',
                f'trusted_y1={(rnz[-1]+1)/args.cov_rows:.4f}',
            ]

    open(args.out, 'w').write('\n'.join(lines) + '\n')
    print(f'📄 已写入 {args.out}')
    print(f'   quality_ok={1 if ok else 0}   RMS={r["rms"]:.4f}   '
          f'k1={r["dist_coeffs"][0]:+.4f}')
    print(f'   裁决: {verdict}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
