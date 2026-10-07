#!/usr/bin/env python3
"""四物块桌面位态估计（棋盘单应 → grid 平面 yaw）

输入一张"棋盘 + 四个彩色物块"的图：
  1. 检出 7x5 棋盘内角点 → findHomography H（像素 → grid 厘米，格 3.3cm）
  2. 用 color_block_detect 检出彩色物块（复用项目检测器）
  3. 每个物块：minAreaRect 四角经 H 映射到 grid 平面 → 求边方向角 → yaw（[-45,45)）
  4. 输出 grid 系 (color, Xcm, Ycm, yaw_deg) 表格 + 标注图 + JSON

用法：
    ~/mj/bin/python pose_est_4blocks.py --image a.png --out b.png --json-out b.json
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import color_block_detect as cd  # noqa: E402

CELL_CM = 3.3
BOARD_COLS, BOARD_ROWS = 7, 5
_CB_FLAGS = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
_SUBPIX = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def board_homography(gray):
    """像素 → grid(厘米)。返回 (H, corners) 或 (None, None)。"""
    ok, corners = cv2.findChessboardCorners(gray, (BOARD_COLS, BOARD_ROWS),
                                            _CB_FLAGS)
    if not ok:
        return None, None
    corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), _SUBPIX)
    pts = corners.reshape(-1, 2)
    src, dst = [], []
    for i in range(BOARD_ROWS):
        for j in range(BOARD_COLS):
            px, py = pts[i * BOARD_COLS + j]
            src.append((px, py))
            dst.append((j * CELL_CM, i * CELL_CM))
    src = np.array(src, np.float32)
    dst = np.array(dst, np.float32)
    H, _inl = cv2.findHomography(src, dst, cv2.RANSAC, 0.15)
    if H is None:
        return None, None

    def _quality(H, s, d):
        proj = cv2.perspectiveTransform(s.reshape(-1, 1, 2), H).reshape(-1, 2)
        err = np.linalg.norm(proj - d, axis=1)
        return err.mean(), err.max(), err

    # 质量校验（同 object_localizer）。容忍单个坏角点：超过 0.3cm 就丢弃
    # 误差最大的那个角点重拟合一次（RANSAC 有时会放行一个离群角点）。
    mean, mx, err = _quality(H, src, dst)
    if mx > 0.3:
        keep = np.argmax(err)
        mask = np.ones(len(src), bool)
        mask[keep] = False
        H2, _inl2 = cv2.findHomography(src[mask], dst[mask], cv2.RANSAC, 0.15)
        if H2 is not None:
            m2, x2, _ = _quality(H2, src, dst)
            if x2 < mx:  # 重拟合确实更干净 → 采用
                H, mean, mx = H2, m2, x2
    if mean > 0.1 or mx > 0.5:
        return None, None
    return H, (dst, float(mean), float(mx))


def block_yaw_grid(box_px, H):
    """minAreaRect 四角(像素) → grid 平面 → 边方向角 → yaw ∈ [-45,45)。"""
    g = cv2.perspectiveTransform(
        np.asarray(box_px, np.float64).reshape(-1, 1, 2), H).reshape(-1, 2)
    # 取较长边（boxPoints 不保证边序），与 color_block_detect 一致
    e1 = g[1] - g[0]
    e2 = g[2] - g[1]
    edge = e1 if math.hypot(float(e1[0]), float(e1[1])) >= \
        math.hypot(float(e2[0]), float(e2[1])) else e2
    a = math.degrees(math.atan2(float(edge[1]), float(edge[0])))
    return (a + 45.0) % 90.0 - 45.0, g


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--image', required=True)
    ap.add_argument('--out', help='标注图输出路径')
    ap.add_argument('--json-out', help='JSON 输出路径')
    ap.add_argument('--colors', default=None,
                    help='颜色表（默认用 color_block_detect 默认表）')
    a = ap.parse_args()

    img = cv2.imread(a.image, cv2.IMREAD_COLOR)
    if img is None:
        print(f'读不到图片：{a.image}')
        return 2
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    H, corners = board_homography(gray)
    if H is None:
        print('❌ 图里没有可用的 7x5 棋盘 → 无法求桌面 yaw（换有棋盘的图）')
        return 1
    _pts, err_mean, err_max = corners
    print(f'棋盘单应重投影误差：平均 {err_mean:.3f}cm / 最大 {err_max:.3f}cm')

    hue_centers = dict(cd.DEFAULT_HUE_CENTERS)
    colors = a.colors if a.colors else ','.join(cd.DEFAULT_COLOR_TABLE)
    color_table = cd.parse_color_table(colors, hue_centers)
    args = cd.parse_args(['--image', 'x'] + (['--colors', colors] if a.colors else []))
    blocks, mask, info = cd.detect_blocks(img, args, color_table, hue_centers)

    rows = []
    vis = img.copy()
    for i, b in enumerate(blocks):
        rect = cv2.minAreaRect(np.asarray(b['contour'], np.int32))
        box = cv2.boxPoints(rect)
        yaw_deg, g_box = block_yaw_grid(box, H)
        gx_cm, gy_cm = cv2.perspectiveTransform(
            np.array([[[b['cx'], b['cy']]]], np.float64), H)[0][0]
        rows.append({
            'color': b['color'],
            'X_cm': round(float(gx_cm), 2),
            'Y_cm': round(float(gy_cm), 2),
            'yaw_deg': round(float(yaw_deg), 2),
            'conf': round(b['confidence'], 3),
            'cx_px': round(b['cx'], 1),
            'cy_px': round(b['cy'], 1),
        })
        # 标注：grid 里的框 + yaw 文字
        box_i = np.int32(cv2.boxPoints(
            cv2.minAreaRect(g_box.astype(np.float32))))
        cv2.drawContours(vis, [box_i], -1, (255, 255, 0), 2)
        cv2.circle(vis, (int(b['cx']), int(b['cy'])), 4, (0, 0, 255), -1)
        cv2.putText(vis, f"#{i} {b['color']} yaw={yaw_deg:+.1f}deg "
                         f"({gx_cm:.1f},{gy_cm:.1f})cm",
                    (max(2, int(b['cx']) - 60), max(14, int(b['cy']) - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(vis, f"#{i} {b['color']} yaw={yaw_deg:+.1f}deg "
                         f"({gx_cm:.1f},{gy_cm:.1f})cm",
                    (max(2, int(b['cx']) - 60), max(14, int(b['cy']) - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1, cv2.LINE_AA)

    print('=== 四物块桌面位态估计（grid 系：棋盘原点=左上内角点，格 3.3cm）===')
    print(f'棋盘单应重投影误差：平均/最大见下')
    print(f'{"颜色":8s} {"X(cm)":>7s} {"Y(cm)":>7s} {"yaw(deg)":>9s} {"conf":>6s}')
    for r in rows:
        print(f"{r['color']:8s} {r['X_cm']:7.2f} {r['Y_cm']:7.2f} "
              f"{r['yaw_deg']:+9.2f} {r['conf']:6.3f}")
    print(f'\n物块数：{len(rows)}')
    if a.json_out:
        with open(a.json_out, 'w', encoding='utf-8') as fh:
            json.dump({'image': a.image, 'cell_cm': CELL_CM,
                       'frame': 'grid', 'blocks': rows}, fh,
                      ensure_ascii=False, indent=2)
        print(f'JSON → {a.json_out}')
    if a.out:
        cv2.imwrite(a.out, vis)
        print(f'标注图 → {a.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
