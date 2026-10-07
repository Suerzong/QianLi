#!/usr/bin/env python3
"""长期棋盘坐标系：建一次，长期复用。

--capture : 需要棋盘完整可见(7x5 内角点全找到) -> 存单应 + 仿射换算
--locate  : 用已存坐标系定位颜色方块 -> 棋盘坐标 + base_link 坐标
--show    : 打印当前坐标系信息

持久化路径（不放 /tmp，避免被清）:
  ~/QianLi/qianli_ws/config/board_frame.json
  ~/QianLi/qianli_ws/config/board_frame.npz   (单应矩阵)
"""
import argparse
import json
import math
import os
import sys

import cv2
import numpy as np

CFG_DIR = os.path.expanduser('~/QianLi/qianli_ws/config')
JSON_PATH = os.path.join(CFG_DIR, 'board_frame.json')
NPZ_PATH = os.path.join(CFG_DIR, 'board_frame.npz')

CELL = 33.0
COLS, ROWS = 7, 5
FLAGS = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)

# 5 组实测（棋盘坐标 cm -> base_link m），用于 grid->base 仿射
MARKS = [((0.0, 0.0), (0.2915, 0.0296)),
         ((9.9, 0.0), (0.3050, -0.0688)),
         ((0.0, 6.6), (0.2260, 0.0168)),
         ((9.9, 6.6), (0.2323, -0.0710)),
         ((3.3, 3.3), (0.2582, -0.0086))]

SPEC = {'blue': [((95, 90, 60), (135, 255, 255))],
        'green': [((35, 80, 60), (85, 255, 255))],
        'yellow': [((15, 60, 60), (40, 255, 255))],
        'red': [((0, 100, 70), (8, 255, 255)), ((170, 100, 70), (179, 255, 255))],
        'purple': [((136, 60, 60), (168, 255, 255))]}


def affine_from_marks():
    G = np.array([m[0] for m in MARKS], float) / 100.0
    B = np.array([m[1] for m in MARKS], float)
    X = np.hstack([G, np.ones((len(G), 1))])
    coef, *_ = np.linalg.lstsq(X, B, rcond=None)
    resid = X @ coef - B
    rms = float(np.sqrt(np.mean(np.sum(resid ** 2, axis=1)))) * 1000
    return coef, rms


def grab(n=15):
    cap = cv2.VideoCapture(0)
    img = None
    for _ in range(n):
        ok, f = cap.read()
        if ok:
            img = f
    cap.release()
    return img


def full_board_H(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, (COLS, ROWS), FLAGS)
    if not found:
        return None, None
    corners = cv2.cornerSubPix(
        gray, corners, (7, 7), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
    obj = np.zeros((COLS * ROWS, 2), np.float32)
    obj[:, 0] = np.tile(np.arange(COLS), ROWS) * CELL
    obj[:, 1] = np.repeat(np.arange(ROWS), COLS) * CELL
    H, _ = cv2.findHomography(corners.reshape(-1, 2).astype(np.float32), obj)
    c = corners.reshape(-1, 2)
    return H, (float(c[:, 0].min()), float(c[:, 0].max()),
               float(c[:, 1].min()), float(c[:, 1].max()))


def load_frame():
    if not os.path.exists(JSON_PATH):
        return None, None
    meta = json.load(open(JSON_PATH))
    H = np.load(NPZ_PATH)['H'] if os.path.exists(NPZ_PATH) else np.array(meta['H'])
    return meta, H


def save_frame(H, px, extra=None):
    os.makedirs(CFG_DIR, exist_ok=True)
    coef, rms = affine_from_marks()
    meta = {'cell_mm': CELL, 'cols': COLS, 'rows': ROWS,
            'pixel_extent': list(px), 'H': H.tolist(),
            'affine': coef.tolist(), 'affine_rms_mm': rms,
            'note': '棋盘坐标系（相机固定时长期有效）'}
    if extra:
        meta.update(extra)
    json.dump(meta, open(JSON_PATH, 'w'), indent=2)
    np.savez(NPZ_PATH, H=H)
    print(f'✅ 已保存长期坐标系 → {JSON_PATH}')
    print(f'   grid->base 仿射残差 RMS {rms:.2f} mm')


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=False)
    g.add_argument('--capture', action='store_true')
    g.add_argument('--locate', action='store_true')
    g.add_argument('--show', action='store_true')
    ap.add_argument('--promote-quad', help='把 /tmp/board_homography.json 里的'
                                           '抗遮挡单应提升为长期坐标系')
    ap.add_argument('--color', default=None)
    a = ap.parse_args()
    if not (a.capture or a.locate or a.show or a.promote_quad):
        ap.error('需要 --capture / --locate / --show / --promote-quad 之一')

    if a.show:
        meta, H = load_frame()
        if meta is None:
            print('尚未建立坐标系')
            return 0
        print(json.dumps({k: v for k, v in meta.items() if k != 'H'},
                         indent=2, ensure_ascii=False))
        return 0

    if a.promote_quad:
        src = json.load(open(a.promote_quad))
        H = np.array(src['H'])
        save_frame(H, src.get('pixel_extent',
                              [284, 476, 236, 368]),
                   extra={'source': 'occlusion-tolerant (SB sub-board)',
                          'match_err_px': src.get('match_err_px')})
        return 0

    img = grab()
    if img is None:
        print('取帧失败')
        return 1

    if a.capture:
        H, px = full_board_H(img)
        if H is None:
            print('❌ 棋盘未完整可见（7x5 内角点没全找到）——'
                  '请先把方块从棋盘上拿开再 capture')
            return 1
        print(f'棋盘完整可见 ✅  内角点像素范围 x {px[0]:.0f}..{px[1]:.0f} '
              f'y {px[2]:.0f}..{px[3]:.0f}')
        save_frame(H, px)
        return 0

    # --locate
    meta, H = load_frame()
    if H is None:
        print('❌ 还没有长期坐标系，先跑 --capture')
        return 1
    coef = np.array(meta['affine'])
    _r = meta.get('affine_rms_mm')
    _rtxt = f'{float(_r):.2f}mm' if _r is not None else '拖拽实测重建'
    print(f'使用长期坐标系（{meta.get("affine_source", "capture")}，'
          f'仿射 RMS {_rtxt}）')
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    print(f'\n{"颜色":>7}{"像素":>13}{"边长":>7}{"棋盘cm":>17}{"base(m)":>21}'
          f'{"半径":>7}  判定')
    hits = []
    for name, rngs in SPEC.items():
        if a.color and name != a.color:
            continue
        mask = None
        for lo_, hi_ in rngs:
            m = cv2.inRange(hsv, np.array(lo_), np.array(hi_))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        for cnt in cnts:
            area = cv2.contourArea(cnt)
            if area < 350:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            if not 0.6 <= w / float(h) <= 1.7:
                continue
            cx, cy = x + w / 2, y + h / 2
            v = H @ np.array([cx, cy, 1.0])
            v = v[:2] / v[2]
            gx_cm, gy_cm = v[0] / 10.0, v[1] / 10.0
            bx, by = coef[:2].T @ np.array([gx_cm / 100, gy_cm / 100]) + coef[2]
            rad = math.hypot(bx, by) * 1000
            onb = (-0.5 <= v[0] / CELL <= COLS - 0.5
                   and -0.5 <= v[1] / CELL <= ROWS - 0.5)
            print(f'{name:>7}{f"({cx:.0f},{cy:.0f})":>13}{max(w,h):>6.0f}px'
                  f'{f"({gx_cm:+.2f},{gy_cm:+.2f})":>17}'
                  f'{f"({bx:.4f},{by:.4f})":>21}{rad:>7.0f}  '
                  f'{"棋盘上" if onb else "棋盘外"}'
                  f'{" 可达" if rad < 360 else " 偏远"}')
            hits.append({'color': name, 'px': [float(cx), float(cy)],
                         'grid_cm': [float(gx_cm), float(gy_cm)],
                         'base_m': [float(bx), float(by)],
                         'radius_mm': float(rad), 'on_board': bool(onb),
                         'span_px': int(max(w, h))})
    json.dump(hits, open('/tmp/blocks_located.json', 'w'), indent=2)
    print(f'\n共 {len(hits)} 个候选 → /tmp/blocks_located.json')
    return 0


if __name__ == '__main__':
    sys.exit(main())
