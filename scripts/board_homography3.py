#!/usr/bin/env python3
"""抗遮挡棋盘单应 v3：子棋盘 + 已知像素范围定相位。

上一次棋盘完整可见时测得：整盘 7x5 内角点像素范围 x 286..475, y 236..364
（相机不动，该范围长期有效）。黄方块只挡住棋盘左中部，右侧子块仍完整，
于是 findChessboardCorners 找子尺寸即可；用上面的范围反推它是哪一块。
"""
import json
import sys

import cv2
import numpy as np

CELL = 33.0
COLS, ROWS = 7, 5
FLAGS = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
KNOWN = (286.0, 475.0, 236.0, 364.0)     # xmin xmax ymin ymax


def main():
    img = cv2.imread('/tmp/board_src.jpg')
    if img is None:
        cap = cv2.VideoCapture(0)
        for _ in range(20):
            ok, f = cap.read()
            if ok:
                img = f
        cap.release()
    cv2.imwrite('/tmp/board_src.jpg', img)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.createCLAHE(2.0, (8, 8)).apply(gray)

    cands = []
    for cw in range(3, COLS + 1):
        for ch in range(3, ROWS + 1):
            if cw == COLS and ch == ROWS:
                continue
            found, corners = cv2.findChessboardCorners(gray, (cw, ch), FLAGS)
            if found:
                cands.append((cw * ch, cw, ch,
                              corners.reshape(-1, 2).astype(np.float64)))
    if not cands:
        print('❌ 连子棋盘都没找到')
        return 1
    cands.sort(reverse=True)
    print(f'找到 {len(cands)} 个子棋盘，最大 {cands[0][1]}x{cands[0][2]} '
          f'({cands[0][0]} 角点)')

    best = None
    for n, cw, ch, corners in cands:
        corners = cv2.cornerSubPix(
            gray, corners.reshape(-1, 1, 2).astype(np.float32), (7, 7), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        ).reshape(-1, 2).astype(np.float64)
        for ox in range(COLS - cw + 1):
            for oy in range(ROWS - ch + 1):
                obj = np.zeros((cw * ch, 2), np.float32)
                obj[:, 0] = (np.tile(np.arange(cw), ch) + ox) * CELL
                obj[:, 1] = (np.repeat(np.arange(ch), cw) + oy) * CELL
                H, _ = cv2.findHomography(corners.astype(np.float32), obj)
                if H is None:
                    continue
                # 整盘 35 个内角点映到像素，看范围是否匹配已知
                full = np.zeros((COLS * ROWS, 2), np.float32)
                full[:, 0] = np.tile(np.arange(COLS), ROWS) * CELL
                full[:, 1] = np.repeat(np.arange(ROWS), COLS) * CELL
                inv = np.linalg.inv(H)
                v = (inv @ np.hstack([full, np.ones((len(full), 1))]).T).T
                p = v[:, :2] / v[:, 2:3]
                err = (abs(p[:, 0].min() - KNOWN[0]) + abs(p[:, 0].max() - KNOWN[1])
                       + abs(p[:, 1].min() - KNOWN[2]) + abs(p[:, 1].max() - KNOWN[3]))
                if best is None or err < best[0]:
                    best = (err, H, (cw, ch), (ox, oy), p)
    err, H, size, off, p = best
    print(f'最佳: 子棋盘 {size[0]}x{size[1]} 偏移({off[0]},{off[1]})  '
          f'范围匹配误差合计 {err:.1f}px')
    print(f'  预测整盘像素范围 x {p[:,0].min():.0f}..{p[:,0].max():.0f} '
          f'y {p[:,1].min():.0f}..{p[:,1].max():.0f}')
    print(f'  已知范围            x {KNOWN[0]:.0f}..{KNOWN[1]:.0f} '
          f'y {KNOWN[2]:.0f}..{KNOWN[3]:.0f}')
    if err > 40:
        print('❌ 匹配太差，放弃')
        return 1
    json.dump({'H': H.tolist(), 'cell_mm': CELL, 'sub': list(size),
               'offset': list(off), 'match_err_px': float(err)},
              open('/tmp/board_homography.json', 'w'), indent=2)
    print('✅ 已写 /tmp/board_homography.json')

    # 顺带把黄方块定位出来
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, np.array((20, 90, 90)), np.array((34, 255, 255)))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    MARKS = [((0.0, 0.0), (0.2915, 0.0296)), ((9.9, 0.0), (0.3050, -0.0688)),
             ((0.0, 6.6), (0.2260, 0.0168)), ((9.9, 6.6), (0.2323, -0.0710)),
             ((3.3, 3.3), (0.2582, -0.0086))]
    G = np.array([x[0] for x in MARKS], float) / 100.0
    B = np.array([x[1] for x in MARKS], float)
    aff, *_ = np.linalg.lstsq(np.hstack([G, np.ones((len(G), 1))]), B, rcond=None)
    print('\n黄色候选:')
    for c in cnts:
        a = cv2.contourArea(c)
        if a < 350:
            continue
        x, y, w, hh = cv2.boundingRect(c)
        if not 0.6 <= w / float(hh) <= 1.7:
            continue
        cx, cy = x + w / 2, y + hh / 2
        v = H @ np.array([cx, cy, 1.0])
        v = v[:2] / v[2]
        gx, gy = v[0] / 10.0, v[1] / 10.0
        bx, by = aff[:2].T @ np.array([gx / 100, gy / 100]) + aff[2]
        onb = -0.5 <= v[0] / CELL <= COLS - 0.5 and -0.5 <= v[1] / CELL <= ROWS - 0.5
        print(f'  像素({cx:.0f},{cy:.0f}) {max(w,hh):.0f}px  '
              f'棋盘({gx:+.2f},{gy:+.2f})cm  base({bx:.4f},{by:.4f})  '
              f'{"在棋盘上" if onb else "棋盘外"}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
