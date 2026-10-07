#!/usr/bin/env python3
"""检查棋盘 (0,0) 上的方块有没有被移动 —— 只看 (0,0) 附近窗口内的黄色掩码重合。

原理
----
把棋盘 (0,0) 点投影到像素，取其周围 ±WIN 像素的窗口；
窗口内的黄色掩码与"原位参考掩码"求交：
    重合 >= 参考面积的 MIN_OVERLAP  -> 没动 (YES)
    否则                            -> 被拿走 (NO)

用法: check_at_00.py [最小重合比例] [窗口像素半径]   ; --reset 重建参考
"""

from project_paths import project_path
import os
import sys

import cv2
import numpy as np

CFG = os.path.expanduser(project_path('config'))
REF = os.path.join(CFG, 'at00_mask.png')
args = [x for x in sys.argv[1:] if not x.startswith('--')]
MIN_OVERLAP = float(args[0]) if args else 0.10
WIN = int(args[1]) if len(args) > 1 else 70
RESET = '--reset' in sys.argv


def yellow_mask(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, np.array((15, 60, 60)), np.array((40, 255, 255)))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return m


def grab():
    """带超时的相机取帧：绝不挂死/空转。

    相机被锁或 USB 抖动时 cv2.read() 会阻塞，旧实现会永远卡住
    （进程 99% CPU 空转 -> 拖慢流式下发 -> 抓取一卡一卡）。
    这里用线程 + 1.5s 超时兜底，超时即返回 None（本次判定 NO）。
    """
    import threading

    box = {'img': None}

    def _do():
        for idx in (0, 1):
            try:
                cap = cv2.VideoCapture(idx)
            except Exception:
                continue
            if not cap.isOpened():
                cap.release()
                continue
            for _ in range(4):
                try:
                    ok, f = cap.read()
                except Exception:
                    break
                if ok:
                    box['img'] = f
                    cap.release()
                    return
            cap.release()

    t = threading.Thread(target=_do, daemon=True)
    t.start()
    t.join(1.5)
    if t.is_alive():
        return None          # 卡住 -> 交给外层超时/重试
    return box['img']


import json
fr = json.load(open(os.path.join(CFG, 'board_frame.json')))
H = np.array(fr['H'])                      # 像素 -> 棋盘 mm
Hi = np.linalg.inv(H)
v = Hi @ np.array([0.0, 0.0, 1.0])
PX = v[:2] / v[2]                          # 棋盘 (0,0) 的像素位置

img = grab()
if img is None:
    print('NO')
    raise SystemExit(1)
h, w = img.shape[:2]
x0 = int(max(0, min(w - 1, PX[0] - WIN)))
x1 = int(max(1, min(w, PX[0] + WIN)))
y0 = int(max(0, min(h - 1, PX[1] - WIN)))
y1 = int(max(1, min(h, PX[1] + WIN)))
win = img[y0:y1, x0:x1]
mask = yellow_mask(win)

if RESET or not os.path.exists(REF):
    area = cv2.countNonZero(mask)
    # 只在掩码面积是"方块量级"时才建立参考：
    # 放完方块时夹爪还停在 (0,0)，若此时录参考会把爪子黄件录进去，
    # 之后臂一收回就误判"方块被拿走" -> 自激循环
    if area < 600 or area > 5000:
        print(f'RESET_SKIP area={area}')
        raise SystemExit(0)
    os.makedirs(CFG, exist_ok=True)
    cv2.imwrite(REF, mask)
    print(f'RESET_OK area={area}')
    raise SystemExit(0)

ref = cv2.imread(REF, cv2.IMREAD_GRAYSCALE)
if ref is None or ref.shape != mask.shape:
    cv2.imwrite(REF, mask)
    print('YES')
    raise SystemExit(0)

inter = cv2.countNonZero(cv2.bitwise_and(mask, ref))
ref_area = max(1, cv2.countNonZero(ref))
ratio = inter / float(ref_area)
print('YES' if ratio >= MIN_OVERLAP else 'NO')
