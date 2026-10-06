#!/usr/bin/env python3
"""内参验证：原图 vs 去畸变，并量化位移量

为什么还要看这个
----------------
直线度改善 85% 是定量证据，但去畸变**是否过度**（把直线反向弯成桶形）
光看数字看不出来。这里并排 + 量化最大位移，一眼就能判断。
"""

from __future__ import annotations

import sys
import time

import cv2
import numpy as np

YAML = '/tmp/camera_intrinsics.yaml'


def load_yaml(path):
    """极简解析：只为读本项目的内参文件，不引 PyYAML。

    注意：**不能用 `s.strip('- []')` 取数** —— strip 会剥掉字符集合里的
    所有字符，包括第一个数的负号！实测把 k1=-0.4622 读成了 +0.4622，
    畸变方向整个反过来。必须按括号切片再 split。
    """
    K, D, size = None, None, [None, None]
    lines = open(path, encoding='utf-8-sig').read().splitlines()
    mats, cur = [], None
    for ln in lines:
        s = ln.strip()
        if s.startswith('image_width:'):
            size[0] = int(float(s.split(':')[1]))
        elif s.startswith('image_height:'):
            size[1] = int(float(s.split(':')[1]))
        elif s.startswith('camera_matrix:'):
            cur = 'K'
        elif s.startswith('distortion_coefficients:'):
            cur = 'D'
        elif s.startswith('- [') and cur in ('K', 'D'):
            body = s[s.index('[') + 1: s.rindex(']')]
            vals = [float(v) for v in body.split(',')]
            if cur == 'K':
                mats.append(vals)
            else:
                D = np.array(vals)
    if mats:
        K = np.array(mats)
    return K, D, size


def grab_from_stream(url='http://127.0.0.1:8098/stream.mjpg', timeout=15):
    """从标定前端的 MJPEG 流里取一帧。

    为什么不直接开相机：标定前端正占着 /dev/video0，再开会报
    "can't open camera by index"。走它的流既不抢设备，也保证拿到的是
    同一路画面。
    """
    import urllib.request
    req = urllib.request.Request(url)
    buf = b''
    with urllib.request.urlopen(req, timeout=timeout) as r:
        while len(buf) < 4_000_000:
            chunk = r.read(4096)
            if not chunk:
                break
            buf += chunk
            i = buf.find(b'\xff\xd8')          # JPEG SOI
            j = buf.find(b'\xff\xd9', i + 2)   # JPEG EOI
            if i >= 0 and j > i:
                arr = np.frombuffer(buf[i:j + 2], np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if img is not None:
                    return img
    return None


def main():
    K, D, size = load_yaml(YAML)
    if K is None or D is None:
        print('❌ 内参解析失败')
        return 1
    w, h = size
    print(f'内参: {w}x{h}  fx={K[0,0]:.1f} fy={K[1,1]:.1f} '
          f'cx={K[0,2]:.1f} cy={K[1,2]:.1f}')
    print(f'畸变: k1={D[0]:+.4f} k2={D[1]:+.4f} '
          f'p1={D[2]:+.5f} p2={D[3]:+.5f} k3={D[4]:+.4f}')

    img = grab_from_stream()
    if img is None:
        # 退路：直接开相机（前端没在跑时用）
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        time.sleep(1.0)
        for _ in range(10):
            cap.read()
        ok, img = cap.read()
        cap.release()
        if not ok:
            print('❌ 既取不到前端流，也打不开相机')
            return 1
        print('帧来源：直接开相机')
    else:
        print(f'帧来源：标定前端 MJPEG 流  {img.shape[1]}x{img.shape[0]}')
    if img.shape[1] != w or img.shape[0] != h:
        print(f'⚠️ 帧尺寸 {img.shape[1]}x{img.shape[0]} 与内参 {w}x{h} 不一致')
        return 1

    # 去畸变位移场。
    #
    # 不能用 undistortPoints：它把输入当**畸变**点，方向正好相反；
    # 而且边缘处反向迭代会发散（实测给出 331707 px 这种荒谬值）。
    # initUndistortRectifyMap 是 OpenCV 自己用的正式路径：
    # map 给出"输出(去畸变)像素 → 输入(畸变)像素"，两者之差就是校正量。
    map1, map2 = cv2.initUndistortRectifyMap(K, D, None, K, (w, h), cv2.CV_32FC1)
    xs, ys = np.meshgrid(np.arange(w, dtype=np.float32),
                         np.arange(h, dtype=np.float32))
    dx = map1 - xs
    dy = map2 - ys
    # 映射到图像外的像素（map=-1 之类）会给出假的巨大值，要剔掉
    valid = (map1 >= 0) & (map1 < w) & (map2 >= 0) & (map2 < h)
    disp = np.sqrt(dx ** 2 + dy ** 2)
    disp_v = np.where(valid, disp, np.nan)
    print()
    print(f'有效映射像素占比 {100.0*valid.mean():.1f}%')
    print(f'去畸变位移：最大 {np.nanmax(disp_v):.2f} px  '
          f'平均 {np.nanmean(disp_v):.2f} px')
    dmap = disp_v
    print(f'  中心区 (200:280, 280:360) 平均 '
          f'{np.nanmean(dmap[200:280,280:360]):.2f} px')
    cz = np.concatenate([dmap[:40, :40].ravel(), dmap[:40, -40:].ravel(),
                         dmap[-40:, :40].ravel(), dmap[-40:, -40:].ravel()])
    print(f'  四角 平均 {np.nanmean(cz):.2f} px   最大 {np.nanmax(cz):.2f} px')

    und_img = cv2.undistort(img, K, D, None, K)
    # 并排 + 分界线
    sep = np.full((h, 6, 3), 255, np.uint8)
    both = np.hstack([img, sep, und_img])
    for txt, x0 in (('RAW (distorted)', 10), ('UNDISTORTED', w + 16)):
        cv2.putText(both, txt, (x0, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 0, 0), 4)
        cv2.putText(both, txt, (x0, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 2)

    # 关键验证：**先在畸变图上画网格，再去畸变**。
    # 网格在 RAW 里会是弯的（桶形），去畸变后应变直 —— 这才看得出效果。
    # 反过来（先去畸变再画）两边都是直的，等于什么都没验证。
    grid_raw = img.copy()
    for x in range(0, w, 80):
        cv2.line(grid_raw, (x, 0), (x, h), (0, 255, 255), 1)
    for y in range(0, h, 80):
        cv2.line(grid_raw, (0, y), (w, y), (0, 255, 255), 1)
    grid_und = cv2.undistort(grid_raw, K, D, None, K)
    grid = np.hstack([grid_raw, sep, grid_und])
    for txt, x0 in (('grid on RAW -> stays bowed', 10),
                    ('same grid -> UNDISTORTED', w + 16)):
        cv2.putText(grid, txt, (x0, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 0, 0), 4)
        cv2.putText(grid, txt, (x0, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 2)
    cv2.imwrite('/tmp/undistort_check.png', np.vstack([both, grid]))
    print('\n📄 /tmp/undistort_check.png（上：原图对比；下：网格弯曲→变直）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
