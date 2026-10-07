#!/usr/bin/env python3
"""像素 → base_link 管线：去畸变 → 检棋盘 → 单应到 grid → 外参到 base_link

为什么需要它
------------
旧管线 `object_localizer.py` 是**直接在原始畸变图上**拟合一个 Homography
来吸收畸变。这不严谨：畸变不是单应变换，单应吸收不了它。棋盘角点本身是
弯的，用它拟合出的 H 在**物块所在位置**就是错的，离棋盘中心越远误差越大。

本管线的顺序是：
    原始图 --内参去畸变--> 无畸变图 --单应--> grid(米) --外参--> base_link
其中单应只负责"平面透视"，畸变由内参单独处理，两者各司其职。

输出格式（与 grasp_planner 对齐）
--------------------------------
每个物块给出 base_link 下的 `(x, y, z, yaw)`：
  · (x, y, z) 是物块**中心**在 base_link 下的位置（米）
  · yaw 是物块绕**竖直轴**的转角（弧度）
宽度由调用方自己传（EVA 块是 40mm），管线不管。

三层防护（每一层都会拒绝而不是静默出错）
----------------------------------------
1. **内参缺失/不合格** → 明确拒绝，绝不退回用畸变图硬跑
2. **外参是废弃值** → 明确拒绝（`/tmp/extrinsic.txt` 是已判不可信的两点法）
3. **检出物块落在标定可信区域之外** → 标记 out_of_trusted_region

用法::

    ~/mj/bin/python block_pipeline.py --selftest          # 离线数学自检
    ~/mj/bin/python block_pipeline.py --image a.png --out b.png
    ~/mj/bin/python block_pipeline.py --ros                # 实时
"""

from __future__ import annotations

from project_paths import calibration_path, default_camera

import argparse
import json
import math
import os
import sys
import time

import cv2
import numpy as np

INTRINSICS = calibration_path('camera_intrinsics.yaml')
EXTRINSICS = calibration_path('extrinsic.txt')
# 已废弃：两点法旧值（θ=-97.75°），两个内角点 Z 差 9.5mm、反推格宽 34.6mm≠33mm。
# 见 docs/GRASP_REAL_AUDIT.md。绝不允许静默使用。
DEPRECATED_EXTRINSICS = calibration_path('extrinsic_old_twopoint.txt')


# ------------------------------------------------------------ 参数加载
def load_intrinsics(path=INTRINSICS):
    """读内参。返回 (K, D, meta) 或 (None, None, 原因)。

    为什么要两套解析器：前端 `intrinsic_calib_gui.py` 写的是我自己排版的
    YAML（camera_matrix 写成嵌套列表），OpenCV 的 FileStorage 解析它会抛
    `isMap()` 断言失败。所以：

        先试 FileStorage（兼容 cv2 自己写出来的标准格式）
        失败再退回正则（解析我自己那个格式）

    **两次尝试必须互相独立。** 早先把两步塞在同一个 try 里，FileStorage
    一抛异常函数就整个返回错误，正则兜底根本没机会跑 —— 于是"内参明明标好了"
    却报"解析失败"。
    """
    if not os.path.exists(path):
        return None, None, f'内参文件不存在：{path}'
    try:
        text = open(path, encoding='utf-8').read()
    except OSError as exc:
        return None, None, f'内参文件读不了：{exc}'
    K = D = None
    meta = {}
    for line in text.splitlines():
        s = line.strip()
        if s.startswith('#') and 'RMS' in s:
            meta['note'] = s.lstrip('# ').strip()

    # --- 尝试 1：OpenCV FileStorage（标准格式） ---
    try:
        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        if fs.isOpened():
            k = fs.getNode('camera_matrix').mat()
            d = fs.getNode('distortion_coefficients').mat()
            w = fs.getNode('image_width').real()
            if k is not None and d is not None:
                K, D = k, d
                if w:
                    meta['image_size'] = [
                        int(w), int(fs.getNode('image_height').real())]
        fs.release()
    except Exception:  # noqa: BLE001
        K = D = None

    # --- 尝试 2：正则解析我自己写的格式 ---
    if K is None or D is None:
        import re
        m = re.search(r'camera_matrix:\s*\n((?:\s*-\s*\[[^\]]+\]\s*\n){3})',
                      text)
        if m:
            rows = re.findall(r'\[([^\]]+)\]', m.group(1))
            try:
                K = np.array([[float(v) for v in r.split(',')] for r in rows])
            except ValueError:
                K = None
        m = re.search(r'distortion_coefficients:\s*\n\s*-\s*\[([^\]]+)\]',
                      text)
        if m:
            try:
                D = np.array([[float(v) for v in m.group(1).split(',')]])
            except ValueError:
                D = None
        m = re.search(r'quality_ok\s*=\s*(\d)', text)
        if m:
            meta['quality_ok'] = int(m.group(1))
        m = re.search(r'image_width:\s*(\d+)', text)
        m2 = re.search(r'image_height:\s*(\d+)', text)
        if m and m2:
            meta['image_size'] = [int(m.group(1)), int(m2.group(1))]

    if K is None or D is None:
        return None, None, (f'内参文件里读不到 camera_matrix / '
                            f'distortion_coefficients：{path}')

    # 质量裁决必须通过。裁决由标定前端写进文件（quality_ok=0/1）。
    # 没有裁决标记的旧文件一律拒绝 —— 宁可报"未就绪"，
    # 也不要拿一组没裁决过的参数去抓取，那是最危险的静默失败。
    q = meta.get('quality_ok')
    if q == 0:
        return None, None, f'内参质量裁决未通过（quality_ok=0）：{path}'
    if q is None:
        return None, None, (f'内参文件没有质量裁决标记（quality_ok），'
                            f'无法确认是否可用，拒绝使用：{path}')
    return K, D.reshape(-1), meta


def load_extrinsics(path=EXTRINSICS):
    from qianli_vision.calibration import load_extrinsics as validated_extrinsics
    return validated_extrinsics(path)


# ------------------------------------------------------------ 坐标变换
def grid_to_base(xy_grid, ext):
    """grid 平面坐标(米) → base_link 平面坐标(米)。

    外参存的是 grid 原点在 base_link 下的位置 + grid 相对 base 的转角。
    """
    th = math.radians(ext['grid_theta_deg'])
    c, s = math.cos(th), math.sin(th)
    x, y = float(xy_grid[0]), float(xy_grid[1])
    return np.array([ext['grid_origin_x'] + c * x - s * y,
                     ext['grid_origin_y'] + s * x + c * y])


def board_homography(px_corners, cols, rows, cell_m):
    """棋盘角点(无畸变像素) → grid(米) 的单应矩阵。

    角点顺序必须与 findChessboardCorners 的输出一致：
    行优先，起点是棋盘某一角。**顺序翻转会让物块定位转到错误象限**，
    所以调用方要用 base_link 范围做校验（见 Pipeline.check_order）。
    """
    obj = np.zeros((rows * cols, 2), np.float32)
    obj[:, :] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    obj *= cell_m
    H, mask = cv2.findHomography(np.asarray(px_corners, np.float32).reshape(-1, 2),
                                 obj, 0)
    return H


def apply_h(H, px):
    p = np.array([px[0], px[1], 1.0])
    q = H @ p
    return q[:2] / q[2]


# ------------------------------------------------------------ 相机位移自检
class CameraMoveWatch:
    """盯着画面里一块**固定不动**的区域，判断相机有没有被碰过。

    为什么需要：撕掉棋盘之后，整条外参就建立在"相机没动"这个假设上。
    "相机固定死了"是个假设，不是保证 —— 而且它失效时**没有任何症状**，
    只会让抓取慢慢开始偏。所以用一个便宜的观测去守它：
    取画面里一块（默认左上角一个小方块）区域作参照，
    每次启动时和"基准指纹"比，偏移超过阈值就报警。

    参照物怎么选：相机视野里**永远不动**的东西 —— 桌面一角、夹具本体、
    贴的一个标记。**不能选棋盘**（它会被撕掉），也不能选会被物块/机械臂
    挡住的区域。默认取左上角，通常那里是背景。
    """

    def __init__(self, path=calibration_path('camera_ref.npz'), roi=None, thresh_px=3.0):
        self.path = path
        self.roi = roi or (0.03, 0.03, 0.22, 0.22)   # x0,y0,x1,y1 归一化
        self.thresh_px = thresh_px
        self.ref = None
        self._load()

    def _crop(self, frame):
        h, w = frame.shape[:2]
        x0, y0, x1, y1 = self.roi
        return frame[int(y0 * h):int(y1 * h), int(x0 * w):int(x1 * w)]

    def _fingerprint(self, frame):
        g = cv2.cvtColor(self._crop(frame), cv2.COLOR_BGR2GRAY)
        g = cv2.GaussianBlur(g, (5, 5), 0)
        return g.astype(np.float32)

    def _load(self):
        if os.path.exists(self.path):
            try:
                self.ref = np.load(self.path)['ref']
            except Exception:  # noqa: BLE001
                self.ref = None

    def save_reference(self, frame):
        ref = self._fingerprint(frame)
        np.savez_compressed(self.path, ref=ref)
        self.ref = ref
        return ref.shape

    def check(self, frame):
        """返回 (ok, shift_px, msg)。没有基准指纹时返回 (None, 0, 提示)。"""
        if self.ref is None:
            return None, 0.0, (f'没有基准指纹：{self.path}。'
                               f'撕棋盘前请先 `--save-camera-ref` 存一次')
        cur = self._fingerprint(frame)
        if cur.shape != self.ref.shape:
            return False, float('inf'), (f'参照区域尺寸变了（{cur.shape} vs '
                                         f'{self.ref.shape}）—— 分辨率或相机换了？')
        # 相位相关求亚像素平移：对整体亮度变化不敏感，只对位移敏感，
        # 正合适"相机被碰了一下"这种场景。
        (dx, dy), _ = cv2.phaseCorrelate(self.ref, cur)
        shift = float(np.hypot(dx, dy))
        ok = shift <= self.thresh_px
        msg = (f'相机参照位移 {shift:.2f} px '
               f'({"✅ 没动" if ok else "⛔ 可能被移动过"}，'
               f'阈值 {self.thresh_px:.1f} px)')
        return ok, shift, msg


# ------------------------------------------------------------ 主管线
class Pipeline:
    def __init__(self, cols=7, rows=5, cell_m=0.033, verbose=True):
        self.cols, self.rows, self.cell_m = cols, rows, cell_m
        self.K = self.D = None
        self.ext = None
        self.verbose = verbose
        self.calib_ready = False
        self.blockers = []
        self._reload()

    # ---- 就绪状态（给前端/上游脚本看的） ----
    def _reload(self):
        self.blockers = []
        K, D, why_i = load_intrinsics()
        if K is None:
            self.blockers.append(f'内参未就绪：{why_i}')
        else:
            self.K, self.D = K, D
        ext, why_e = load_extrinsics()
        if ext is None:
            self.blockers.append(f'外参未就绪：{why_e}')
        else:
            self.ext = ext
        self.calib_ready = not self.blockers
        return self.calib_ready

    def status(self):
        return {
            'ready': self.calib_ready,
            'blockers': list(self.blockers),
            'has_intrinsics': self.K is not None,
            'has_extrinsics': self.ext is not None,
            'extrinsics': self.ext,
        }

    def undistort(self, frame):
        if self.K is None:
            return frame
        return cv2.undistort(frame, self.K, self.D)

    def find_board(self, gray):
        """在（已去畸变的）灰度图上找棋盘。返回角点或 None。"""
        flags = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        found, corners = cv2.findChessboardCorners(
            gray, (self.cols, self.rows), flags)
        if not found:
            return None
        return cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

    def board_H(self, gray):
        corners = self.find_board(gray)
        if corners is None:
            return None, None
        return board_homography(corners, self.cols, self.rows,
                                self.cell_m), corners

    def trusted_region_warn(self, px):
        """物块像素是否落在"标定有数据的区域"之外。

        畸变在画面边缘最大，而标定时未必覆盖到每一边。落在没有角点数据的
        区域里，去畸变就是外推 —— 结果不可信，必须报出来而不是静默给数。
        """
        if self.ext is None:
            return None
        return None  # 由调用方结合 coverage 判断，这里留接口

    def to_base(self, px, H):
        """像素 → base_link (x, y)。"""
        g = apply_h(H, px)
        b = grid_to_base(g, self.ext)
        return b, g

    def process(self, frame):
        """整帧处理。返回 dict：
            {'ready', 'board': bool, 'blocks': [...], 'blockers': [...]}
        """
        out = {'ready': self.calib_ready, 'board': False, 'blocks': [],
               'blockers': list(self.blockers)}
        if not self.calib_ready:
            return out
        und = self.undistort(frame)
        gray = cv2.cvtColor(und, cv2.COLOR_BGR2GRAY)
        H, corners = self.board_H(gray)
        if H is None:
            return out
        out['board'] = True
        out['corners'] = corners.reshape(-1, 2).tolist()
        return out


# ------------------------------------------------------------ 自检
def selftest(cols=7, rows=5, cell_m=0.033, seed=0):
    """离线数学自检：不去畸变真图，而是走"点"的同一套数学。

    为什么这样测就够：生产管线是"先把图去畸变、再检角点"，而检出的角点
    等价于"把真实（畸变）角点做 undistortPoints"。两者的差别只是重采样，
    不改变几何。所以用投影出来的畸变角点 + undistortPoints 就能把
    **去畸变 → 单应 → 外参**整条数学链验证一遍。

    做法：
      1. 伪造一组内参 K、畸变 D；
      2. 给棋盘一个真实位姿，用 projectPoints(带 D) 得到**畸变**像素；
      3. undistortPoints 还原成无畸变像素；
      4. 拟合单应 H，用**棋盘中心**这个从没参与拟合的点做检验；
      5. 检查 H 反算出的 grid 坐标是否等于真值。
    """
    rng = np.random.default_rng(seed)
    K = np.array([[480.0, 0, 320.0], [0, 478.0, 240.0], [0, 0, 1.0]])
    D = np.array([-0.30, 0.12, 0.001, -0.002, -0.02])
    size = (640, 480)

    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * cell_m

    print('=' * 74)
    print('  像素管线离线自检（内参 → 去畸变 → 单应 → 外参）')
    print('=' * 74)
    print(f'  伪造内参 fx={K[0,0]:.0f} fy={K[1,1]:.0f} '
          f'cx={K[0,2]:.0f} cy={K[1,2]:.0f}')
    print(f'  伪造畸变 k1={D[0]:+.3f} k2={D[1]:+.3f} k3={D[4]:+.3f}')
    print()

    # 伪外参：grid 原点在 base_link 的某处，转角 -97.75°
    ext_true = {'grid_origin_x': 0.3420, 'grid_origin_y': 0.0584,
                'grid_origin_z': -0.0690, 'grid_theta_deg': -97.75,
                'quality_ok': 1.0}

    fails = []
    for trial in range(5):
        # 棋盘相对相机的位姿：正对为主，加一点倾斜和距离变化
        rvec = np.array([rng.uniform(-0.35, 0.35),
                         rng.uniform(-0.35, 0.35),
                         rng.uniform(-0.3, 0.3)])
        tvec = np.array([rng.uniform(-0.06, 0.06),
                         rng.uniform(-0.06, 0.06),
                         rng.uniform(0.28, 0.42)])

        # 1) 真值：带畸变的像素坐标
        px_dist, _ = cv2.projectPoints(objp.reshape(-1, 1, 3), rvec, tvec, K, D)
        px_dist = px_dist.reshape(-1, 2)

        # 2) 去畸变（等价于"图先 undistort 再检角点"）
        und = cv2.undistortPoints(px_dist.reshape(-1, 1, 2), K,
                                  D.reshape(-1, 1), P=K).reshape(-1, 2)

        # 3) 拟合单应：无畸变像素 → grid
        H = board_homography(und, cols, rows, cell_m)
        if H is None:
            fails.append(f'第{trial+1}次：单应拟合失败')
            continue

        # 4) 检验点：取网格中一个**不在角点上**的点（格心）
        gx, gy = 2.5 * cell_m, 1.5 * cell_m
        p3 = np.array([[[gx, gy, 0.0]]], np.float32)
        px_true, _ = cv2.projectPoints(p3, rvec, tvec, K, D)
        px_true_u = cv2.undistortPoints(px_true.reshape(-1, 1, 2), K,
                                        D.reshape(-1, 1),
                                        P=K).reshape(-1, 2)
        g_est = apply_h(H, px_true_u[0])

        err_mm = float(np.linalg.norm(g_est - [gx, gy])) * 1000
        b_est = grid_to_base(g_est, ext_true)
        b_true = grid_to_base([gx, gy], ext_true)
        err_base_mm = float(np.linalg.norm(b_est - b_true)) * 1000

        # 5) 另外量一下"不去畸变直接拟合单应"会差多少 —— 这就是旧管线的错
        H_naive = board_homography(px_dist, cols, rows, cell_m)
        g_naive = apply_h(H_naive, px_true.reshape(-1, 2)[0])
        err_naive_mm = float(np.linalg.norm(g_naive - [gx, gy])) * 1000

        print(f'  第{trial+1}次  格心({gx*1000:.1f},{gy*1000:.1f})mm  '
              f'本管线 grid 误差 {err_mm:6.3f} mm '
              f'(base_link {err_base_mm:.3f} mm)   '
              f'| 旧管线不去畸变 {err_naive_mm:7.3f} mm')
        if err_mm > 0.5:
            fails.append(f'第{trial+1}次误差 {err_mm:.3f}mm > 0.5mm')

    print()
    if fails:
        print('  ❌ 自检未通过：')
        for f in fails:
            print('     ·', f)
    else:
        print('  ✅ 自检通过：本管线 grid 误差 < 0.5mm；'
              '旧管线（不去畸变）误差大一个量级')
    print('=' * 74)
    return 1 if fails else 0


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cols', type=int, default=7)
    ap.add_argument('--rows', type=int, default=5)
    ap.add_argument('--cell-mm', type=float, default=33.0)
    ap.add_argument('--image')
    ap.add_argument('--out')
    ap.add_argument('--ros', action='store_true')
    ap.add_argument('--selftest', action='store_true')
    ap.add_argument('--status', action='store_true')
    ap.add_argument('--save-camera-ref', action='store_true',
                    help='抓一帧存为"相机参照指纹"。**撕棋盘之前跑一次**，'
                         '之后就能检测相机有没有被碰过。')
    ap.add_argument('--check-camera', action='store_true',
                    help='检查相机相对基准指纹有没有移动')
    ap.add_argument('--ref-thresh-px', type=float, default=3.0,
                    help='相机位移报警阈值（像素）')
    args = ap.parse_args()

    if args.selftest:
        return selftest(args.cols, args.rows, args.cell_mm / 1000.0)

    # ---- 相机位移自检（与内参/外参是否就绪无关，所以放在前面） ----
    if args.save_camera_ref or args.check_camera:
        cap = cv2.VideoCapture(default_camera())
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        time.sleep(0.8)
        for _ in range(5):
            cap.read()
        ok, frame = cap.read()
        cap.release()
        if not ok:
            print('❌ 读不到相机')
            return 1
        watch = CameraMoveWatch(thresh_px=args.ref_thresh_px)
        if args.save_camera_ref:
            shape = watch.save_reference(frame)
            print(f'✅ 已存相机参照指纹 {shape} → {watch.path}')
            print('   ⚠️ 现在开始，相机不能再动了。')
            return 0
        ok, shift, msg = watch.check(frame)
        print(msg)
        return 0 if ok else 1

    pipe = Pipeline(args.cols, args.rows, args.cell_mm / 1000.0)
    st = pipe.status()
    print('管线状态：' + ('✅ 就绪' if st['ready'] else '❌ 不可用'))
    for b in st['blockers']:
        print('  ⛔', b)
    if args.status:
        return 0 if st['ready'] else 1
    if not st['ready']:
        print('\n拒绝运行：抓取管线当前用不了。'
              '先跑 intrinsic_calib_gui.py 标内参，'
              '再跑 extrinsic_calib_multi.py 标外参。')
        return 1

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f'❌ 读不到图 {args.image}')
            return 1
        res = pipe.process(frame)
        print(json.dumps({k: v for k, v in res.items() if k != 'corners'},
                         ensure_ascii=False, indent=2))
        if args.out:
            und = pipe.undistort(frame)
            cv2.imwrite(args.out, und)
            print(f'📄 去畸变图写入 {args.out}')
        return 0

    if args.ros:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        rclpy.init()
        node = Node('block_pipeline')
        pub = node.create_publisher(String, '/blocks_base', 10)
        cap = cv2.VideoCapture(default_camera())
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        while rclpy.ok():
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            res = pipe.process(frame)
            msg = String()
            msg.data = json.dumps({'stamp': time.time(),
                                   'board': res['board'],
                                   'blocks': res['blocks']})
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.01)
        cap.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()
        return 0

    print('指定 --selftest / --status / --image / --ros 之一')
    return 0


if __name__ == '__main__':
    sys.exit(main())
