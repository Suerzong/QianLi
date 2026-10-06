#!/usr/bin/env python3
"""color_block_detect —— EVA 彩色方块识别（纯视觉，不碰机械臂）

解决什么问题
------------
SO-101 抓取作业里，桌面上散着一批 4cm 边长的 EVA 泡棉方块（红/黄/绿/紫…）。
抓取前必须知道"哪一块、什么颜色、在画面的哪个位置、转了多少度"。
本模块只做这件事：从一帧图像里找出所有彩色方块，输出质心/颜色/朝向/边长/置信度。
**它不发布任何关节指令、不打开串口、不碰任何硬件话题**——只往 /blocks 发 JSON 字符串。

为什么这么做（算法取舍）
------------------------
1. 颜色判定走 Lab 的 (a, b) 色度方向，而不是 RGB 或"写死的 HSV 数值"。
   EVA 泡棉哑光、高饱和、无镜面高光，它的**颜色种类**（色相/色度方向）在
   不同亮度、不同白平衡下几乎不变；变的只是"有多亮、多饱和"（明度 L 与色度模长）。
   把 (a, b) 归一化成单位方向再取角，等于把"亮度变化"这一维从判据里彻底扔掉，
   所以日光灯闪烁、阴影、手机补光、相机自动曝光漂移都不会改判颜色。
   相比之下 RGB 判据必须同时卡三个通道的绝对值，一暗就全废；
   写死 HSV 的 S/V 区间也会因为整体曝光变化而失效（H 本身其实很稳，这是我们的主判据，
   所以取色相时用的是 OpenCV 的 H 通道 —— 它和大白平衡/曝光无关）。
2. 前景检测不写死色度阈值，而是让它跟着当前场景自适应：
   阈值 = max(--chroma-min, 场景色度 99.5 百分位 × 0.45)，并封顶在 2.5 倍下限。
   为什么不用 cv2.THRESH_OTSU：EVA 场景的色度直方图是"一大坨 0（灰桌白纸）
   + 一坨高值（彩色料）"的极端双峰，OpenCV 的 Otsu 碰到这种分布会直接返回 0，
   于是 inRange(0,255) 变成"全图都是前景"（这个坑实测踩过）。
   百分位法同样"看这一帧自己"，换灯光/换背景布不用重调参数，但不会退化成 0。
3. 形状过滤用"面积 + 长宽比 + 最小外接矩形填充率 + 凸度 + approxPolyDP 顶点数"
   一起卡，专门滤掉灯管高光条、投影、线缆这类细长/不规则的东西。
   注意这里**不能用"面积/轴对齐包围盒"**（常见的 extent）：正方形斜放 45° 时
   它只有 0.5，会把所有转着放的方块误杀。用跟着物体一起转的最小外接矩形填充率，
   这个指标才是旋转不变的。
4. 相机内参没有时不做去畸变，只提示，继续用原始图跑——
   标定是渐进过程，识别不该因为标定文件还没生成就不能用。

怎么用
------
    # 离线（最重要：没有相机也能测）
    ~/mj/bin/python color_block_detect.py --image test.png --out annotated.png
    ~/mj/bin/python color_block_detect.py --image test.png --out a.png --json-out r.json

    # 自检：合成图 + 断言，不需要相机、不需要 ROS、不需要标定文件
    ~/mj/bin/python color_block_detect.py --selftest

    # ROS 2：开 /dev/video0 持续检测，JSON 发到 /blocks (std_msgs/String)
    ~/mj/bin/python color_block_detect.py --ros --debug-image
    ros2 topic echo /blocks

依赖：只用到 numpy + opencv（ROS 模式下才 import rclpy）。
内参文件默认读 /tmp/camera_intrinsics.yaml，存在就自动去畸变，不存在就提示并继续。
"""

import argparse
import json
import math
import os
import re
import signal
import sys
import time

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# 常量与默认值
# ---------------------------------------------------------------------------

DEFAULT_INTRINSICS = '/tmp/camera_intrinsics.yaml'
DEFAULT_DEBUG_IMAGE = '/tmp/color_blocks_debug.jpg'

# OpenCV 的 H 是 0~179（真实色相角度的一半）：红≈0、黄≈30、绿≈60、青≈90、紫≈150。
# 这些值对应 EVA 泡棉常见的饱和色，但**不要把它当唯一真理**：
# 用 --colors 覆盖（例如 "red:0,yellow:28"），或看自检/A 段的"色相闭环"输出来校正。
DEFAULT_HUE_CENTERS = {
    'red': 0,
    'yellow': 30,
    'green': 60,
    'yellow2': 38,   # "第五块可能是第二种黄"——单独留一档，默认不作为标签上报
    'purple': 150,
}

# 默认颜色标签表：标签 -> 参考色相。做成参数就是为了不把颜色写死，
# 换一批料（比如换成青色/橙色）只改这张表，不用动代码。
DEFAULT_COLOR_TABLE = ('red:0', 'yellow:30', 'green:60', 'purple:150')

# 判定为某颜色的最大色相偏差。EVA 是哑光高饱和料，同一块料在不同光照下
# 色相漂移通常 < 5 个 H 单位（=10°真实色角），8 已经相当宽松；
# 实机若出现"绿判成黄"，先量测再调，不要瞎改。
DEFAULT_HUE_TOL = 8.0

# Otsu 之外的色度绝对下限（Lab 色度模长，a/b 都是 [-127,127] 量级）。
# 灰纸/白桌面/金属的色度模长通常 < 8，实测彩色 EVA 在 30 以上，
# 取 26 是为了让"几乎无彩色的场景"（例如桌上只有灰块）不产生噪声掩码。
DEFAULT_CHROMA_MIN = 26.0

# approxPolyDP 顶点数区间。正方形近似后是 4；圆角/像素化方块偶尔会多出 1~2 个点，
# 所以上限给到 6；要喂圆形物体（瓶盖）就放到 8~10。
VERTEX_MIN, VERTEX_MAX = 4, 6

# 判断"某色块是否够方"时，用 OpenCV 默认像素角点假设；这里只影响极小的修正。
_POLY_EPS_RATIO = 0.03


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------

def log(msg, level='INFO'):
    """统一日志：保持朴素可 grep 的格式（ROS 自带的日志格式反而不好复制粘贴）。"""
    print(f'[{level}] {msg}', flush=True)


def ang_diff_deg(a, b):
    """两个色相角（0~179 半角单位）的最小差值绝对值，结果落在 [0, 90]。

    存在意义：色相 0 和 179 其实相邻（红绕回来了）。
    直接做减法取绝对值的话，红会在 0/180 边界上被劈成两半。
    """
    d = abs((float(a) - float(b)) % 180.0)
    return min(d, 180.0 - d)


def wrap_half_open(angle):
    """把朝向角折叠到 [-45, 45)。

    方块是正方形，转 90° 后外观完全一样，所以原始角度天生有 90° 二义性；
    折叠到半开区间后，同一个物理朝向在连续帧里不会在 ±45 之间来回跳。
    """
    return (float(angle) + 45.0) % 90.0 - 45.0


def open_camera(index, width, height):
    """打开 USB 摄像头。失败返回 None（绝不抛异常，方便 ROS 主循环重试）。"""
    if os.name == 'nt':
        cap = cv2.VideoCapture(index)
    else:
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not cap.isOpened():
            # 有些环境 V4L2 后端不可用（容器/虚拟相机），退回默认后端再试一次
            cap.release()
            cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        cap.release()
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # 宁可丢帧也不要处理 0.3 秒前的旧画面
    except Exception:
        pass
    return cap


# ---------------------------------------------------------------------------
# 相机内参：极简 YAML 读取 + 去畸变
# ---------------------------------------------------------------------------

def _numbers(text):
    """抠出一行里的所有浮点数（含科学计数法）。"""
    return [float(x) for x in
            re.findall(r'[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?', text)]


def load_intrinsics(path):
    """读内参 YAML，返回 (K, dist, (w, h))；解析失败返回 None。

    为什么不用 yaml 库：环境里不一定有 PyYAML，而这份文件的结构是固定的
    （image_width/image_height/camera_matrix/distortion_coefficients），
    几十行就够，还顺便兼容 OpenCV FileStorage 那种
    'distortion_coefficients: !!opencv-matrix' + rows/cols/dt/data 的写法。

    解析策略是"按段收数字，而不是按行当记录"：
    同一个段里的数字不管是写在一行、拆成多行、还是 YAML 列表项，
    都原样追加到一个列表里，最后按数学含义重新整形。
    这么做是因为"逐行解析"在这里有个无解的歧义：
    YAML 列表项以 '-' 开头，而负数也以 '-' 开头，
    于是 "[0.1, 0.2,\\n  -0.3, -0.4]" 里的 -0.3 会被当成新的一行数据，
    把数组截断成一半——而且不会报错，只会静默地少读几个系数、错读整张内参矩阵。
    """
    if not path or not os.path.isfile(path):
        return None
    try:
        # utf-8-sig：Windows 上编辑器/脚本很容易给文件加 UTF-8 BOM，
        # 而 BOM 会让第一行 "image_width: 640" 的正则匹配失败
        # （BOM 不是字母），于是宽高解析不出来、分辨率一致性检查静默失效。
        with open(path, 'r', encoding='utf-8-sig', errors='replace') as fh:
            lines = fh.readlines()
    except OSError as exc:
        log(f'内参文件打不开（{exc}），改用原始图继续', 'WARN')
        return None

    width = height = None
    rows = {'camera_matrix': [], 'distortion_coefficients': []}
    section = None        # 当前正在收集哪一段
    open_key = None       # 段内某个 "key: [ ..." 还没闭合，后续行继续算这个段的数字
    for raw in lines:
        line = raw.split('#', 1)[0].strip()      # 标定脚本会在文件头写注释
        if not line:
            continue
        m = re.match(r'image_width\s*:\s*(\d+)', line)
        if m:
            width = int(m.group(1))
            section = open_key = None
            continue
        m = re.match(r'image_height\s*:\s*(\d+)', line)
        if m:
            height = int(m.group(1))
            section = open_key = None
            continue
        m = re.match(r'([A-Za-z_]+)\s*:\s*(.*)$', line)
        if m:
            key, rest = m.group(1), m.group(2).strip()
            if key in rows:
                section, open_key = key, None
                if rest:
                    vals = _numbers(rest)
                    if vals:
                        rows[section].extend(vals)
                    # 数组没闭合（'[' 比 ']' 多）就继续吃后续行
                    if rest.count('[') > rest.count(']'):
                        open_key = section
                continue
            # camera_matrix 前面那层是 3x3，必须知道自己收了几个数才能整形；
            # 记下"这一段已经收了多少个数字"就够了。
            if key == 'data' and section:
                rows[section].extend(_numbers(rest))
                open_key = section if rest.count('[') > rest.count(']') else None
                continue
            # rows/cols/dt 的数字（3、1、d）只是元信息，一旦混进矩阵就会错位
            if key in ('rows', 'cols', 'dt'):
                continue
            section = open_key = None
            continue
        if section is None:
            continue
        # 没有 key 的行：列表项 "- 1.0" 或跨行数组的续行，两种都只是"这一段的数字"
        rows[section].extend(_numbers(line))
        if ']' in line:
            open_key = None

    vals = rows['camera_matrix']
    if len(vals) < 9:
        log(f'内参文件 {path} 里的 camera_matrix 数字不足 9 个（实得 '
            f'{len(vals)}），忽略', 'WARN')
        return None
    K = np.array(vals[:9], np.float64).reshape(3, 3)

    dvals = rows['distortion_coefficients']
    if len(dvals) >= 4:
        dist = np.array(dvals[:5], np.float64).reshape(1, -1)
    else:
        # 少于 4 个系数说明文件被截断/写坏了。宁可按"无畸变"处理，
        # 也不要拿一堆 0 去假装标定成功——那会让画面默默错位。
        if dvals:
            log(f'内参文件 {path} 的畸变系数只有 {len(dvals)} 个，按无畸变处理', 'WARN')
        dist = np.zeros((1, 5), np.float64)
    return K, dist, (width, height)


class Undistorter:
    """把"每帧去畸变"封装成一次预计算 + 一次 remap。

    为什么要预计算：initUndistortRectifyMap 每帧算一遍约 10~20ms，
    10Hz 下白白吃掉 20% CPU；映射表只跟内参和分辨率有关，算一次就够。
    """

    def __init__(self, yaml_path, log_fn=log):
        self.path = yaml_path
        self.enabled = False
        self.K = None
        self.dist = None
        self.size = None
        self._maps = None
        self._map_size = None
        self.notes = []
        self._log = log_fn
        if not yaml_path:
            self.notes.append('未指定内参文件，使用原始图（未去畸变）')
        elif not os.path.isfile(yaml_path):
            self.notes.append(
                f'未找到内参文件 {yaml_path}：使用原始图继续。'
                '等标定完成后重跑本脚本即可自动启用去畸变')
        else:
            loaded = load_intrinsics(yaml_path)
            if loaded is None:
                self.notes.append(f'{yaml_path} 解析失败：使用原始图继续')
            else:
                self.K, self.dist, self.size = loaded
                self.enabled = True
                w, h = self.size
                self.notes.append(f'已加载内参 {yaml_path}'
                                  + (f'（标定分辨率 {w}x{h}）' if w and h else ''))

    def apply(self, frame):
        """返回去畸变后的图；没内参时原样返回（不崩，也不重复提示）。"""
        if not self.enabled:
            return frame
        h, w = frame.shape[:2]
        if self._maps is None or self._map_size != (w, h):
            self._maps = cv2.initUndistortRectifyMap(
                self.K, self.dist, None, self.K, (w, h), cv2.CV_16SC2)
            self._map_size = (w, h)
            cw, ch = self.size if self.size else (None, None)
            if cw and ch and (cw, ch) != (w, h):
                # 映射表能按当前分辨率生成，但畸变系数是按标定分辨率拟合的，
                # 尺度对不上会不准——只警告，不阻断（分辨率小改通常仍可用）。
                self._log(f'注意：内参标定于 {cw}x{ch}，当前帧 {w}x{h}，'
                          '去畸变可能不准（建议用标定时的分辨率）', 'WARN')
        return cv2.remap(frame, self._maps[0], self._maps[1], cv2.INTER_LINEAR)


# ---------------------------------------------------------------------------
# 颜色参考表
# ---------------------------------------------------------------------------

def parse_color_table(spec, hue_centers):
    """把 "red:0,yellow:30" 解析成 [(标签, 参考色相), ...]。

    - 只给标签（"red"）时用内置色相中心；
    - 标签后加 '+'（"yellow:30+"）表示允许重复标签（两块黄的场景用得上）；
      默认重名会被跳过并在日志里说明——避免"我配了两个颜色"其实只生效一个。
    """
    out = []
    if not spec:
        return out
    for item in str(spec).split(','):
        item = item.strip()
        if not item:
            continue
        allow_dup = item.endswith('+')
        item = item.rstrip('+').strip()
        if ':' in item:
            label, _, value = item.partition(':')
            label = label.strip()
            try:
                center = float(value)
            except ValueError:
                log(f'颜色表项 "{item}" 的色相不是数字，跳过', 'WARN')
                continue
        else:
            label = item
            if label not in hue_centers:
                log(f'颜色表里的 "{label}" 没有已知参考色相，跳过', 'WARN')
                continue
            center = float(hue_centers[label])
        if not label:
            continue
        if not allow_dup and any(lbl == label for lbl, _ in out):
            log(f'颜色表里 "{label}" 重复，保留第一个'
                f'（确实需要两个同色档请写 "{label}:{center:g}+"）', 'WARN')
            continue
        out.append((label, center % 180.0))
    return out


def mean_circular_hue(hue_values):
    """角度量的"平均"必须按矢量平均，不能直接取算术平均。

    直接 mean() 的经典翻车：几个 178 和几个 2（都是红）会平均成 90（青）。
    这里把每个角度映射成单位圆上的 (cos, sin) 再求和，结果对 0/180 环绕天然免疫。
    返回 (均值角度, 集中度 R, 样本数)：R = 合矢量长度/样本数 ∈ [0,1]，
    R 越接近 1 说明这一片的色相越一致（越像一块纯色料）。
    """
    values = np.asarray(hue_values, np.float64)
    n = int(values.size)
    if n == 0:
        return 0.0, 0.0, 0
    rad = values * (math.pi / 90.0)          # H 是半角，乘 2 才是真实色相角
    vs = float(np.sin(rad).mean())
    vc = float(np.cos(rad).mean())
    mean_h = (math.degrees(math.atan2(vs, vc)) * 0.5) % 180.0
    return mean_h, math.hypot(vs, vc), n


# ---------------------------------------------------------------------------
# 检测
# ---------------------------------------------------------------------------

def default_thresholds(shape):
    """按画面尺寸给出轮廓面积的默认区间。

    为什么不写死像素数：换分辨率（320x240 / 1280x720）后那些数字全废。
    4cm 方块在 640x480、桌面俯视的典型距离下约占 35~90 px 边长
    （对照 object_localizer 里边长 4cm 对应的 min_size/max_size = 15/60），
    这里用"画面面积的 0.04%~12%"作为等价面积区间，折算到 640x480 是
    123~36864 px²（约 11~192 px 边长），足够宽以兼容远近差异。
    注意：**没有**用 HSV 饱和度做面积门限，面积只做粗筛，细节交给形状指标。
    """
    h, w = shape[:2]
    pixels = float(h * w)
    return max(60.0, 0.0004 * pixels), max(400.0, 0.12 * pixels)


def _shape_metrics(contour):
    """一次算完所有形状指标，避免后面重复遍历轮廓点。

    'vertex_count' 是 approxPolyDP 的结果，用来判断"像不像四边形"。
    eps 取周长的 3% 是 OpenCV 文档里对规则多边形常用的经验值：
    太小会把像素锯齿当成顶点（正方形变 8 边形），太大则会把直角抹掉（变 3 边形）。
    两个方向的误判都会让真块被 vertex 过滤挡掉，所以下面还有一道"反查"兜底。
    """
    area = float(cv2.contourArea(contour))
    x, y, bw, bh = cv2.boundingRect(contour)
    hull = cv2.convexHull(contour)
    hull_area = float(cv2.contourArea(hull))
    peri = float(cv2.arcLength(contour, True))
    approx = cv2.approxPolyDP(contour, _POLY_EPS_RATIO * peri, True)
    rect = cv2.minAreaRect(contour)          # ((cx,cy),(w,h),angle)
    rect_w, rect_h = float(rect[1][0]), float(rect[1][1])

    metrics = {
        'area': area,
        'bbox': (x, y, bw, bh),
        'rect': rect,
        'rect_w': rect_w,
        'rect_h': rect_h,
        'solidity': area / hull_area if hull_area > 1e-6 else 0.0,
        # extent 只是诊断量，**绝不能**当形状判据：它等于"面积/轴对齐包围盒"，
        # 而对角线摆放的正方形这个值只有 0.5（45° 时最小）。用它过滤会误杀
        # 所有斜放的方块——本项目真的踩过这个坑。
        'extent': area / float(bw * bh) if bw > 0 and bh > 0 else 0.0,
        # fill：面积 / 最小外接矩形面积。最小外接矩形跟着物体一起旋转，所以
        # 这个比值对**旋转不敏感**：正方形 0.95~1.0，圆/椭圆 ≈0.785（π/4）。
        # 注意它单独也不够——细长条填充率同样接近 1——必须和长宽比、顶点数合用。
        'fill': area / (rect_w * rect_h) if rect_w * rect_h > 1e-6 else 0.0,
        'aspect': max(bw, bh) / float(min(bw, bh)) if min(bw, bh) > 0 else 99.0,
        'vertex_count': int(len(approx)),
    }
    # 反查兜底：approxPolyDP 顶点数不在合理区间时，逐点量"原轮廓点是否落在
    # 近似多边形之外"。只要没有点跑到外面（max_dev <= 0.5），说明多边形是
    # 一个包裹住轮廓的合法凸包，顶点多一个少一个不算问题，不该误杀真块。
    if not (VERTEX_MIN <= metrics['vertex_count'] <= VERTEX_MAX) and len(approx) >= 3:
        dev = 0.0
        for p in contour.reshape(-1, 2).astype(np.float32):
            d = cv2.pointPolygonTest(approx, (float(p[0]), float(p[1])), True)
            if d < 0:                      # 只在轮廓点跑到多边形外时才关心
                dev = max(dev, -d)
        metrics['poly_outside_dev'] = float(dev)

    # 细长条（灯管反光、线缆）的一项快速否决：面积相对包围盒极小
    return metrics


def detect_blocks(image, args, color_table, hue_centers, verbose=False):
    """从一帧（已去畸变的）BGR 图里找出所有彩色方块。

    处理链：色度模长 → Otsu 自动阈值 → 形态学 → 外轮廓 → 形状过滤
            → 色相矢量平均 → 最近参考色 + 置信度。

    返回 (blocks, mask, info)。blocks 的字段见下面的字典构造，
    其中 contour/extent/solidity/mean_hue 等是给调试叠加和调参用的诊断量，
    真正发到 /blocks 的只有 round_block() 挑出来的那六个。
    """
    h, w = image.shape[:2]
    area_min, area_max = args.area_min, args.area_max
    if area_min is None or area_max is None:
        d_min, d_max = default_thresholds(image.shape)
        area_min = d_min if area_min is None else area_min
        area_max = d_max if area_max is None else area_max

    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue = hsv[:, :, 0]

    # 色度模长 |(a-128, b-128)|：颜色越艳值越大，而它**与明暗无关**
    # （Lab 里 a/b 是色差轴，L 才管亮度），所以"同一块料在暗处/亮处"
    # 在这里得到几乎同一个值——这正是我们要的抗光照性质。
    a = lab[:, :, 1].astype(np.float32) - 128.0
    b = lab[:, :, 2].astype(np.float32) - 128.0
    chroma = cv2.magnitude(a, b)

    # 自适应色度阈值。为什么不用 cv2.THRESH_OTSU：
    # EVA 场景的色度直方图是"一大坨 0（灰桌/白纸，色度≈0）+ 一坨高值（彩色料）"
    # 的极端双峰，OpenCV 的 Otsu 实现碰到这种分布会直接返回 0，
    # 于是 inRange(0,255) 变成"全图都是前景"，噪声、阴影全进掩码。
    # 这里改用对分布形状不敏感的百分位法：
    #   阈值 = floor ~ 2.5*floor 之间，取场景色度 99.5 百分位的 45%。
    # 这么做的道理和 Otsu 一样（让阈值随当前场景的彩色强度缩放，换灯光不用重调），
    # 但不会在"几乎没有彩色"时把阈值定成 0。
    #   系数 0.45 ← 实测彩色料内部色度远高于边缘过渡带，0.45 倍高分位正好落在
    #               "料本体"和"抗锯齿边缘"之间；
    #   上限 2.5*floor ← 防止画面里出现大片高饱和背景（海报/彩布）时
    #                    阈值被抬到把真块也滤掉。
    # 现场如果发现"掩码里有料但没被检出"，先看调试图右边那张掩码，
    # 再调 --chroma-min（等价于整体平移这个阈值）。
    floor = float(args.chroma_min)
    p995 = float(np.percentile(chroma, 99.5))
    thr = max(floor, min(0.45 * p995, 2.5 * floor))
    mask = cv2.inRange(chroma, thr, 255.0)

    # 形态学：开运算清掉孤立噪声点，闭运算补上块表面因压痕/反光造成的针孔。
    # 3x3 是有意的——更大的核会把小像素尺寸下的方块轮廓啃掉一圈，
    # 直接损害面积、填充度、凸度这些我们赖以做形状判别的指标。
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    blocks = []
    rejected = []
    for contour in contours:
        m = _shape_metrics(contour)
        x, y, bw, bh = m['bbox']

        # 这些 if 的顺序是按"代价从低到高"排的，同时也是按"误判危害从大到小"：
        # 面积不对的直接排除，不用再算后面的几何量。
        if m['area'] < area_min or m['area'] > area_max:
            continue                      # 太小=噪点/远处杂物，太大=墙面/桌面
        if x <= 0 or y <= 0 or x + bw >= w or y + bh >= h:
            continue                      # 贴边框的一定被截断，尺寸/角度都不可信
        if m['aspect'] > args.aspect_max:
            continue                      # 细长：灯管高光条、线缆、桌沿
        if m['fill'] < args.fill_min:
            continue                      # 最小外接矩形里填不满：细长条 / 锯齿形杂物
        if m['solidity'] < args.solidity_min:
            continue                      # 凹：L 形、被遮挡、半影
        if not (args.vertex_min <= m['vertex_count'] <= args.vertex_max):
            # 顶点数越界时看反查结果：多边形没能包住轮廓才是真的形状不对
            if m.get('poly_outside_dev', 0.0) > 0.5:
                rejected.append({**m, 'reason': 'vertex_count'})
                continue

        # ---- 色相统计：只在"落在该轮廓内部"的像素上做 ----
        # 为什么不直接对整个包围盒统计：盒子里可能混进背景像素，
        # 背景若是中性色（R≈G≈B），它的 H 是纯噪声，会把平均色相拽走。
        sub_hue = hue[y:y + bh, x:x + bw]
        sub_cnt = contour - (x, y)
        inside = np.zeros((bh, bw), np.uint8)
        cv2.drawContours(inside, [sub_cnt], -1, 255, thickness=cv2.FILLED)
        vals = sub_hue[inside > 0]
        mean_h, concentration, n_px = mean_circular_hue(vals)
        if n_px < 12:
            continue

        # 色相集中度太低 = 这片区域颜色不纯（阴影/反光/多种颜色混在一起），
        # 直接丢。宁可漏检也不给机械臂一个错颜色的目标。
        if concentration < args.hue_consistency_min:
            rejected.append({**m, 'reason': 'hue_inconsistent',
                             'concentration': round(concentration, 3)})
            continue

        # ---- 最近参考色 ----
        color, dist, second_d = classify_color(mean_h, color_table)
        if color is None or dist > args.hue_tol:
            rejected.append({**m, 'reason': 'no_color_match',
                             'mean_hue': round(mean_h, 1),
                             'nearest': color, 'dist': round(dist, 1)})
            continue
        # 次近色离得越远，说明这一块的颜色越"没有歧义"。
        # 注意：这里**不**否决，只降置信度——因为"褪色的黄偏绿"这一类
        # 边界情况还是要报出来给人看，直接丢弃反而更难排查。
        if second_d >= 1e8:
            margin_score = 1.0
        else:
            margin_score = max(0.0, min(1.0, (second_d - dist) / args.hue_tol))

        # ---- 几何：minAreaRect 给出朝向和边长 ----
        rect = m['rect']
        (rcx, rcy), (_rw, _rh), _rangle = rect
        rect_w, rect_h = m['rect_w'], m['rect_h']
        if rect_w <= 0 or rect_h <= 0:
            continue
        box = cv2.boxPoints(rect)
        # 让 angle_deg 描述"某条边的方向"：boxPoints 不保证长短边顺序，
        # 所以挑较长的那条边，朝向才在帧与帧之间稳定（不会突然跳 90°）。
        e1 = box[1] - box[0]
        e2 = box[2] - box[1]
        edge = e1 if math.hypot(float(e1[0]), float(e1[1])) >= \
            math.hypot(float(e2[0]), float(e2[1])) else e2
        angle_deg = wrap_half_open(
            math.degrees(math.atan2(float(edge[1]), float(edge[0]))))

        # ---- 置信度 ----
        # 三个可解释的因子合成，任一项差就拉低总分，方便下游在 /blocks 上设阈值：
        #   color_conf ← 色相一致性 R（这块料的颜色纯不纯）
        #                + 彩色像素占比（有没有把影子/反光边缘算进来）
        #                + 与次近色的间隔（有没有卡在两个标签中间）
        #   shape_conf ← 填充率与凸度（像不像方/圆，而不是被啃过的不规则块）
        # shape_conf 用 m['fill']（最小外接矩形填充率）而不是 extent：
        # 后者对旋转敏感，会让斜放的方块凭白掉分。
        coverage = min(1.0, n_px / max(1.0, m['area']))
        color_conf = min(1.0, 0.55 * concentration + 0.25 * coverage
                         + 0.20 * margin_score)
        shape_conf = min(1.0, 0.6 * m['fill'] + 0.4 * m['solidity'])
        confidence = float(max(0.0, min(1.0, color_conf * shape_conf)))
        if confidence < args.min_confidence:
            rejected.append({**m, 'reason': 'low_confidence',
                             'confidence': round(confidence, 3)})
            continue

        blocks.append({
            # ---- 需要交付的六项 ----
            'color': color,
            'cx': float(rcx),
            'cy': float(rcy),
            'angle_deg': float(angle_deg),
            'size_px': float(math.sqrt(rect_w * rect_h)),
            'confidence': confidence,
            'contour': contour.reshape(-1, 2).astype(int).tolist(),
            # ---- 诊断量（进不了 /blocks 的 JSON，只用于调参和调试图）----
            'mean_hue': float(mean_h),
            'hue_concentration': float(concentration),
            'hue_distance': float(dist),
            'color_margin': None if second_d >= 1e8 else float(second_d - dist),
            'rect_w': rect_w,
            'rect_h': rect_h,
            'fill': float(m['fill']),
            'extent': float(m['extent']),
            'solidity': float(m['solidity']),
            'aspect': float(m['aspect']),
            'vertex_count': int(m['vertex_count']),
            'coverage': float(coverage),
            'area': float(m['area']),
        })

    # 按置信度降序、再按面积降序：下游通常只关心"最像的那几块"，
    # 稳定排序也让 /blocks 的 JSON 在帧间可比对（顺序不会乱跳）。
    blocks.sort(key=lambda blk: (-blk['confidence'], -blk['area']))
    if args.limit and args.limit > 0:
        blocks = blocks[:args.limit]

    info = {
        'chroma_threshold': float(thr),
        'chroma_p995': p995,
        'chroma_adaptive_raised': bool(thr > floor + 1e-6),
        'colorful_pixels': int(mask.sum() // 255),
        'candidates': int(len(contours)),
        'area_range': (float(area_min), float(area_max)),
        'rejected': rejected[:10],
    }
    if verbose:
        log(f'色度阈值 {thr:.0f}（绝对下限 {floor:.0f}，场景 99.5 百分位 {p995:.0f}'
            f'{"，本次由百分位自适应抬高" if info["chroma_adaptive_raised"] else "，本次由下限决定"}）'
            f'，彩色像素 {info["colorful_pixels"]}，外轮廓 {info["candidates"]}'
            f'，面积区间 {area_min:.0f}~{area_max:.0f}，命中 {len(blocks)}')
        for r in info['rejected']:
            log(f'  丢弃 area={r["area"]:.0f} bbox={r["bbox"]} '
                f'fill={r["fill"]:.2f} solidity={r["solidity"]:.2f} '
                f'aspect={r["aspect"]:.2f} 顶点={r["vertex_count"]} '
                f'原因={r["reason"]}', 'DEBUG')
    return blocks, mask, info


def classify_color(mean_h, color_table):
    """最近参考色（按环绕角距离）。

    返回 (标签, 到该标签的距离, 到次近标签的距离)；表为空时返回 (None, inf, inf)。
    """
    best_label, best_d, second_d = None, 1e9, 1e9
    for label, center in color_table:
        d = ang_diff_deg(mean_h, center)
        if d < best_d:
            best_label, second_d, best_d = label, best_d, d
        elif d < second_d:
            second_d = d
    if best_label is None:
        return None, 1e9, 1e9
    return best_label, best_d, second_d


# ---------------------------------------------------------------------------
# 绘制 / 输出
# ---------------------------------------------------------------------------

_PALETTE = [(0, 255, 0), (0, 200, 255), (255, 128, 0), (255, 0, 255),
            (255, 255, 0), (128, 0, 255), (0, 255, 255), (255, 255, 255)]


def annotate(image, blocks, mask=None, info=None):
    """把检测结果画到图上，供人眼复核。

    调试图的价值在于定位失败环节：一眼看出是"掩码里就没有那块料"
    （光照/色度阈值问题）还是"掩码有但被形状过滤掉了"（几何参数问题），
    这决定了下一步该调哪一组参数。所以右侧会并排贴一份二值掩码。
    """
    canvas = image.copy()
    for i, blk in enumerate(blocks):
        color_bgr = _PALETTE[i % len(_PALETTE)]
        cnt = np.asarray(blk['contour'], np.int32).reshape(-1, 1, 2)
        cv2.drawContours(canvas, [cnt], -1, color_bgr, 2)

        cx, cy = int(round(blk['cx'])), int(round(blk['cy']))
        cv2.circle(canvas, (cx, cy), 3, (0, 0, 255), -1)

        rect = ((blk['cx'], blk['cy']), (blk['rect_w'], blk['rect_h']),
                blk['angle_deg'])
        cv2.drawContours(canvas, [cv2.boxPoints(rect).astype(np.int32)],
                         -1, (255, 255, 255), 1)

        label = (f"{blk['color']} #{i} ({cx},{cy}) "
                 f"{blk['angle_deg']:+.1f}deg {blk['size_px']:.0f}px "
                 f"h{blk['mean_hue']:.0f} conf{blk['confidence']:.2f}")
        ty = max(14, int(blk['cy'] - blk['rect_h'] * 0.6) - 6)
        tx = max(2, min(int(blk['cx'] - 80), max(2, canvas.shape[1] - 300)))
        # 先黑后彩描两遍：亮背景上白字看不见，暗背景上彩色字看不清，
        # 描边能同时解决这两种情况。
        cv2.putText(canvas, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(canvas, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    color_bgr, 1, cv2.LINE_AA)

    head = f'blocks={len(blocks)}'
    if info:
        head += (f" | chroma thr={info['chroma_threshold']:.0f}"
                 f" (p99.5 {info['chroma_p995']:.0f})"
                 f" | colorful px={info['colorful_pixels']}")
    cv2.putText(canvas, head, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(canvas, head, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 0), 1, cv2.LINE_AA)

    if mask is not None:
        vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        if vis.shape[:2] != canvas.shape[:2]:
            vis = cv2.resize(vis, (canvas.shape[1], canvas.shape[0]))
        canvas = np.hstack([canvas, vis])
    return canvas


def imwrite(path, image):
    """写图但绝不因为失败而崩：无人值守跑 ROS 时 /tmp 满或没权限都可能发生。"""
    try:
        ok = cv2.imwrite(path, image)
        if not ok:
            log(f'写图失败（cv2.imwrite 返回 False）：{path}', 'WARN')
        return bool(ok)
    except Exception as exc:
        log(f'写图异常 {path}: {exc}', 'WARN')
        return False


def round_block(blk, nd=4):
    """给 /blocks 用的精简结构。

    刻意不带 contour：JSON 每帧都发，几百个坐标点会让话题带宽和
    `ros2 topic echo` 的可读性一起崩掉；需要轮廓请看调试图。
    """
    return {
        'color': blk['color'],
        'cx': round(blk['cx'], nd),
        'cy': round(blk['cy'], nd),
        'angle_deg': round(blk['angle_deg'], nd),
        'size_px': round(blk['size_px'], nd),
        'confidence': round(blk['confidence'], nd),
    }


def blocks_message(blocks, stamp=None):
    """组装发到 /blocks 的 JSON 字符串。

    结构固定为 {"stamp": float, "blocks": [...]}：
    下游有 stamp 才能判断数据新旧、做时间对齐；没有它就只能"收到就算最新"。
    """
    payload = {
        'stamp': round(float(time.time() if stamp is None else stamp), 4),
        'blocks': [round_block(b) for b in blocks],
    }
    return json.dumps(payload, ensure_ascii=False)


# ---------------------------------------------------------------------------
# 离线模式
# ---------------------------------------------------------------------------

def run_offline(args, color_table, hue_centers):
    image = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if image is None:
        log(f'读不到图片：{args.image}', 'ERROR')
        return 2

    undist = Undistorter(args.intrinsics)
    for note in undist.notes:
        log(note)
    frame = undist.apply(image)

    blocks, mask, info = detect_blocks(frame, args, color_table, hue_centers,
                                       verbose=True)
    print(blocks_message(blocks))

    if args.out:
        imwrite(args.out, annotate(frame, blocks, mask, info))
        log(f'已写出标注图：{args.out}')
    if args.save_mask:
        imwrite(args.save_mask, mask)
        log(f'已写出掩码：{args.save_mask}')
    if args.json_out:
        try:
            with open(args.json_out, 'w', encoding='utf-8') as fh:
                fh.write(blocks_message(blocks) + '\n')
            log(f'已写出 JSON：{args.json_out}')
        except OSError as exc:
            log(f'写 JSON 失败：{exc}', 'WARN')
    return 0


# ---------------------------------------------------------------------------
# ROS 2 模式
# ---------------------------------------------------------------------------

def run_ros(args, color_table, hue_centers):
    """ROS 2 模式：只发 /blocks（std_msgs/String），绝不下发任何控制指令。"""
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError as exc:
        log(f'ROS 模式需要 rclpy（{exc}）。本机没有 ROS 时请用离线模式：'
            '--image xxx.png --out yyy.png', 'ERROR')
        return 2

    class BlockDetectorNode(Node):
        def __init__(self):
            super().__init__('color_block_detect')
            self.pub = self.create_publisher(String, args.topic, 10)
            self.undist = Undistorter(args.intrinsics,
                                      log_fn=self.get_logger().warn)
            for note in self.undist.notes:
                self.get_logger().info(note)
            self.cap = None
            self.frames = 0
            self.fail_streak = 0
            self.last_status = 0.0
            self.last_debug = 0.0
            self.create_timer(1.0 / max(0.5, float(args.rate)), self.tick)
            self.get_logger().info(
                f'开始检测：相机 {args.camera} {args.width}x{args.height}，'
                f'发布到 {args.topic} @ {args.rate:g}Hz'
                '（仅视觉识别，不控制机械臂）')

        def _ensure_camera(self):
            if self.cap is not None:
                return True
            cap = open_camera(args.camera, args.width, args.height)
            if cap is None:
                # 相机打不开是最常见的现场故障（被占用/权限/拔了）。
                # 节流提示 + 继续重试，绝不因为一次失败就退出进程。
                now = time.time()
                if now - self.last_status > 5.0:
                    self.last_status = now
                    self.get_logger().warn(
                        f'打不开相机 {args.camera}（可能被占用/无权限/未插），'
                        '持续重试；Ctrl+C 退出')
                return False
            self.cap = cap
            self.last_status = time.time()
            self.get_logger().info(f'相机 {args.camera} 已打开')
            return True

        def _release(self):
            if self.cap is not None:
                try:
                    self.cap.release()
                except Exception:
                    pass
                self.cap = None

        def tick(self):
            if not self._ensure_camera():
                return
            try:
                ok, frame = self.cap.read()
            except Exception as exc:
                self.get_logger().warn(f'读帧异常：{exc}')
                ok, frame = False, None
            if not ok or frame is None:
                self.fail_streak += 1
                if self.fail_streak >= 20:
                    # 连续读失败通常意味着设备掉了。释放句柄重新走 open 流程，
                    # 而不是对着一个死掉的对象一直 read()。
                    self.get_logger().warn('连续读帧失败，重连相机')
                    self._release()
                    self.fail_streak = 0
                return
            self.fail_streak = 0
            self.frames += 1

            frame = self.undist.apply(frame)
            blocks, mask, info = detect_blocks(frame, args, color_table,
                                               hue_centers, verbose=False)
            self.pub.publish(String(data=blocks_message(blocks)))

            if args.print_every and self.frames % args.print_every == 0:
                self.get_logger().info(
                    f'帧 {self.frames}: {len(blocks)} 块 '
                    + ', '.join(f"{b['color']}@({b['cx']:.0f},{b['cy']:.0f})"
                                for b in blocks[:4]))

            if args.debug_image:
                now = time.time()
                if now - self.last_debug >= args.debug_period:
                    self.last_debug = now
                    imwrite(args.debug_path, annotate(frame, blocks, mask, info))

    rclpy.init(args=None)
    node = BlockDetectorNode()

    # Ctrl+C 的优雅退出：这里的主循环是手写的（不用 rclpy.spin），
    # 所以必须自己置停止标志位，并保证相机一定被释放——
    # 否则下一次启动会因为设备被本进程占着而"打不开相机"。
    stop = {'flag': False}

    def _on_signal(_signum, _frame):
        stop['flag'] = True

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, _on_signal)
        except (ValueError, OSError):
            pass    # 非主线程里装不了信号处理器，忽略

    rate = node.create_rate(max(0.5, float(args.rate)))
    try:
        while rclpy.ok() and not stop['flag']:
            rclpy.spin_once(node, timeout_sec=0.05)
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('退出中：释放相机、销毁节点')
        node._release()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            # 退出路径上的异常绝不能掩盖"已经收到了 Ctrl+C"，所以吞掉
            pass
    return 0


# ---------------------------------------------------------------------------
# 自检（不需要相机、ROS、标定文件）
# ---------------------------------------------------------------------------

SS = 4      # 合成图的超采样倍率，见 _synthetic_frame 的说明


def _blank_frame(width=640, height=480, lo=135, hi=180):
    """合成图的"桌面"底：纯灰 + 左右亮度梯度（模拟一侧打光）。"""
    gradient = np.linspace(lo, hi, width, dtype=np.float32)[None, :]
    gray = np.repeat(gradient, height, axis=0)
    return np.dstack([gray, gray, gray]).astype(np.uint8)


def _draw_rect(canvas, cx, cy, box, hsv_color, angle=0.0, ss=SS):
    """在 box x box 的方形 ROI 里画一个（可旋转的）实心方块并贴到 canvas 上。

    hsv_color=(H,S,V)；canvas 坐标是 1x 尺度。
    box 是 ROI 的边长；方块的实际边长会被缩到 box/(|cos|+|sin|)，
    这样任意角度下方块的**外接框都正好等于 box**——原因见下面那段。
    返回方块的实际边长（浮点像素），自检拿它当真值。
    """
    big = box * ss
    center = big / 2.0
    # 先算"旋转后外接框恰好等于 ROI"的边长。
    # 如果不管角度、直接按 box 当边长画，旋转后外接框会超过 ROI，
    # 贴图时被裁掉四个角，方块就变成了菱形——菱形的 minAreaRect 在任何方向
    # 面积都差不多，旋转卡壳会锁死在轴对齐包围盒上，测出的朝向恒为 0°。
    # 这个 bug 的表现（角度永远是 0）和它的根因（合成图渲染）离得非常远，
    # 排查时务必要记住这条链路。
    span = abs(math.cos(math.radians(angle))) + abs(math.sin(math.radians(angle)))
    side = box / max(1.0, span)
    half = side * ss / 2.0
    th = math.radians(angle)
    rot = np.array([[math.cos(th), -math.sin(th)],
                    [math.sin(th), math.cos(th)]])
    corners = np.array([[-half, -half], [half, -half],
                        [half, half], [-half, half]], np.float64)
    corners = corners @ rot.T + center

    mask_big = np.zeros((big, big), np.uint8)
    cv2.fillPoly(mask_big, [np.round(corners).astype(np.int32)], 255)
    # ss 倍超采样再面积平均：边界得到亚像素级的软过渡，和真实镜头的低通效果一致。
    # alpha 二值化阈值取 0.25 而不是 0.5：旋转方块的四个"尖角"是远小于一像素的
    # 三角形，盖不满半个像素，按 0.5 判就整块被切掉，方块又被削圆。
    mask_small = cv2.resize(mask_big, (box, box), interpolation=cv2.INTER_AREA)
    alpha = (mask_small > 64).astype(np.float32)[:, :, None]

    bgr = cv2.cvtColor(np.uint8([[list(hsv_color)]]), cv2.COLOR_HSV2BGR)[0, 0]
    patch = np.zeros((box, box, 3), np.uint8)
    patch[:, :] = bgr

    half_i = box // 2
    x0, y0 = int(cx - half_i), int(cy - half_i)
    if x0 < 0 or y0 < 0 or x0 + box > canvas.shape[1] or y0 + box > canvas.shape[0]:
        raise ValueError('合成方块超出画面范围，检查真值坐标')
    roi = canvas[y0:y0 + box, x0:x0 + box]
    canvas[y0:y0 + box, x0:x0 + box] = (patch * alpha
                                        + roi.astype(np.float32)
                                        * (1.0 - alpha)).astype(np.uint8)
    return side


def _synthetic_frame(width=640, height=480, lo=135, hi=180, noise=2.5, seed=7):
    """合成一帧"类实拍"图：亮度梯度 + 噪声 + 已知颜色方块 + 干扰物。

    为什么必须有亮度梯度：EVA 场景最常见的失败模式就是"灯在一侧，
    画面一半亮一半暗"。如果检测对亮度敏感，梯度图上的颜色判定必然出错，
    自检就该挂掉。梯度 lo→hi 相当于约 1.33 倍的曝光差。
    """
    rng = np.random.default_rng(seed)
    frame = _blank_frame(width, height, lo, hi)

    # 五个"真值"方块：(标签, 中心x, 中心y, ROI边长, H, S, V, 旋转角)
    # 两块黄是为了验证"重复标签"（用户说第五块可能是第二种黄）。
    # 第 9 项 side 由 _draw_rect 回填：旋转后方块被缩到"外接框=ROI"，它是真值边长。
    specs = [
        ['red', 110, 130, 48, 0, 225, 215, 0.0, 0.0],
        ['yellow', 300, 120, 44, 30, 235, 230, 20.0, 0.0],
        ['green', 480, 140, 52, 60, 210, 205, 0.0, 0.0],
        ['purple', 160, 340, 46, 150, 215, 210, -15.0, 0.0],
        ['yellow', 420, 330, 50, 30, 225, 220, 0.0, 0.0],
    ]
    for spec in specs:
        spec[8] = _draw_rect(frame, spec[1], spec[2], spec[3],
                             (spec[4], spec[5], spec[6]), spec[7])

    # 干扰物 1：细长的橙色反光条（模拟灯管高光/线缆）。它有明确的色相，
    # 所以一定会进入彩色掩码——能不能挡住它，检验的才是**形状过滤**。
    stripe = cv2.cvtColor(np.uint8([[[15, 170, 245]]]),
                          cv2.COLOR_HSV2BGR)[0, 0]
    frame[250:258, 200:380] = stripe
    # 干扰物 2：低色度的灰块（模拟桌面上的金属件）。它形状很方，
    # 只能靠颜色下限挡住——检验的是**前景分割**而不是形状过滤。
    _draw_rect(frame, 545, 400, 40, (90, 6, 130))
    # 干扰物 3：一块青色方块（默认 --colors 表里没有青色）。它形状、色度都合格，
    # 理应被"最近参考色距离过大"挡掉——验证颜色表可配置且**不会乱贴标签**。
    _draw_rect(frame, 300, 400, 44, (90, 220, 220))

    # 传感器噪点；抗锯齿已经在 _draw_rect 里做过了（见那里的说明）。
    noisy = frame.astype(np.float32) + rng.normal(0.0, noise, frame.shape)
    return np.clip(noisy, 0, 255).astype(np.uint8), specs


def _probe_hue(args, h, s, v, box=48, cx=320, cy=240):
    """在一张干净的梯度底图上贴一块纯色，走完整检测链，量出它被判定的色相。

    这是自检里最有价值的一条：它验证"参考色相表 ↔ OpenCV 实际测出的色相"
    是闭合的。只写表不验证，很容易出现"表里写 30、实际测出 52"这种静默错位——
    检测永远不命中，还查不出原因（因为所有阈值看起来都"合理"）。

    刻意用干净的底图而不是那张布满方块和干扰物的合成图：
    探针贴在杂乱背景上时，可能与旁边的反光条在掩码里连成一片，
    量到的就不是探针自己的色相了（这个坑在写这个自检时真踩过）。

    参考表里放的**就是被测的那个色相**，所以这一条检查同时验证了
    分类器和色相量测：若两者不自洽，检测结果会是空的。
    """
    img = _blank_frame()
    _draw_rect(img, cx, cy, box, (h, s, v))
    blocks, _mask, _info = detect_blocks(img, args, [('probe', float(h))], {},
                                         verbose=False)
    for blk in blocks:
        if abs(blk['cx'] - cx) < 8 and abs(blk['cy'] - cy) < 8:
            return blk
    return None


def run_selftest(args, color_table, hue_centers):
    """自检：合成图 + 断言，全程不需要相机/ROS/不带内参。

    检查项：
      A. 参考色相表与实测色相闭环（防"表写错了"这种静默失效）；
      B. 五个已知方块全部找到，颜色标签正确（含重复的黄）；
      C. 质心 / 朝向 / 边长的量测误差在容差内；
      D. 干扰物（细长反光条、低色度灰块、表外的青色块）都不被误报。
    """
    failures = []
    notes = []

    def check(cond, ok_msg, fail_msg):
        if cond:
            notes.append(f'  [OK] {ok_msg}')
        else:
            failures.append(f'  [FAIL] {fail_msg}')

    log('=== color_block_detect 自检开始（合成图，不碰相机/ROS/标定）===')
    frame, specs = _synthetic_frame()
    # 自检刻意不读真实内参文件：同一份代码在不同机器上必须给出同样的结论。
    probe_args = argparse.Namespace(**vars(args))
    probe_args.intrinsics = None

    # ---- A. 色相表闭环 ----
    if not args.no_selftest_hue_check:
        for label, center in color_table:
            probe_h = {'red': 0, 'yellow': 30, 'green': 60, 'purple': 150}.get(label)
            if probe_h is None:
                continue
            blk = _probe_hue(probe_args, probe_h, 225, 215)
            if blk is None:
                failures.append(f'  [FAIL] 色相闭环：纯 {label} 色块没有被检出')
                continue
            d = ang_diff_deg(blk['mean_hue'], center)
            check(d <= 4.0,
                  f'色相闭环 {label}: 表 {center:.0f} vs 实测 {blk["mean_hue"]:.1f}'
                  f'（差 {d:.1f}）',
                  f'色相闭环 {label}: 表 {center:.0f} 与实测 {blk["mean_hue"]:.1f} '
                  f'差 {d:.1f} > 4，颜色表需要校正')

    # ---- B/C/D. 完整检测 ----
    blocks, mask, info = detect_blocks(frame, probe_args, color_table,
                                       hue_centers, verbose=True)
    log(f'自检检测到 {len(blocks)} 块：'
        + '; '.join(f"{b['color']}@({b['cx']:.0f},{b['cy']:.0f})"
                    f"/{b['angle_deg']:+.1f}deg/{b['size_px']:.0f}px"
                    f"/conf{b['confidence']:.2f}" for b in blocks))

    check(len(blocks) == len(specs),
          f'方块数量正确（{len(blocks)}/{len(specs)}）',
          f'方块数量不符：期望 {len(specs)}，实测 {len(blocks)}'
          f'（多出来的是把干扰物当真块，少了的是漏检）')

    # 每个真值找距离最近的检测做一对一匹配（避免"都检到了但位置全错"被算通过）
    used = set()
    for idx, (label, cx, cy, _box, _h, _s, _v, angle, side) in enumerate(specs):
        best, best_d, best_j = None, 1e9, None
        for j, blk in enumerate(blocks):
            if j in used:
                continue
            d = math.hypot(blk['cx'] - cx, blk['cy'] - cy)
            if d < best_d:
                best, best_d, best_j = blk, d, j
        if best is None or best_d > 6.0:
            failures.append(f'  [FAIL] 真值#{idx} {label}@({cx},{cy}) 没有对应检测'
                            f'（最近距离 {best_d:.1f}px）')
            continue
        used.add(best_j)
        check(best['color'] == label,
              f'真值#{idx} 颜色 {best["color"]} 正确'
              f'（mean_hue={best["mean_hue"]:.1f}，真值色相'
              f'{specs[idx][4]}）',
              f'真值#{idx}@({cx},{cy}) 颜色错误：期望 {label}，实测 {best["color"]}'
              f'（mean_hue={best["mean_hue"]:.1f}）')
        check(abs(best['cx'] - cx) <= 2.5 and abs(best['cy'] - cy) <= 2.5,
              f'真值#{idx} 质心误差 ({best["cx"] - cx:+.1f},{best["cy"] - cy:+.1f})px',
              f'真值#{idx} 质心偏差过大：({best["cx"]:.1f},{best["cy"]:.1f})'
              f' vs ({cx},{cy})')
        # 真值边长 side 是 _draw_rect 实际画出来的边长（旋转方块的边长会被缩到
        # 让外接框等于 ROI），所以这里量的是"检测 vs 实际渲染"，不是跨模块猜数。
        check(abs(best['size_px'] - side) <= max(4.0, 0.10 * side),
              f'真值#{idx} 边长 {best["size_px"]:.1f}px（真值 {side:.1f}px）',
              f'真值#{idx} 边长偏差过大：{best["size_px"]:.1f} vs {side:.1f}')
        # 正方形有 90° 对称性：真值 20° 与 -70° 在物理上等价，所以比"模 90 的环绕差"
        d_ang = abs(best['angle_deg'] - angle) % 90.0
        d_ang = min(d_ang, 90.0 - d_ang)
        check(d_ang <= 8.0,
              f'真值#{idx} 朝向 {best["angle_deg"]:+.1f}°'
              f'（真值 {angle:+.1f}°，差 {d_ang:.1f}°）',
              f'真值#{idx} 朝向偏差过大：{best["angle_deg"]:+.1f} vs {angle:+.1f}')

    # ---- D. 干扰物 ----
    stripe = [b for b in blocks
              if abs(b['cy'] - 254) < 12 and abs(b['cx'] - 290) < 100]
    check(not stripe,
          '细长橙色反光条被形状过滤挡掉（aspect/fill 生效）',
          f'细长反光条被误检成方块：{stripe}')
    check(not any(abs(b['cx'] - 545) < 30 and abs(b['cy'] - 400) < 30
                  for b in blocks),
          '低色度灰块被颜色下限挡掉',
          '低色度灰块被误检——请调高 --chroma-min')
    check(not any(abs(b['cx'] - 300) < 25 and abs(b['cy'] - 400) < 25
                  for b in blocks),
          '颜色表外的青色块没有被乱贴标签',
          '颜色表外的青色块被判成了别的颜色——检查 --hue-tol / --colors')

    # ---- E. 输出契约（/blocks 给下游解析，字段和取值范围必须稳）----
    payload = json.loads(blocks_message(blocks))
    keys = {'color', 'cx', 'cy', 'angle_deg', 'size_px', 'confidence'}
    check(isinstance(payload.get('stamp'), float)
          and all(set(b) == keys for b in payload['blocks']),
          f'JSON 字段与约定一致（stamp + {sorted(keys)}）',
          f'JSON 字段不符合约定：{payload["blocks"][:1]}')
    h_img, w_img = frame.shape[:2]
    check(all(0.0 <= b['confidence'] <= 1.0 for b in blocks)
          and all(0.0 <= b['cx'] < w_img and 0.0 <= b['cy'] < h_img for b in blocks)
          and all(-45.0 <= b['angle_deg'] < 45.0 for b in blocks),
          '置信度∈[0,1]、质心在画面内、朝向∈[-45,45)：取值范围正确',
          '取值越界：' + str([(round(b['confidence'], 2), round(b['cx'], 1),
                             round(b['cy'], 1), round(b['angle_deg'], 1))
                            for b in blocks]))

    if args.out:
        imwrite(args.out, annotate(frame, blocks, mask, info))
        log(f'自检标注图：{args.out}')

    print('\n'.join(notes))
    if failures:
        print('\n'.join(failures))
        log(f'自检失败：{len(failures)} 项不通过', 'ERROR')
        return 1
    log('自检通过 ✅（所有断言成立）')
    return 0


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description='EVA 彩色方块识别（纯视觉：不下发关节指令、不控制机械臂）',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    mode = ap.add_argument_group('运行模式（三选一）')
    mode.add_argument('--image', help='离线模式：读这张图')
    mode.add_argument('--out', help='把标注图写到这个路径（离线/自检都可用）')
    mode.add_argument('--json-out', help='离线模式：把检测结果 JSON 写到这个路径')
    mode.add_argument('--save-mask', help='离线模式：另外单独存一份二值掩码图')
    mode.add_argument('--ros', action='store_true',
                      help='ROS 2 模式：开相机并把 JSON 发到 /blocks')
    mode.add_argument('--selftest', action='store_true',
                      help='自检：合成图跑一遍并断言（不需要相机/ROS）')

    cam = ap.add_argument_group('相机')
    cam.add_argument('--camera', type=int, default=0, help='摄像头索引')
    cam.add_argument('--width', type=int, default=640)
    cam.add_argument('--height', type=int, default=480)
    cam.add_argument('--intrinsics', default=DEFAULT_INTRINSICS,
                     help='相机内参 YAML；文件不存在则提示并继续用原始图')

    ros = ap.add_argument_group('ROS 2')
    ros.add_argument('--topic', default='/blocks',
                     help='发布话题（std_msgs/String，JSON 文本）')
    ros.add_argument('--rate', type=float, default=10.0, help='检测频率 Hz')
    ros.add_argument('--debug-image', action='store_true',
                     help='周期性往 /tmp 写调试图（无人值守跑时用）')
    ros.add_argument('--debug-path', default=DEFAULT_DEBUG_IMAGE)
    ros.add_argument('--debug-period', type=float, default=1.0,
                     help='调试图写盘间隔（秒）')
    ros.add_argument('--print-every', type=int, default=0,
                     help='每 N 帧打印一次检测摘要，0 = 不打印')

    col = ap.add_argument_group('颜色')
    col.add_argument('--colors', default=','.join(DEFAULT_COLOR_TABLE),
                     help='颜色表 "标签:参考色相" 逗号分隔；标签后加 + 允许重复')
    col.add_argument('--hue-tol', type=float, default=DEFAULT_HUE_TOL,
                     help='判定为某颜色的最大色相偏差（0-179 半角单位）')
    col.add_argument('--hue-consistency-min', type=float, default=0.55,
                     help='连通域内色相集中度下限（0-1），低于此视为颜色不纯')
    col.add_argument('--chroma-min', type=float, default=DEFAULT_CHROMA_MIN,
                     help='色度绝对下限（Lab 单位）；Otsu 结果低于它时采用它')
    col.add_argument('--min-confidence', type=float, default=0.35,
                     help='置信度下限，低于此不上报')

    shp = ap.add_argument_group('形状过滤')
    shp.add_argument('--area-min', type=float, default=None,
                     help='轮廓面积下限（像素²）；默认按画面尺寸自适应')
    shp.add_argument('--area-max', type=float, default=None,
                     help='轮廓面积上限（像素²）；默认按画面尺寸自适应')
    shp.add_argument('--aspect-max', type=float, default=1.8,
                     help='包围盒长宽比上限（正方形=1.0）')
    shp.add_argument('--fill-min', type=float, default=0.70,
                     help='轮廓面积/最小外接矩形面积 下限（正方形≈0.95，圆≈0.785；'
                          '注意别用 extent，那个对旋转敏感）')
    shp.add_argument('--solidity-min', type=float, default=0.80,
                     help='轮廓面积/凸包面积 下限（正方形≈1.0）')
    shp.add_argument('--vertex-min', type=int, default=VERTEX_MIN,
                     help='approxPolyDP 顶点数下限')
    shp.add_argument('--vertex-max', type=int, default=VERTEX_MAX,
                     help='approxPolyDP 顶点数上限（正方形=4，圆形物可放到 8~10）')
    shp.add_argument('--limit', type=int, default=0,
                     help='每帧最多上报几块（0 = 不限）')

    st = ap.add_argument_group('自检')
    st.add_argument('--no-selftest-hue-check', action='store_true',
                    help='自检时跳过"色相表闭环"检查')

    args = ap.parse_args(argv)
    if not (args.ros or args.selftest or args.image):
        ap.error('必须指定一种模式：--image 图片 / --ros / --selftest')
    return args


def main(argv=None):
    args = parse_args(argv)
    hue_centers = dict(DEFAULT_HUE_CENTERS)
    color_table = parse_color_table(args.colors, hue_centers)
    if not color_table:
        log('颜色表为空，没有可判定的颜色（用 --colors 指定，比如 "red:0,yellow:30"）',
            'ERROR')
        return 2
    log('颜色表：' + ', '.join(f'{lbl}:{center:g}' for lbl, center in color_table))

    if args.selftest:
        return run_selftest(args, color_table, hue_centers)
    if args.ros:
        return run_ros(args, color_table, hue_centers)
    return run_offline(args, color_table, hue_centers)


if __name__ == '__main__':
    sys.exit(main())
