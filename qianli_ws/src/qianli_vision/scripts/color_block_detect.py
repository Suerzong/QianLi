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
5. 三个现场故障都有对应的独立机制（各有自检断言与覆盖率守卫）：
   · **粘连分割**：红块和蓝块贴在一起会连成一个连通域，色相集中度 R 掉到
     0.44，光靠阈值只能整块丢掉（两块一起漏）。这里按**色相**把它切开，
     再各自重新走一遍检测（`--no-split` 可关）。
   · **碎条合并**：同一个块被高光/阴影割出来的细长碎片，按"同色 + 紧贴"
     并回大块（`--no-merge` 可关）。合并前会复查"并起来还纯不纯"，
     免得把刚切开的不同色块又粘回去。
   · **色相漂移**：灯光/白平衡变了会让整幅画面的色相整体旋转。
     但**乱校正比不校正更危险**——现场出现过"凭空估出 +5.8 并把整表挪歪"，
     所以现在只有"把表挪过去能实打实多救回 ≥2 块"时才动手，
     否则原样返回并在 info['hue_offset_note'] 里写明理由（`--no-hue-drift` 可关）。

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

from project_paths import calibration_path, camera_source, default_camera

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

DEFAULT_INTRINSICS = calibration_path('camera_intrinsics.yaml')
DEFAULT_DEBUG_IMAGE = '/tmp/color_blocks_debug.jpg'

# OpenCV 的 H 是 0~179（真实色相角度的一半）：红≈0、黄≈30、绿≈60、青≈90、紫≈130。
#
# ⚠️ 下面这组数值是**这台相机 + 这套灯光**下实测出来的（2026-10 现场标定），
# 来源是 --probe-colors 对该场景的量测：red 175.9、yellow 26.9、green 62.3、
# blue 110.7、purple 130.6。其中 purple 曾经被写成 150（那是"品红/洋红"，
# 真实色相 300°），导致紫色块形状全过却因为距离 19.4 > hue_tol 被永久漏检。
# 换相机、换灯、换一批料，**都请重跑一次 --probe-colors** 再贴回来，
# 不要照抄这里的数字——色表数值本身就是最容易错、又最难察觉的一环。
DEFAULT_HUE_CENTERS = {
    'red': 0,        # 现场实测 175.9（贴 0/180 接线，圆距离 4.1）
    'yellow': 27,    # 现场实测 26.9
    'green': 62,     # 现场实测 62.3
    'blue': 111,     # 现场实测 110.7（H 111 ≈ 真实色相 222°，蓝）
    'cyan': 90,
    'purple': 130,   # 现场实测 130.6（H 130 ≈ 真实色相 261°，紫）
    'yellow2': 38,   # "第五块可能是第二种黄"——单独留一档，默认不作为标签上报
}

# 默认颜色标签表：标签 -> 参考色相。做成参数就是为了不把颜色写死，
# 换一批料（比如换成青色/橙色）只改这张表，不用动代码。
DEFAULT_COLOR_TABLE = ('red:0', 'yellow:27', 'green:62', 'blue:111', 'purple:130')

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

# ---- 上表面（顶面）检测参数 ----
# 侧面的可见高度不到这个比例时，认为"几乎正上方俯视"，直接退化成剪影结果，
# 不去编造看不见的侧面边界。0.06 的依据：4cm 方块在 50px 尺度下，
# 侧面只有 3px 时数学上还能算，但求出来的"顶面近边"位置误差和 3px 同量级，
# 不如老实退化成剪影（那时剪影≈顶面，误差更小）。
TOP_SIDE_VISIBLE_MIN = 0.06
# 亮度剖面的先验中心：顶面近边大致落在方块竖直方向的这个比例处。
# 为什么是 0.42 而不是 0.5：相机从上前方看向下，顶面投影比侧面大一些。
TOP_EDGE_PRIOR = 0.42
TOP_EDGE_PRIOR_SIGMA = 0.18
# 硬窗口：顶面近边的允许位置（占方块竖直方向的比例）。
# 下界 0.12 是因为近边不能贴到方块顶端；上界 0.85 是因为侧面在图像里
# 至少得占一点高度，近边不可能压到方块底边——不设这个上界的话，
# "侧面→背景"那条强梯度会被误当成顶面近边（实测踩过）。
TOP_EDGE_WINDOW = (0.12, 0.85)
# 判定"确实存在亮度台阶"的最小归一化落差：梯度强度/本块亮度动态范围。
# 0.05 的依据：顶面比侧面暗 5% 以上才值得信；再小就是噪声和渐变了。
TOP_EDGE_MIN_DROP = 0.05


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------

def log(msg, level='INFO'):
    """统一日志：保持朴素可 grep 的格式（ROS 自带的日志格式反而不好复制粘贴）。"""
    print(f'[{level}] {msg}', flush=True)


def ang_diff_deg(a, b):
    """两个色相角（0~179 半角单位）的**圆距离**，结果落在 [0, 90]。

    这是全文件最容易写错、也最不能写错的一个函数：OpenCV 的 H 是 0~179 的
    环形刻度，0 和 179 是同一个物理颜色（红）。直接 `abs(a-b)` 会把贴着边界的
    红劈成两半——现场实测红色块色相 175.9（= 标准色相 351.8°，贴着 0/360 接线），
    它到 red:0 的真实距离只有 4.1，而错误的减法会算出 175.9，
    于是"颜色明明很纯的红块"被 no_color_match 丢掉。
    实现上先取模 180 再对折，两处都处理了环绕。
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
    4cm 方块在 640x480、桌面俯视的典型距离下约占 35~90 px 边长，
    折算面积约 1200~8100 px²。

    上限为什么收紧到画面面积的 5%（640x480 → 15360 px²，约 124 px 边长）：
    现场量到**黄色机械臂本体是 9804 px² 的最大连通域**，色相与黄块完全一致
    （hue=27.7 R=1.000）。目前它靠形状（aspect/fill）被挡住，但机械臂姿态
    一变、在画面里显得方一点，就可能被当成黄块上报。物块只有 4cm，
    面积上限按"物块可能的最大投影"设，是最省事又最可靠的护栏。
    代价：贴着镜头（4cm 方块占画面 1/4 以上）时会漏检——那种情况请显式
    调大 --area-max，别把默认值改回去。
    """
    h, w = shape[:2]
    pixels = float(h * w)
    return max(60.0, 0.0004 * pixels), max(400.0, 0.05 * pixels)


def _reject_reason(metrics, reason, **extra):
    """构造一条"被丢弃"的诊断记录。

    只放标量和小元组：这些记录会经过 json.dumps 出现在前端页面上，
    塞进 numpy 数组（比如轮廓点）会让前端整个 /state 接口 500。
    'rect' 这种 OpenCV 返回的嵌套元组也不放进来——JSON 里它没有意义，
    还容易被下游误当成数值用。
    """
    rec = {
        'area': float(metrics['area']),
        'bbox': tuple(int(v) for v in metrics['bbox']),
        'fill': round(float(metrics['fill']), 3),
        'solidity': round(float(metrics['solidity']), 3),
        'aspect': round(float(metrics['aspect']), 3),
        'vertex_count': int(metrics['vertex_count']),
        'reason': reason,
    }
    rec.update(extra)
    return rec


def _polygon_area(points):
    """鞋带公式求多边形面积（点序无所谓，取绝对值）。"""
    pts = np.asarray(points, np.float64).reshape(-1, 2)
    if len(pts) < 3:
        return 0.0
    x, y = pts[:, 0], pts[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def _order_quad(points):
    """把任意顺序的 4 个点排成 [左上, 右上, 右下, 左下]（相对质心的角度序）。

    为什么要统一顺序：上表面的四角要按固定顺序输出，下游（抓取规划）
    才能稳定地取"顶边"和"朝向"；顺序随机的角点等于每次都在抖。
    """
    pts = np.asarray(points, np.float64).reshape(-1, 2)
    if len(pts) != 4:
        return pts
    center = pts.mean(axis=0)
    ang = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
    order = np.argsort(ang)                    # 逆时针（图像坐标下从 +x 起）
    ccw = pts[order]
    # 逆时针序里 y 最小的那个是顶边两端之一，取它和它下一个作为左上/右上
    i_top = int(np.argmin(ccw[:, 1]))
    if ccw[(i_top + 1) % 4, 0] < ccw[i_top, 0]:
        i_top = (i_top + 1) % 4                # 保证"左上"在左边
    return np.roll(ccw, -i_top, axis=0)


def _quad_shape_score(quad, mask_area):
    """给一个上表面四边形打分（0~1），用来在"亮度边界"和"几何退化"两个候选之间选优。

    分数只用与颜色/光照无关的几何量：四边形填充率（相对自己的外接矩形）、
    面积相对于整个剪影是否合理、是否凸。这样在光照很差、亮度边界不可信时，
    几何候选自然胜出，而不是被一个错误的亮度边界带跑。
    """
    if quad is None or len(quad) != 4:
        return 0.0
    pts = np.asarray(quad, np.float64).reshape(-1, 2)
    rect = cv2.minAreaRect(pts.astype(np.float32))
    rw, rh = float(rect[1][0]), float(rect[1][1])
    if rw < 3.0 or rh < 3.0:
        return 0.0
    area = _polygon_area(pts)
    fill = area / (rw * rh)                    # 四边形填充率，平行四边形≈1
    convex = 1.0 if cv2.isContourConvex(pts.astype(np.float32)) else 0.0
    ratio = area / max(1.0, mask_area)         # 顶面应占剪影的 3~9 成
    if ratio < 0.25 or ratio > 1.02:
        return 0.0
    ratio_score = 1.0 - min(1.0, abs(ratio - 0.62) / 0.45)
    return float(max(0.0, 0.60 * fill + 0.25 * convex + 0.15 * ratio_score))


def detect_top_face(image, contour, rect, rect_w, rect_h, guard=6):
    """从"剪影轮廓"里估计**上表面**（朝上那个面）的几何。

    为什么需要它：相机是斜着往下看的，一块立方体的剪影**包含可见侧面**。
    于是剪影质心偏向侧面的那一侧（画面下方），不等于上表面中心；
    剪影边长也混进了侧面的透视缩短。对抓取来说 TCP 要对着上表面中心，
    朝向、尺度也该以上表面为准。

    做法（不依赖颜色，所以对光照和换色都不敏感）：
      1. 用轮廓的最小外接矩形把这一小块**转正**（连同掩码一起 remap），
         转正后顶面近边在图像里几乎是水平线，找水平边界就退化成找行。
      2. 算出每行"属于方块的像素"的平均亮度剖面。EVA 块正对镜头的顶面
         通常比侧面亮（侧面斜对光源/被自身遮挡），所以顶面近边处会出现
         明显的由亮转暗。用平滑梯度 × 位置先验打出每行的得分，取最高分那行。
      3. 该行两侧由掩码列边界裁出端点 → 组成"顶面近边"，和掩码的最上边、
         以及两条侧边界拼成顶面四边形。
      4. 同时算一个纯几何的退化候选（掩码在转正系里的四个极值点），
         用只含几何量的分数择优——光照太平、亮度边界不可信时，
         几何候选会赢，而不是让一个错误的亮度边界把结果带跑。

    返回 dict：top_cx/top_cy/top_angle_deg/top_size_px/corners(4x2)/side_visible，
    退化时 side_visible=False 且几何等于剪影（绝不编造看不见的角点）。
    """
    mask = np.zeros(image.shape[:2], np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, thickness=cv2.FILLED)

    whole = _silhouette_face(rect, rect_w, rect_h, rect_w * rect_h)
    whole['side_visible'] = False
    whole['top_source'] = 'silhouette'          # 完全没找到顶面 → 退回剪影
    whole['top_edge_y'] = 0.0

    # ---- 转正：把最小外接矩形转成轴对齐，行/列才有物理意义 ----
    (rcx, rcy), _, rangle = rect
    big = max(rect_w, rect_h)
    side = int(np.ceil(big * 1.25)) + 2 * guard
    M = cv2.getRotationMatrix2D((float(rcx), float(rcy)), float(rangle), 1.0)
    M[0, 2] += (side / 2.0 - rcx)
    M[1, 2] += (side / 2.0 - rcy)
    rmask = cv2.warpAffine(mask, M, (side, side), flags=cv2.INTER_NEAREST)
    if int((rmask > 0).sum()) < 80:
        return whole
    # 亮度图用最近邻重采样：双线性会在顶面/侧面交界处插值出中间亮度，
    # 把我们要找的那条边界糊掉。
    rgray = cv2.warpAffine(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), M,
                           (side, side), flags=cv2.INTER_NEAREST)
    rr = cv2.boundingRect(rmask)
    x0, y0, rw, rh = rr
    if rw < 4 or rh < 4:
        return whole

    # ---- 每行的平均亮度（只看掩码内的像素）----
    sub_mask = rmask[y0:y0 + rh, x0:x0 + rw] > 0
    sub_gray = rgray[y0:y0 + rh, x0:x0 + rw].astype(np.float32)
    row_sum = (sub_gray * sub_mask).sum(axis=1)
    row_cnt = sub_mask.sum(axis=1)
    if int((row_cnt > 0).sum()) < 6:
        return whole
    sub_bright = np.where(sub_mask, sub_gray, np.nan)   # 非掩码处是 nan

    # ---- 逐列找"顶面→侧面"的亮度台阶 ----
    # 为什么不用"每行平均亮度"：方块在图像里一般是转着的，顶面近边是一条**斜线**，
    # 行平均会把斜线糊成一段平缓的渐变，台阶被抹平（实测就是这么失败的）。
    # 逐列看就没这个问题——每一列上顶面/侧面的交界都是一个清晰的水平台阶。
    # 判据只看台阶**上方**是否明显亮于下方，所以侧面和桌面亮度接近也不影响。
    window = max(1, int(round(rh * 0.035)))
    kernel = np.ones(2 * window + 1, np.float32) / (2 * window + 1)
    lo = max(2, int(round(rh * TOP_EDGE_WINDOW[0])))
    hi = min(rh - 3, int(round(rh * TOP_EDGE_WINDOW[1])))
    if hi <= lo + 1:
        return whole
    rows = np.arange(rh, dtype=np.float32)
    prior = np.exp(-0.5 * ((rows / max(1.0, rh - 1) - TOP_EDGE_PRIOR)
                           / TOP_EDGE_PRIOR_SIGMA) ** 2)
    # 台阶两侧各自需要的最少有效像素数。
    # 这里**不能**按比例放大到 window 以上：before/after 各是半个窗口的均值，
    # 窗口最多 window 个像素，要求超过 window 就等于把所有候选全否掉
    # （这个 bug 的表现是"永远找不到边界、永远退化成剪影"，非常安静）。
    nb_min = max(2, min(3, window * 2, int(round(rh * 0.06)) + 1))
    if rh < 16:
        # 方块只有十几个像素高时，上下各 2~3 像素的均值噪声太大，不值得硬算
        return whole

    best_row, best_score, best_drop = None, 0.0, 0.0
    row_est = np.full(sub_mask.shape[1], np.nan, np.float32)
    for col in range(sub_mask.shape[1]):
        valid_col = ~np.isnan(sub_bright[:, col])
        if int(valid_col.sum()) < 3 * nb_min:
            continue
        filled = np.nan_to_num(sub_bright[:, col])
        # 用前缀和一次算完整列的"上方均值 / 下方均值"。
        # 注意**不能**写成两次卷积再翻转核：方形核是对称的，
        # convolve(x, k) 和 convolve(x, k[::-1]) 结果一模一样，
        # 相减恒等于 0，整个台阶检测就静默失效了（这个坑真踩过）。
        csum = np.concatenate([[0.0], np.cumsum(filled)])
        vsum = np.concatenate([[0], np.cumsum(valid_col.astype(np.int64))])
        col_best, col_score, col_drop = None, 0.0, 0.0
        for r in range(lo, hi):
            if not valid_col[r]:
                continue
            a0, a1 = max(0, r - window), r
            b0, b1 = r, min(len(filled), r + window)
            n_a = int(vsum[a1] - vsum[a0])
            n_b = int(vsum[b1] - vsum[b0])
            if n_a < nb_min or n_b < nb_min:
                continue
            before = (csum[a1] - csum[a0]) / n_a
            after = (csum[b1] - csum[b0]) / n_b
            drop = float(before - after)
            if drop <= 0.0:
                continue                 # 上方不比下方亮 → 不是"顶面→侧面"的台阶
            score = drop * float(prior[r])
            if score > col_score:
                col_best, col_score, col_drop = r, score, drop
        if col_best is None:
            continue
        row_est[col] = float(col_best)
        if col_score > best_score:
            best_score, best_row, best_drop = col_score, col_best, col_drop

    if best_row is None:
        return whole

    # 归一化成"占本块亮度动态范围的比例"：判据与绝对曝光无关，
    # 同一块料在亮/暗环境下标准一致（只比"掉了多少"，不比"掉到多少"）。
    span = float(np.nanmax(sub_bright) - np.nanmin(sub_bright))
    if span < 6.0:
        # 整块亮度几乎均匀 → 没有可靠的亮度台阶，别硬凑（走几何退化路径）
        return whole
    if best_drop / span < TOP_EDGE_MIN_DROP:
        # 台阶太浅：顶面/侧面亮度太接近，硬猜一条不存在的边界不如退化成剪影
        return whole

    # 逐列边界行不能直接逐列用。两个原因：
    #  1) 有些列（尤其是侧面颜色和桌面接近的那一段）根本测不出台阶，留下空洞；
    #  2) 顶面近边在图像里就是一条**直线**（立方体的一条棱），没理由让它弯。
    # 所以对测到的点做直线拟合，再在整个有效列范围上求值：直线天然把空洞外推出去。
    #
    # 拟合必须**抗野值**：靠近掩码边界的那些列，最大梯度其实是"方块→桌面"，
    # 会被误当成顶面近边（实测出现过一根 57 行里跳出来的 28），
    # 普通最小二乘会被这一根带偏整条直线。做法是先拟合一遍，
    # 再用 MAD（中位绝对偏差）把偏离超过 3 倍 MAD 的点剔掉重拟合。
    valid_cols = np.isfinite(row_est)
    n_valid = int(valid_cols.sum())
    if n_valid >= 6:
        xs_fit = np.nonzero(valid_cols)[0].astype(np.float64)
        ys_fit = row_est[valid_cols].astype(np.float64)
        slope, intercept = np.polyfit(xs_fit, ys_fit, 1)
        pred = slope * xs_fit + intercept
        mad = float(np.median(np.abs(ys_fit - pred)))
        # MAD 可能为 0（点几乎共线），给一个像素级下限免得把好点全剔了
        tol = max(2.0, 3.0 * 1.4826 * mad)
        keep = np.abs(ys_fit - pred) <= tol
        if int(keep.sum()) >= 4:
            slope, intercept = np.polyfit(xs_fit[keep], ys_fit[keep], 1)
            pred = slope * xs_fit[keep] + intercept
        resid = float(np.max(np.abs(ys_fit[keep]
                                    - (slope * xs_fit[keep] + intercept))))
        if resid > 0.35 * rh:
            # 剩下的点仍然不是同一条棱 → 说明测到的是互不相干的强梯度，不认
            return whole
        row_est = (slope * np.arange(sub_mask.shape[1]) + intercept
                   ).astype(np.float32)
    elif best_row is not None:
        row_est = np.full(sub_mask.shape[1], float(best_row), np.float32)
    else:
        return whole

    # ---- 用逐列边界行 + 掩码拼顶面四边形（转正坐标系）----
    # 注意用 cv2.transform 而不是 cv2.perspectiveTransform：
    # invertAffineTransform 给的是 2x3 仿射矩阵，perspectiveTransform 只吃 3x3，
    # 传错了 OpenCV **不报错**，会拿垃圾矩阵算出几万像素的坐标，
    # 于是"四边形永远不成形、上表面检测永远退化成剪影"——而且没有任何异常提示。
    # 另外 cand 的坐标是**相对截取出来的子图**的，必须先把 (x0,y0) 加回去，
    # 才是 M 所在的原图坐标系；漏掉这一步会整体偏移几十像素。
    minv = cv2.invertAffineTransform(M)
    offset = np.array([x0, y0], np.float64)

    def _back_to_image(pts):
        moved = np.asarray(pts, np.float64).reshape(-1, 2) + offset
        return cv2.transform(moved.reshape(-1, 1, 2), minv).reshape(-1, 2)

    built = _top_face_from_edge(sub_mask, row_est)
    if built is None:
        return whole
    cand, boundary_y = built
    quad = _back_to_image(cand)

    # ---- 纯几何候选：转正掩码的四个极值点 ----
    ys, xs = np.nonzero(sub_mask)
    geo = np.array([[xs.min(), ys.min()], [xs.max(), ys.min()],
                    [xs.max(), ys.max()], [xs.min(), ys.max()]], np.float64)
    geo_q = _back_to_image(geo)

    mask_area = float(sub_mask.sum())
    s_edge, s_geo = (_quad_shape_score(quad, mask_area),
                     _quad_shape_score(geo_q, mask_area))
    if s_edge < 0.45 and s_geo >= s_edge:
        return whole                              # 两个候选都不可信 → 退化
    chosen, source = (quad, 'brightness_edge') if s_edge >= s_geo \
        else (geo_q, 'geometry_extremes')
    chosen = _order_quad(chosen)

    top_area = _polygon_area(chosen)
    top_rect = cv2.minAreaRect(chosen.astype(np.float32))
    (tcx, tcy), (tw, th), tang = top_rect
    if tw < 3.0 or th < 3.0:
        return whole
    box = cv2.boxPoints(top_rect)
    e1, e2 = box[1] - box[0], box[2] - box[1]
    edge = e1 if math.hypot(float(e1[0]), float(e1[1])) >= \
        math.hypot(float(e2[0]), float(e2[1])) else e2
    top_angle = wrap_half_open(math.degrees(math.atan2(float(edge[1]),
                                                       float(edge[0]))))
    # 侧面可见高度 = 剪影底到顶面近边的距离（在转正系里量，最直观）
    side_h = float(rh - 1 - boundary_y)
    side_visible = (side_h / max(1.0, float(rh))) >= TOP_SIDE_VISIBLE_MIN
    if not side_visible:
        # 侧面几乎看不见 → 剪影≈顶面，退化成剪影结果更准（也避免编造近边）
        return whole
    return {
        'top_cx': float(tcx),
        'top_cy': float(tcy),
        'top_angle_deg': float(top_angle),
        'top_size_px': float(math.sqrt(abs(tw * th))),
        'corners': [[float(px), float(py)] for px, py in chosen],
        'side_visible': True,
        'top_source': source,
        'top_edge_y': side_h,
        'top_fill': round(float(top_area / max(1.0, abs(tw * th))), 3),
    }


def _top_face_from_edge(sub_mask, row_est):
    """由"逐列的顶面近边行"求出顶面四边形（转正坐标系，返回 (4x2, 边界中位行)）。

    做法：顶面是**剪影的一部分**，上边贴着掩码最上沿、下边就是 row_est 给的
    那条近边，左右两条边则与剪影的左右边重合。所以只要定下"左右各收到哪一列"，
    四边形就定了——于是在候选列范围里搜一遍，用纯几何分数（_quad_shape_score）
    挑最像正方形/平行四边形、且面积占剪影比例合理的那个。

    为什么不用"四线拟合再求交点"：剪影的左右边属于**侧面**，比顶面的左右边
    长得多，拿它去拟合顶面的边线会系统性地偏；而且靠近掩码边缘的列里，
    最大梯度是"方块→桌面"而不是"顶面→侧面"，容易混进野值。
    搜索 + 形状打分对这种局部污染天然免疫。
    """
    h, w = sub_mask.shape
    row_est = np.asarray(row_est, np.float64).reshape(-1)
    if row_est.size != w:
        return None
    per_col_top = np.full(w, np.nan, np.float64)
    for cx in range(w):
        col = np.nonzero(sub_mask[:, cx])[0]
        if col.size:
            per_col_top[cx] = float(col.min())
    usable = np.isfinite(per_col_top) & (row_est > per_col_top + 1.0)
    n_usable = int(usable.sum())
    if n_usable < 6:
        return None
    # 中位数滤波压掉个别列的锯齿（形态学留下的 1px 毛刺会让边界抖动）。
    # 自己用 numpy 滑窗做一维中值，不用 cv2.medianBlur：
    # 后者只接受 CV_8U，而 y 坐标在转正坐标系里可能超过 255，会被它拒绝。
    k = int(np.clip(n_usable // 8, 3, 9)) | 1
    for arr in (per_col_top, row_est):
        padded = np.pad(arr, (k // 2, k // 2), mode='edge')
        wins = np.lib.stride_tricks.sliding_window_view(padded, k)
        arr[:] = np.median(wins, axis=1)

    idx = np.nonzero(usable)[0]
    lo_c, hi_c = int(idx.min()), int(idx.max())
    span = hi_c - lo_c
    if span < 6:
        return None
    mask_area = float(sub_mask.sum())
    # 左右各允许往里收一点：测出近边的那几列不一定覆盖顶面的全部宽度
    margins = sorted({int(round(span * f)) for f in
                      (0.0, 0.05, 0.10, 0.16, 0.24, 0.34)})
    best = None
    for m_l in margins:
        for m_r in margins:
            xl2, xr2 = lo_c + m_l, hi_c - m_r
            if xr2 - xl2 < 6:
                continue
            quad = np.array([[xl2, float(per_col_top[xl2])],
                             [xr2, float(per_col_top[xr2])],
                             [xr2, float(row_est[xr2])],
                             [xl2, float(row_est[xl2])]], np.float64)
            score = _quad_shape_score(quad, mask_area)
            if score <= 0.0:
                continue
            # 在形状分数接近的候选里，取面积更大的那个：顶面应该尽量撑满
            # 剪影的上半部分，而不是缩成一个小方块（否则"随便一个小正方形"
            # 也能拿高分）。1e-3 的容差把"分数相同"的候选归到一组再比面积。
            key = (round(score, 3), _polygon_area(quad))
            if best is None or key > best[0]:
                best = (key, quad)
    if best is None:
        return None
    quad = best[1]
    bot_est = float(np.median(row_est[lo_c:hi_c + 1]))
    return quad, bot_est


def _silhouette_face(rect, rect_w, rect_h, _area):
    """剪影本身作为"上表面"的退化结果。

    什么时候用：块转得太斜、侧面几乎看不见（正上方俯视）时，剪影≈上表面，
    这时直接用剪影比硬去猜一条内部边界更准，也不会凭空编出看不见的角点。
    """
    (cx, cy), _, _ = rect
    box = cv2.boxPoints(rect)
    e1, e2 = box[1] - box[0], box[2] - box[1]
    edge = e1 if math.hypot(float(e1[0]), float(e1[1])) >= \
        math.hypot(float(e2[0]), float(e2[1])) else e2
    angle = wrap_half_open(math.degrees(math.atan2(float(edge[1]),
                                                   float(edge[0]))))
    return {
        'top_cx': float(cx),
        'top_cy': float(cy),
        'top_angle_deg': float(angle),
        'top_size_px': float(math.sqrt(max(1.0, rect_w * rect_h))),
        'corners': [[float(px), float(py)] for px, py in _order_quad(box)],
        'top_fill': None,
    }


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


def _otsu_threshold(values_0_179):
    """在一维直方图上做 Otsu，返回阈值下标；直方图太空就返回 None。

    为什么不用 cv2.threshold(..., THRESH_OTSU)：它只接受 CV_8U/CV_16U。
    我们的直方图是浮点计数（像素数会超过 255），直接喂进去会抛
    "src_type is CV_32FC1"。自己实现十几行，语义还更清楚。
    """
    hist = np.asarray(values_0_179, np.float64)
    total = hist.sum()
    if total <= 0 or hist.size < 3:
        return None
    w0 = np.cumsum(hist)
    w1 = total - w0
    valid = (w0 > 0) & (w1 > 0)
    if not np.any(valid):
        return None
    idx = np.arange(hist.size, dtype=np.float64)
    m0 = np.cumsum(hist * idx) / np.maximum(w0, 1e-9)
    m1 = (np.sum(hist * idx) - np.cumsum(hist * idx)) / np.maximum(w1, 1e-9)
    var = w0 * w1 * (m0 - m1) ** 2
    var[~valid] = -1.0
    # 不要首尾两个 bin：那等于"把整块切成 0 像素 + 全部"
    var[:2] = -1.0
    var[-2:] = -1.0
    best = int(np.argmax(var))
    return best if var[best] > 0 else None


def _hue_stats(hue_sub, region_bool):
    """在 region_bool 为真的像素上量色相：返回 (圆均值, 集中度 R, 像素数)。

    R 用圆矢量长度算，范围 [0,1]：R≈1 说明这一片是单一纯色，
    R 明显偏低（现场实测粘连的红+蓝是 0.445）说明里面混了不止一种颜色。
    这正是判断"要不要切开"的判据——比看面积/形状可靠得多，
    因为两个不同颜色的块粘在一起时，形状指标可能全都正常。
    """
    vals = hue_sub[region_bool]
    return mean_circular_hue(vals)


def _split_by_hue(hue_sub, region_bool, min_area, min_conc, tol=0.6):
    """把一个"色相不纯"的连通域按色相切成两块。切不动就返回 None。

    为什么选"按色相切"而不是分水岭：现场的两个块本来就是**靠颜色可分**的
    （红 ≈0/180、蓝 ≈111），色相直方图是清晰的双峰；而距离变换/分水岭依赖
    形状的连通性，对这种贴在一起的两个正方形并不稳（块边缘被高光啃过之后
    更不稳）。既然判据是颜色，就用颜色去切，失败再退回"整块丢弃"，
    不会比现在更差。

    实现上把色相旋转到"圆矢量平均 +90°"处再展开：这样跨 0/180 接线的
    双峰（比如红 175 和红 5）不会被人为劈开，真双峰（红/蓝）依然是双峰。
    阈值用 Otsu 从直方图自己找，不写死。
    """
    n = int(region_bool.sum())
    if n < 2 * min_area:
        return None
    mean_h, _conc, _n = mean_circular_hue(hue_sub[region_bool])
    vals = hue_sub[region_bool].astype(np.float32)
    # 把"圆均值"搬到 90，其余色相跟着转，得到不会跨界的一维展开
    shifted = (vals - mean_h + 90.0) % 180.0
    hist = np.bincount(np.clip(shifted.astype(np.int32), 0, 179),
                       minlength=180).astype(np.float32)
    hist = cv2.GaussianBlur(hist.reshape(-1, 1), (1, 3), 0).ravel()
    split_idx = _otsu_threshold(hist)
    if split_idx is None:
        return None                       # 直方图没有可分性 → 不是双峰
    split = float(split_idx)
    if not (1.0 <= split <= 178.0):
        return None                       # 阈值贴在两端 → 根本没有双峰

    full_shift = (hue_sub.astype(np.float32) - mean_h + 90.0) % 180.0
    hi = region_bool & (full_shift > split)
    lo = region_bool & (full_shift <= split)
    if int(hi.sum()) < min_area or int(lo.sum()) < min_area:
        return None

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    parts = []
    for part in (hi, lo):
        pm = cv2.morphologyEx(part.astype(np.uint8) * 255, cv2.MORPH_OPEN,
                              kernel)
        # 只保留最大的一块：切出来的碎屑不应该单独成块
        n_lab, lab, stats, _ = cv2.connectedComponentsWithStats(pm, 8)
        if n_lab <= 1:
            return None
        biggest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        keep = (lab == biggest)
        # 两半都必须自己"颜色很纯"，否则这次切分没有意义（例如把一块
        # 有渐变的料切成两半，各自 R 还是不高）——那就宁可不切
        if float(keep.sum()) < min_area:
            return None
        _mh, c, _c = mean_circular_hue(hue_sub[keep])
        if c < min_conc:
            return None
        parts.append(keep)
    return parts


def _find_regions(mask):
    """把掩码按连通域拆开，返回 [(region_bool, 2D 轮廓), ...]。

    为什么要按连通域组织，而不是直接对整张掩码 findContours：
    只有拿到"单个连通域的像素集合"，才能对它做色相统计（判断纯度）
    和按色相分割；对整张掩码做这些既慢又没法定位到具体哪个域。
    """
    n_lab, labels, stats, _cent = cv2.connectedComponentsWithStats(mask, 8)
    out = []
    for i in range(1, n_lab):
        x = int(stats[i, cv2.CC_STAT_LEFT])
        y = int(stats[i, cv2.CC_STAT_TOP])
        bw = int(stats[i, cv2.CC_STAT_WIDTH])
        bh = int(stats[i, cv2.CC_STAT_HEIGHT])
        region = (labels[y:y + bh, x:x + bw] == i)
        sub = (region.astype(np.uint8)) * 255
        cnts, _ = cv2.findContours(sub, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            continue
        cnt = max(cnts, key=cv2.contourArea) + (x, y)   # 还原到整图坐标
        out.append((region, cnt, (x, y)))
    return out


def _merge_fragments(regions, hue, args, tol=0.6, purity_min=0.9,
                     merge_stats=None):
    """把"同一颜色、紧贴大块"的小碎条并回它所属的块里。

    现场现象：绿块本体 R=0.878，另外 6 个 hue≈59.4~59.8、R>0.99 的细长碎条
    （58x19、57x15、44x13）——那是绿块被高光/阴影割出来的边缘。它们形状上
    过不了 aspect/fill，本来就会被丢，但白占轮廓数、拖慢后处理；
    更要紧的是"块本体被割掉一条边"会拉低它自己的 fill/solidity。

    做法：对每个"小块"，找同色相（圆距离 < tol）且**最近**的大块，
    若两者间距 <= 大块边长的 merge_dist 倍，就把两者的掩码并起来。
    为什么这么保守：这个距离必须**远小于**相邻物块的间距（现场两个物块
    之间隔着上百像素），否则会把两块独立的料粘起来——那正好是问题 1 的病。
    0.35 倍边长意味着"只并真正贴着的碎屑"。

    与问题 1 的分割不矛盾：分割处理的是"一个连通域里有两种颜色"，
    合并处理的是"同一颜色的多个连通域、且彼此贴得很近"。两者的判据
    （色相是否一致）是正交的，而且合并**只在同一色相内部**进行，
    不会把已经切开的红/蓝又并回去。
    """
    if getattr(args, 'no_merge', False) or len(regions) < 2:
        return regions
    # 只在"小块 vs 大块"之间合并：面积比小于 frag_ratio 的算碎片
    frag_ratio = 0.4
    items = []
    for region, cnt, off in regions:
        area = float(cv2.contourArea(cnt))
        _mh, conc, _n = _hue_stats(
            hue[off[1]:off[1] + region.shape[0], off[0]:off[0] + region.shape[1]],
            region)
        items.append({'region': region, 'cnt': cnt, 'off': off,
                      'area': area, 'hue': _mh, 'conc': conc, 'merged': False})
    order = sorted(range(len(items)), key=lambda i: -items[i]['area'])
    # 每个碎片最多检查这么多宿主候选：宿主按面积从大到小排，碎片真正属于的
    # 那个"大块"几乎总在最前面。加这个上限是因为**噪声帧**实测有 18 个连通域，
    # 逐对做距离变换要 115ms/帧（9fps），直接拖垮前端的 30fps。
    # 采样而不是全查，代价是"极小碎块挂在第 6 大块上"会漏并——那只是漏清理，
    # 不影响检出正确性。
    max_host_probe = 5
    for fi in order:
        frag = items[fi]
        if frag['merged']:
            continue
        parent = None
        best_gap = None
        probed = 0
        for pi in order:
            host = items[pi]
            if host is frag or host['area'] < frag['area'] / frag_ratio:
                continue                      # 只往"更大的块"上并
            # 颜色不同，绝不并。
            # 容差取得比分割判据更紧：分割问的是"这里面有没有两种颜色"，
            # 合并问的是"这两个是不是同一种颜色"，后者必须更保守。
            if ang_diff_deg(frag['hue'], host['hue']) > min(6.0, tol * 10.0):
                continue
            probed += 1
            if probed > max_host_probe:
                break
            # 两域之间的最小距离：把宿主掩码画到"覆盖两块"的公共画布上，
            # 做一次距离变换，再看碎片像素上的最小值。
            # 为什么必须用公共画布：碎片完全可能落在宿主包围盒**之外**
            # （实测那条 8x40 的碎条就在宿主 bbox 右边 6px 处），
            # 只在宿主 bbox 内取子块会算出"没有重叠"，直接漏判。
            hx, hy = host['off']
            fx, fy = frag['off']
            hr, fr = host['region'], frag['region']
            # 画布只开到"两块并集 + 一点点余量"，不要铺满整帧：
            # 距离变换的开销随画布面积走，而远分离的两块只需要知道"很大"。
            pad = max(1, int(round(host['area'] ** 0.5 * args.merge_dist))) + 2
            x0 = min(hx, fx) - pad
            y0 = min(hy, fy) - pad
            x1 = max(hx + hr.shape[1], fx + fr.shape[1]) + pad
            y1 = max(hy + hr.shape[0], fy + fr.shape[0]) + pad
            x0, y0 = max(0, x0), max(0, y0)
            x1 = min(hue.shape[1], x1)
            y1 = min(hue.shape[0], y1)
            if x1 <= x0 or y1 <= y0:
                continue
            canvas_h = np.zeros((y1 - y0, x1 - x0), np.uint8)
            canvas_h[hy - y0:hy - y0 + hr.shape[0],
                     hx - x0:hx - x0 + hr.shape[1]] = hr.astype(np.uint8)
            canvas_f = np.zeros_like(canvas_h)
            canvas_f[fy - y0:fy - y0 + fr.shape[0],
                     fx - x0:fx - x0 + fr.shape[1]] = fr.astype(np.uint8)
            # distanceTransform 默认算"每个非零点到最近零点"的距离，
            # 所以输入要取反：宿主内部值为 0（距离源），外部是到宿主的距离
            dist = cv2.distanceTransform(1 - canvas_h, cv2.DIST_L2, 3)
            if not np.any(canvas_f):
                continue
            rad = max(1, int(round(host['area'] ** 0.5 * args.merge_dist)))
            if merge_stats is not None:
                merge_stats['dist_check'] = merge_stats.get('dist_check', 0) + 1
            gap = float(dist[canvas_f > 0].min())
            if gap > rad:
                if merge_stats is not None:
                    merge_stats['dist_reject'] = \
                        merge_stats.get('dist_reject', 0) + 1
                continue                      # 距离超过阈值：不是"紧贴的碎条"
            if best_gap is None or gap < best_gap:
                best_gap, parent = gap, host
        if parent is None:
            continue
        # 走到这不只是"该合并"，也说明上面那套"距离判定"真的被执行过。
        # 之所以专门记一笔：上一轮我就是在这里翻车——距离判定里写了个
        # 少了必填参数的 copyMakeBorder，而当时的 --selftest 用例恰好
        # 从没走到这一步（碎条用例被分割分支提前接管），于是"47 项全过"
        # 却一上真机就 cv2.error 崩栈。**代码路径没被跑到，测试就是假的。**
        # 现在自检在跑完用例后会检查这些计数器，任何一条路径没被摸到就判 FAIL。
        if merge_stats is not None:
            merge_stats['merge'] = merge_stats.get('merge', 0) + 1
        # 合并：把两块画到同一张画布上（取并集），再重新取外轮廓
        hx, hy = parent['off']
        fx, fy = frag['off']
        x0 = min(hx, fx)
        y0 = min(hy, fy)
        x1 = max(hx + parent['region'].shape[1], fx + frag['region'].shape[1])
        y1 = max(hy + parent['region'].shape[0], fy + frag['region'].shape[0])
        canvas = np.zeros((y1 - y0, x1 - x0), np.uint8)
        canvas[hy - y0:hy - y0 + parent['region'].shape[0],
               hx - x0:hx - x0 + parent['region'].shape[1]] |= \
            parent['region'].astype(np.uint8)
        canvas[fy - y0:fy - y0 + frag['region'].shape[0],
               fx - x0:fx - x0 + frag['region'].shape[1]] |= \
            frag['region'].astype(np.uint8)
        cnts, _ = cv2.findContours(canvas * 255, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            continue
        # 并完之后必须复查"颜色纯度"：这是防止合并反向破坏分割的**最后一道闸**。
        # 现场真踩过：按色相把红+蓝切开后，两块各自都很纯（R=1.000），
        # 但紧接着合并逻辑看它们"颜色接近、贴得又近"，又给粘了回去，
        # 结果比不分割还糟（分割白做，块还是丢的）。
        # 现在只要并起来之后 R 掉了，就撤销这次合并。
        bh2, bw2 = canvas.shape
        hx2, hy2 = x0, y0
        merged_conc = None
        if purity_min is not None:
            full_hue = np.zeros((bh2, bw2), np.float32)
            # 从原图把这一块的色相切出来（canvas 坐标 -> 全图坐标）
            gy0, gx0 = max(0, hy2), max(0, hx2)
            gy1 = min(hue.shape[0], hy2 + bh2)
            gx1 = min(hue.shape[1], hx2 + bw2)
            if gy1 > gy0 and gx1 > gx0:
                patch = hue[gy0:gy1, gx0:gx1]
                sub = (canvas[gy0 - hy2:gy1 - hy2, gx0 - hx2:gx1 - hx2] > 0)
                _mh2, merged_conc, _n2 = mean_circular_hue(patch[sub])
        if merged_conc is not None and merged_conc < purity_min:
            if os.environ.get('CBD_MERGE_TRACE'):
                log(f'  撤销合并：并起来后 R={merged_conc:.3f} < {purity_min:.2f}')
            continue
        parent['region'] = canvas > 0
        parent['off'] = (x0, y0)
        parent['cnt'] = max(cnts, key=cv2.contourArea) + (x0, y0)
        parent['area'] = float(cv2.contourArea(parent['cnt']))
        total = max(1e-6, parent['area'] + frag['area'])
        parent['hue'] = ((parent['hue'] * parent['area']
                          + frag['hue'] * frag['area']) / total) % 180.0
        frag['merged'] = True
    return [(it['region'], it['cnt'], it['off']) for it in items
            if not it['merged']]


def _estimate_hue_offset(measurements, color_table, hue_tol, max_offset):
    """在线估计"色相整体漂移量"（白平衡/灯光变化），返回 (offset, 投票数, 说明)。

    现场现象：同一块黄料几十分钟前是 27，现在 37.7（偏了 10.7 > hue_tol 8），
    于是被 no_color_match 丢掉。这不是标定错，是灯光/自动白平衡漂了。

    为什么不用"把 --hue-tol 放大到 15"：容差一放大，黄(37.7) 到绿(62) 的
    距离只剩 24 度，两边都可能匹配上，边界会变得模糊且不可预测；
    而且容差是**对称**放开，误匹配的风险跟着一起涨。

    为什么用"整体平移"模型：白平衡/色温漂移在 OpenCV 的 H 刻度上近似是
    一个**绕色相环的常量旋转**（这是 HSV 色相的定义性质——色温改变主要
    拉伸/压缩 RGB 的相对强度，在色相环上表现为整体转一个角度），
    所以"所有块一起偏了多少"是可以估计的；而单个块的漂移没法区分
    "料变色了"和"参考值错了"。估出这个整体偏量再统一校正，
    既保住了块与块之间的**相对**色相差（黄和绿永远差 24 度，
    不会因为校正而互相靠拢），又让绝对匹配重新生效。

    估计方法：两条独立证据投票，避免单一先验把整表带偏。
      A. 场景里已经颜色很纯（集中度 >= 0.85）的块，取"离最近参考色的
         圆距离"的中位数——这些块大概率就是那几种料；
      B. 让"净距离"最小的平移量（净距离=|到最近参考色距离| 之和），
         它对"某块料还没进表"这类情况更宽容。
    两者互相验证：接近就用，差太远就放弃校正（返回 0）。
    """
def _estimate_hue_offset(measurements, color_table, hue_tol, max_offset,
                         min_samples=3, min_gain=2):
    """在线估计"色相整体漂移量"（白平衡/灯光变化），返回 (offset, 票数, 说明)。

    ⚠️ 这个功能是**保守到几乎不触发**的，因为现场证明"乱校正比不校正更危险"：
    真机上某帧出现过"估出 +5.8 并把整表挪了 5.8"，而同一时刻用原始表
    **5 块全中、实测色相与表值几乎完全吻合（偏差 ≤1.4）**。
    也就是说那 5.8 是假的——两个"纯色样本"恰好都被阴影/反光压偏了，
    而真正的漂移根本不存在。一个全局校正会把本来正确的结果推歪。

    所以现在的判据是**反事实检验**：只有当"把表挪过去"能比"不挪"
    多救回至少 min_gain 块时，才认为漂移真的存在。具体是：
      1. 只采信集中度 R >= 0.9 的样本，且至少 min_samples 块；
      2. 两种独立估计（最近色带符号距离的中位数 / 扫格子的净距离最小点）
         必须互相印证，差别超过 hue_tol/2 就拒绝；
      3. **回代计数**：用原表能匹配上几块、用挪过的表能匹配上几块，
         增益不足 min_gain 就返回 0（不校正）。
    第 3 条是关键：真漂移会让"挪过去"一次性救回好几块；
    个别被阴影压偏的样本只会给自己那一块加分，救不回别人。

    ⚠️ 保守化的代价（现场请据此判断要不要调）：如果漂移**只**影响了 1 块料
    （其余几块本来就不在容差边缘），这套判据会拒绝校正、宁可漏那一块。
    真遇到这种情况，请用 --probe-colors 重新标定，而不是靠在线校正硬凑。
    要强行关掉/打开这个功能：--no-hue-drift。
    """
    pure = [m for m in measurements if m['concentration'] >= 0.9]
    if not color_table:
        return 0.0, 0, '没有颜色参考表'
    if len(pure) < min_samples:
        return 0.0, len(pure), (f'纯色样本不足（{len(pure)} < {min_samples}），'
                                '不校正')

    def _signed_to_ref(h):
        """量测到最近参考色的带符号偏差：(参考 - 量测) 折算到 (-90, 90]。"""
        best_c = min((c for _l, c in color_table),
                     key=lambda c: ang_diff_deg(h, c))
        raw = (best_c - h) % 180.0
        return raw if raw <= 90.0 else raw - 180.0

    # 约定：shifted_center = center + offset，所以 offset = 量测 - 参考
    votes = [float(np.median([-_signed_to_ref(m['mean_hue']) for m in pure]))]
    coarse = min(4.0, max(1.0, float(hue_tol) / 2.0))
    grid = np.arange(-float(max_offset), float(max_offset) + 1e-6, coarse)
    if grid.size:
        costs = [sum(min(ang_diff_deg(m['mean_hue'], (c + d) % 180.0)
                         for _l, c in color_table) for m in pure)
                 for d in grid]
        votes.append(float(grid[int(np.argmin(costs))]))
    offset = float(np.median(votes))
    if abs(offset) > float(max_offset):
        return 0.0, len(votes), f'漂移 {offset:+.1f} 超过上限 {max_offset:.0f}，不校正'
    if len(votes) > 1 and abs(votes[0] - votes[1]) > max(3.0, hue_tol / 2.0):
        return 0.0, len(votes), (f'两种估计差太多（{votes[0]:+.1f} vs '
                                 f'{votes[1]:+.1f}），不校正')

    # ---- 反事实检验：挪过去到底能多救回几块 ----
    def _matched(table):
        n = 0
        for m in measurements:
            if m['concentration'] < 0.9:
                continue
            _lbl, d, _d2 = min(
                ((lb, ang_diff_deg(m['mean_hue'], c), 0.0)
                 for lb, c in table), key=lambda t: t[1])
            if d <= hue_tol:
                n += 1
        return n

    gain = _matched([(l, (c + offset) % 180.0) for l, c in color_table]) \
        - _matched(color_table)
    if gain < min_gain:
        return 0.0, len(votes), (f'漂移候选 {offset:+.1f} 只能多救回 {gain} 块'
                                 f'（需 >= {min_gain}），判为无漂移/样本被压偏，不校正')
    return offset, len(votes), (f'由 {len(pure)} 块纯色样本估计，'
                                f'回代可多救 {gain} 块')


def detect_blocks(image, args, color_table, hue_centers, verbose=False,
                  collect_rejected=False, coverage=None):
    """从一帧（已去畸变的）BGR 图里找出所有彩色方块。

    处理链：色度模长 → 自适应阈值 → 形态学 → 外轮廓 → 形状过滤
            → 色相矢量平均 → 最近参考色 + 置信度 → 上表面几何。

    返回 (blocks, mask, info)。blocks 的字段见下面的字典构造，
    其中 contour/extent/solidity/mean_hue 等是给调试叠加和调参用的诊断量，
    真正发到 /blocks 的只有 round_block() 挑出来的那六个。

    collect_rejected=True 时 info['rejected'] 会收下**所有**被丢掉的候选
    （默认只留前 10 条，是为了前端页面不被刷爆）——--probe-colors 需要看全，
    否则一个在列表尾部的真块会被"看不见"。
    """
    h, w = image.shape[:2]
    area_min, area_max = args.area_min, args.area_max
    if area_min is None or area_max is None:
        d_min, d_max = default_thresholds(image.shape)
        area_min = d_min if area_min is None else area_min
        area_max = d_max if area_max is None else area_max
    max_block_px = float(getattr(args, 'max_block_px', 0.0) or 0.0)

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
    # 也正因为核小，"同一个块被高光割出来的细缝"不会被闭运算补上：
    # 那种情况交给 _merge_fragments 按"同色+贴近"去并，
    # 而不是靠放大闭运算核（那会把相邻的块也粘起来）。
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # ---- 候选区域：连通域 → （色相不纯就切开）→ 合并碎条 ----
    # 为什么要按连通域组织而不是直接 findContours：只有拿到"单个连通域的
    # 像素集合"，才能对它做色相纯度判断和按色相分割。现场真故障就是
    # 红块和蓝块粘成一个连通域（R=0.445），直接 findContours 出来的是
    # 一个混合轮廓，颜色判不出来，两块一起丢。
    split_min_area = max(60.0, area_min * 0.15)
    split_min_conc = min(0.9, max(0.75, float(args.hue_consistency_min) + 0.25))
    n_regions = [0]        # 原始连通域计数（含被切分前的），闭包累加

    def _regions_of(mask_u8, depth=0, base=(0, 0)):
        """把一张（可能只是某个子区域的）掩码拆成连通域候选。

        base 是 mask_u8 左上角在**全图**里的坐标。递归切分时子掩码自带偏移，
        必须一路带下去：否则会拿子图坐标去 hue 全图采样（采到别的区域），
        返回的轮廓也停在子图坐标系里，后面对不上原图——
        表现就是"明明切开了、形状也都合格，却一块都检不出来"。
        另外把"原始连通域个数"累加到 n_regions 里，省掉外面再跑一遍
        _find_regions（纯噪声帧有几千个连通域，白跑一遍就是 45ms）。
        """
        out = []
        for region, cnt, off in _find_regions(mask_u8):
            n_regions[0] += 1
            area = float(cv2.contourArea(cnt))
            if area < split_min_area:
                continue
            bh, bw = region.shape
            gx, gy = base[0] + off[0], base[1] + off[1]
            hue_sub = hue[gy:gy + bh, gx:gx + bw]
            _mh, conc, _n = mean_circular_hue(hue_sub[region])
            # 只有"颜色明显不纯"才尝试切分。纯色域保持原样，
            # 免得在一块好料上引入无意义的分割误差。
            if (depth < 2 and not getattr(args, 'no_split', False)
                    and conc < float(args.hue_consistency_min)):
                parts = _split_by_hue(hue_sub, region, split_min_area,
                                      split_min_conc)
                if os.environ.get('CBD_SPLIT_TRACE'):
                    log(f'  [split] depth={depth} base={base} off={off} '
                        f'area={int(region.sum())} R={conc:.3f} -> '
                        + ('None' if parts is None
                           else str([int(p.sum()) for p in parts])))
                if parts is not None:
                    for part in parts:
                        sub = (region & part).astype(np.uint8) * 255
                        out.extend(_regions_of(sub, depth + 1, (gx, gy)))
                    continue
            # 轮廓只加一次 base：_find_regions 返回的轮廓已经在**子图**坐标系里，
            # 加上 base 就是全图坐标。再加第二次会整体平移（实测红+蓝切开后
            # 蓝块被推到右边 26px，bbox 里混进红像素，R 掉到 0.4 又被丢）。
            out.append((region, cnt + base, (gx, gy)))
        return out

    raw_regions = _regions_of(mask)
    regions = _merge_fragments(raw_regions, hue, args,
                               merge_stats=(coverage if coverage is not None
                                            else None))
    n_split = n_regions[0] - len(raw_regions)
    if coverage is not None:
        coverage['split_attempt'] = coverage.get('split_attempt', 0) + 1
        coverage['regions'] = coverage.get('regions', 0) + len(raw_regions)

    # ---- 第一遍：形状过滤 + 色相量测（先不判颜色）----
    candidates = []
    rejected = []
    for contour in (c for _r, c, _o in regions):
        m = _shape_metrics(contour)
        x, y, bw, bh = m['bbox']

        # 这些 if 的顺序是按"代价从低到高"排的，同时也是按"误判危害从大到小"：
        # 面积不对的直接排除，不用再算后面的几何量。
        if m['area'] < area_min or m['area'] > area_max:
            continue                      # 太小=噪点/远处杂物，太大=机械臂/桌面
        if max_block_px > 0 and max(m['rect_w'], m['rect_h']) > max_block_px:
            # 面积之外再加一道"尺寸上限"：机械臂这类大块即使被切成方块状，
            # 也不可能只有 4cm 物块那么大。两道一起用，比只看面积稳。
            continue
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
                rejected.append(_reject_reason(m, 'vertex_count'))
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
        candidates.append({
            'contour': contour, 'm': m, 'bbox': (x, y, bw, bh),
            'mean_hue': float(mean_h), 'concentration': float(concentration),
            'n_px': int(n_px), 'sub_hue': sub_hue, 'inside': inside,
        })

    # ---- 在线色相漂移校正（灯光/白平衡变了，整表跟着转）----
    hue_offset, hue_offset_votes, hue_offset_note = 0.0, 0, '未启用'
    if not getattr(args, 'no_hue_drift', False) and candidates:
        hue_offset, hue_offset_votes, hue_offset_note = _estimate_hue_offset(
            candidates, color_table, float(args.hue_tol),
            float(getattr(args, 'hue_drift_max', 15.0)))
        if coverage is not None:
            coverage['drift_checked'] = coverage.get('drift_checked', 0) + 1
            if abs(hue_offset) > 1e-6:
                coverage['drift_applied'] = coverage.get('drift_applied', 0) + 1
    # 校正量作用在**参考表**上而不是量测值上：这样 R/mean_hue 这些
    # 诊断量保持"原始量测"，而分类用"对齐后的语义"。
    # 现场调参时看到的永远是真实的 H，不会被校正悄悄改掉。
    shifted_table = [(lbl, (c + hue_offset) % 180.0) for lbl, c in color_table]

    # ---- 第二遍：颜色分类 + 几何 + 置信度 ----
    blocks = []
    for cand in candidates:
        contour = cand['contour']
        m, (x, y, bw, bh) = cand['m'], cand['bbox']
        mean_h, concentration, n_px = (cand['mean_hue'],
                                       cand['concentration'], cand['n_px'])

        # 色相集中度太低 = 这片区域颜色不纯（阴影/反光/多种颜色混在一起），
        # 直接丢。宁可漏检也不给机械臂一个错颜色的目标。
        # 注意这一步在第一遍已经尝试过"按色相切开"，能切早切了；
        # 走到这里说明切不动（例如同色块粘连），那就只能丢。
        if concentration < args.hue_consistency_min:
            rejected.append(_reject_reason(m, 'hue_inconsistent',
                                           mean_hue=round(mean_h, 1),
                                           concentration=round(concentration, 3)))
            continue

        # ---- 最近参考色（用对齐后的参考表）----
        color, dist, second_d = classify_color(mean_h, shifted_table)
        if color is None or dist > args.hue_tol:
            rejected.append(_reject_reason(m, 'no_color_match',
                                           mean_hue=round(mean_h, 1),
                                           concentration=round(concentration, 3),
                                           nearest=color,
                                           dist=round(float(dist), 1)))
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
            rejected.append(_reject_reason(m, 'low_confidence',
                                           mean_hue=round(mean_h, 1),
                                           confidence=round(confidence, 3)))
            continue

        # ---- 上表面（顶面）几何 ----
        # 剪影包含可见侧面，质心和边长都被侧面带偏；抓取要对准的是上表面中心。
        # 这里就地算一次，字段全部平铺进同一个 dict，保证老调用方
        # （前端只挑自己认识的 key）完全不受影响。
        # 关掉时退化成剪影口径——字段照样齐全，下游不用分支。
        if getattr(args, 'no_top_face', False):
            face = _silhouette_face(rect, rect_w, rect_h, m['area'])
            face['side_visible'] = False
            face['top_source'] = 'silhouette'
        else:
            face = detect_top_face(image, contour, rect, rect_w, rect_h)

        blocks.append({
            # ---- 需要交付的六项（剪影口径，保持向后兼容）----
            'color': color,
            'cx': float(rcx),
            'cy': float(rcy),
            'angle_deg': float(angle_deg),
            'size_px': float(math.sqrt(rect_w * rect_h)),
            'confidence': confidence,
            'contour': contour.reshape(-1, 2).astype(int).tolist(),
            # ---- 上表面口径（抓取该用这组；side_visible=False 时等于剪影）----
            'top_cx': face['top_cx'],
            'top_cy': face['top_cy'],
            'top_angle_deg': face['top_angle_deg'],
            'top_size_px': face['top_size_px'],
            'corners': face['corners'],
            'side_visible': bool(face['side_visible']),
            'top_source': face['top_source'],
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
            # 剪影质心和上表面质心的偏移量：侧面越可见这个值越大。
            # 现场拿它判断"斜视有多严重"，也是验证顶面检测有没有跑偏的抓手。
            'center_shift_px': float(math.hypot(face['top_cx'] - float(rcx),
                                                face['top_cy'] - float(rcy))),
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
        # candidates 是"参与判定前"的形状合格数，去重/合并后的入口数
        'candidates': int(len(candidates)),
        'regions_raw': int(len(raw_regions)),
        'regions_after_merge': int(len(regions)),
        'hue_split_gain': int(n_split),
        'hue_offset': float(hue_offset),
        'hue_offset_votes': int(hue_offset_votes),
        'hue_offset_note': hue_offset_note,
        'area_range': (float(area_min), float(area_max)),
        'rejected': rejected if collect_rejected else rejected[:10],
    }
    if verbose:
        log(f'色度阈值 {thr:.0f}（绝对下限 {floor:.0f}，场景 99.5 百分位 {p995:.0f}'
            f'{"，本次由百分位自适应抬高" if info["chroma_adaptive_raised"] else "，本次由下限决定"}）'
            f'，彩色像素 {info["colorful_pixels"]}'
            f'，连通域 {info["regions_raw"]}→合并后 {info["regions_after_merge"]}'
            f'，形状合格 {info["candidates"]}'
            f'，面积区间 {area_min:.0f}~{area_max:.0f}，命中 {len(blocks)}')
        log(f'色相漂移校正：{hue_offset:+.1f}（{hue_offset_note}）'
            + (f'，等价参考表 ' + ', '.join(f'{l}:{c:.0f}' for l, c in shifted_table)
               if abs(hue_offset) > 1e-6 else ''))
        for r in info['rejected']:
            extra = ''
            if r.get('mean_hue') is not None:
                extra = f' hue={r["mean_hue"]:.1f}'
            if r.get('concentration') is not None:
                extra += f' R={r["concentration"]:.3f}'
            log(f'  丢弃 area={r["area"]:.0f} bbox={r["bbox"]} '
                f'fill={r["fill"]:.2f} solidity={r["solidity"]:.2f} '
                f'aspect={r["aspect"]:.2f} 顶点={r["vertex_count"]}{extra} '
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

    叠加层含义（想一眼分辨"剪影"和"上表面"）：
      细白框 + 轮廓线 = 剪影（含侧面，向后兼容的老口径）
      黄色粗四边形   = 上表面（抓取应该用的口径）
      黄点           = 上表面中心；红点 = 剪影质心，两者分开说明侧面可见
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

        # 上表面：用固定颜色（青）画，和按序号变色的剪影框区分开，
        # 免得多个块叠在一起时看不出哪条线属于谁。
        side_visible = blk.get('side_visible')
        corners = blk.get('corners')
        if corners:
            quad = np.asarray(corners, np.float64).reshape(-1, 1, 2)
            cv2.polylines(canvas, [quad.astype(np.int32)], True,
                          (0, 255, 255), 2 if side_visible else 1)
            if side_visible:
                tcx, tcy = int(round(blk['top_cx'])), int(round(blk['top_cy']))
                cv2.circle(canvas, (tcx, tcy), 3, (0, 255, 255), -1)
                # 剪影质心 → 上表面中心 的位移：斜视越厉害这条线越长
                cv2.line(canvas, (cx, cy), (tcx, tcy), (0, 255, 255), 1)

        top_txt = ''
        if side_visible:
            top_txt = (f" top({blk['top_cx']:.0f},{blk['top_cy']:.0f}) "
                       f"{blk['top_angle_deg']:+.1f}deg "
                       f"{blk['top_size_px']:.0f}px")
        label = (f"{blk['color']} #{i} ({cx},{cy}) "
                 f"{blk['angle_deg']:+.1f}deg {blk['size_px']:.0f}px "
                 f"h{blk['mean_hue']:.0f} conf{blk['confidence']:.2f}{top_txt}")
        ty = max(14, int(blk['cy'] - blk['rect_h'] * 0.6) - 6)
        tx = max(2, min(int(blk['cx'] - 80), max(2, canvas.shape[1] - 300)))
        # 先黑后彩描两遍：亮背景上白字看不见，暗背景上彩色字看不清，
        # 描边能同时解决这两种情况。
        cv2.putText(canvas, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(canvas, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    color_bgr, 1, cv2.LINE_AA)

    # 图上文字一律用 ASCII：cv2.putText 的 Hershey 字体没有中文字形，
    # 中文会画成一串方框乱码（这个是在调试图上亲眼看出来的）。
    # 中文说明只放在日志/注释里——终端和注释里的中文是正常的。
    tops = sum(1 for b in blocks if b.get('side_visible'))
    head = f'blocks={len(blocks)} | topface={tops}'
    if info and info.get('chroma_threshold') is not None:
        head += (f" | chroma thr={info['chroma_threshold']:.0f}"
                 f" (p99.5 {info.get('chroma_p995', 0):.0f})"
                 f" | colorful px={info.get('colorful_pixels', 0)}")
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

    同时给出剪影口径（cx/cy/angle_deg/size_px）和**上表面口径**
    （top_cx/top_cy/top_angle_deg/top_size_px）：老的下游继续用前者，
    抓取这类"要对准朝上那个面"的下游用后者。side_visible 说明这一帧
    到底看没看见侧面——为 False 时 top_* 就是剪影的副本（退化），
    不会出现"悄悄给了一组编造的顶面参数"这种情况。
    """
    return {
        'color': blk['color'],
        'cx': round(blk['cx'], nd),
        'cy': round(blk['cy'], nd),
        'angle_deg': round(blk['angle_deg'], nd),
        'size_px': round(blk['size_px'], nd),
        'confidence': round(blk['confidence'], nd),
        'top_cx': round(blk['top_cx'], nd),
        'top_cy': round(blk['top_cy'], nd),
        'top_angle_deg': round(blk['top_angle_deg'], nd),
        'top_size_px': round(blk['top_size_px'], nd),
        'side_visible': bool(blk['side_visible']),
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
# 色表自标定（--probe-colors）
# ---------------------------------------------------------------------------

def _sanitize_label(label):
    """把任意标签串成 --colors 能安全再解析的形式（分隔符是逗号和冒号）。"""
    out = re.sub(r'[,:+\s]+', '_', str(label)).strip('_')
    return out or 'c'


def probe_color_table(candidates, hue_tol=8.0):
    """在"形状已经过关、只差颜色标签"的候选上现场量色相，生成颜色表。

    为什么要有这个：现场踩过的坑是**颜色表数值本身错了**——紫色块形状全过
    （fill=0.911 solidity=0.954），实测色相 130.6，而表里写的 150
    （OpenCV H 130 ≈ 真实色相 260°紫；150 ≈ 300°品红），距离 19.4 > hue_tol(8)，
    于是被 no_color_match 静默丢弃。这种错不该靠人肉发现。

    算法：对每个候选算圆矢量平均色相 + 集中度 R，然后按色相**排序后沿圆环
    取相邻中点**作为标签边界——不用任何预设名称、不假设有几种颜色，
    所以"第五块是第二种黄"这种模糊描述不会让它出错。
    标签统一用 color1/color2…（纯机器名，不猜"这到底算黄还是橙"）；
    想改名就改 --colors 字符串里的标签，数值不用动。

    同色相的多块料会被**合并成一个色档**：色表表达的是"颜色类别"，
    两个一模一样的色相写成两个标签既没有意义，还会让回代校验彼此抢标签
    （自检里真出现过：两块黄生成 color1:30,color2:30 然后校验互相判错）。
    真要两个同色档，自己按 --colors 的 "标签:色相+" 语法手动加。
    """
    if not candidates:
        return [], []
    ordered = sorted(candidates, key=lambda c: c['mean_hue'])
    # 先把色相几乎相同的候选合并（阈值取 hue_tol 的一半：比"能可靠区分"更近的
    # 两个色相，本来也不该是两个类）。
    merged = []
    for cand in ordered:
        if merged and ang_diff_deg(cand['mean_hue'],
                                   merged[-1]['mean_hue']) <= hue_tol / 2.0:
            keep = merged[-1]
            # 合并时按面积加权，让"大块"主导该色档的中心（大块的量测更稳）
            w0, w1 = keep['area'], cand['area']
            total = max(1e-6, w0 + w1)
            keep['mean_hue'] = ((keep['mean_hue'] * w0
                                 + cand['mean_hue'] * w1) / total) % 180.0
            keep['area'] = w0 + w1
            keep['merged'] = keep.get('merged', 1) + 1
            continue
        item = dict(cand)
        item['merged'] = 1
        merged.append(item)
    n = len(merged)
    entries = []
    for i, cand in enumerate(merged):
        if n == 1:
            left = right = (cand['mean_hue'] + 90.0) % 180.0
        else:
            prev_h = merged[(i - 1) % n]['mean_hue']
            next_h = merged[(i + 1) % n]['mean_hue']
            # 中点要按圆距离往两边各走一半，不能用 (a+b)/2 直接算
            back = ang_diff_deg(cand['mean_hue'], prev_h) / 2.0
            fwd = ang_diff_deg(cand['mean_hue'], next_h) / 2.0
            left = (cand['mean_hue'] - back) % 180.0
            right = (cand['mean_hue'] + fwd) % 180.0
        # 该色档在本帧的实际容差 = 到左右边界的较小距离，并受 --hue-tol 限制。
        # 用实测间距而不是拍一个 8：颜色接近时自动收紧（不乱标），
        # 颜色稀疏时自动放宽（不错杀）。
        span = min(ang_diff_deg(cand['mean_hue'], left),
                   ang_diff_deg(cand['mean_hue'], right))
        entries.append({
            'label': f'color{i + 1}',
            'center': float(cand['mean_hue']),
            'left': float(left),
            'right': float(right),
            'span': float(span),
            'suggested_tol': float(max(2.0, min(float(hue_tol), span))),
            'merged': int(cand['merged']),
            'source': cand,
        })
    # 标签顺序按色相，方便人眼对照
    entries.sort(key=lambda e: e['center'])
    return entries, merged


def run_probe_colors(args, hue_centers):
    """--probe-colors：量出场景里真实存在的色相，打印可粘贴的 --colors。

    先不管颜色标签，只按"形状像方块 + 色相够纯"挑候选；
    再把生成的表**回代**一遍做闭环断言——这正是自检里那条
    "参考色相表 vs 实测量测是否闭环"的现场版本，用来防止"表和人眼一致、
    但和代码里的量测不一致"这种最难查的静默失效。
    """
    if args.image:
        image = cv2.imread(args.image, cv2.IMREAD_COLOR)
        if image is None:
            log(f'读不到图片：{args.image}', 'ERROR')
            return 2
        undist = Undistorter(args.intrinsics)
        for note in undist.notes:
            log(note)
        frames = [undist.apply(image)]
    else:
        cap = open_camera(args.camera, args.width, args.height)
        if cap is None:
            log(f'打不开相机 {args.camera}（--probe-colors 也可以配 --image 用照片跑）',
                'ERROR')
            return 2
        undist = Undistorter(args.intrinsics)
        for note in undist.notes:
            log(note)
        frames = []
        for _ in range(max(1, args.probe_frames)):
            ok, frame = cap.read()
            if ok and frame is not None:
                frames.append(undist.apply(frame))
        cap.release()
        if not frames:
            log('读帧失败，无法标定色表', 'ERROR')
            return 2
        # 多帧取中值：色相统计对单帧噪声本来就稳，中值是为了消掉
        # "恰好某一帧曝光跳变"这种偶发情况，让标定值可复现。
        if len(frames) > 1:
            frames = [np.median(np.array(frames), axis=0).astype(np.uint8)]

    # 这里刻意用一个"空表"跑检测：形状过滤、色相统计、置信度全部照常执行，
    # 但没有任何颜色标签可以匹配，于是所有形状合格的候选都会落进 rejected，
    # 正好就是我们要标定的那一批。
    probe_table = [('__probe__', 0.0)]
    probe_args = argparse.Namespace(**vars(args))
    probe_args.colors = '__probe__:0'
    blocks, mask, info = detect_blocks(frames[0], probe_args, probe_table,
                                       hue_centers, verbose=False,
                                       collect_rejected=True)

    # 候选 = 所有"形状过关、只因没有匹配颜色而被丢"的轮廓。
    # 另外把因为"色相不纯"被丢的也列出来（标成低集中度），
    # 但不参与色表生成——颜色不纯的块定不出可信的中心。
    raw = [r for r in info['rejected'] if r['reason'] == 'no_color_match']
    impure = [r for r in info['rejected'] if r['reason'] == 'hue_inconsistent']

    # mean_hue 在 rejected 记录里（_reject_reason 传了）。
    # 注意一个细节：**不能**用 n_px/coverage 之类没被带出来的量，
    # 所以这里只依赖 mean_hue / concentration / area / bbox / fill 这些标量。
    candidates = []
    for r in raw:
        if r.get('mean_hue') is None:
            continue
        candidates.append({
            'mean_hue': float(r['mean_hue']) % 180.0,
            'concentration': float(r.get('concentration', float('nan')))
            if r.get('concentration') is not None else float('nan'),
            'area': float(r['area']),
            'fill': float(r['fill']),
            'solidity': float(r['solidity']),
            'bbox': tuple(r['bbox']),
        })

    log(f'=== 色表标定：形状合格候选 {len(candidates)} 个'
        f'（另有 {len(impure)} 个因色相不纯被排除）===')
    if not candidates:
        log('没有找到任何"形状合格"的彩色候选。'
            '先确认画面里有料、掩码不为空（可加 --out 看调试图），'
            '必要时放宽 --fill-min / --solidity-min', 'WARN')
        return 1

    entries, merged_list = probe_color_table(candidates, hue_tol=args.hue_tol)
    for entry in entries:
        c = entry['source']
        cx = c['bbox'][0] + c['bbox'][2] / 2.0
        cy = c['bbox'][1] + c['bbox'][3] / 2.0
        conc = c['concentration']
        conc_txt = 'n/a' if conc != conc else f'{conc:.3f}'
        merged_txt = (f"  合并了 {entry['merged']} 块" if entry['merged'] > 1 else '')
        log(f"  {entry['label']}: hue={c['mean_hue']:6.2f}  R={conc_txt:>5s}  "
            f"area={c['area']:6.0f}  fill={c['fill']:.3f}  "
            f"sol={c['solidity']:.3f}  质心≈({cx:.0f},{cy:.0f})  "
            f"边界=[{entry['left']:.1f},{entry['right']:.1f}] "
            f"建议容差={entry['suggested_tol']:.1f}{merged_txt}")

    colors_str = ','.join(f"{e['label']}:{e['center']:.0f}" for e in entries)
    log('可直接粘贴的颜色表：')
    print(f'--colors "{colors_str}"')

    # ---- 闭环自检：用生成的表回代，每个候选都必须匹配上自己的标签 ----
    back_table = [(e['label'], e['center']) for e in entries]
    log('=== 回代闭环校验（用生成的表重跑，确认每个候选都吃到自己的标签）===')
    bad = 0
    for e in entries:
        label, dist, second = classify_color(e['source']['mean_hue'], back_table)
        ok = (label == e['label'] and dist <= args.hue_tol)
        # 也要检查"离次近标签太近"：那样标签会随光照漂移而跳变
        ambiguous = second < args.hue_tol and second < 1e8
        flag = 'OK' if (ok and not ambiguous) else 'FAIL'
        if flag == 'FAIL':
            bad += 1
        log(f"  [{flag}] hue={e['source']['mean_hue']:6.2f} → 标签 {label} "
            f"(期望 {e['label']}) 距离={dist:.1f} 次近={second if second < 1e8 else float('nan'):.1f}"
            + ('  ← 与邻近标签区分度不足，建议合并或收紧容差' if ambiguous else ''))

    if bad:
        log(f'闭环校验有 {bad} 项不通过：这张表直接拿去用会误判，'
            '请检查是否有两块颜色过于接近的料', 'WARN')
        return 1
    log(f'闭环校验通过 ✅ 共 {len(entries)} 个色档；'
        f'把上面那行 --colors 贴给 color_block_detect.py / block_live_gui.py 即可')
    if args.out:
        imwrite(args.out, annotate(frames[0], blocks, mask, info))
        log(f'标定标注图：{args.out}')
    return 0


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


def _draw_rect(canvas, cx, cy, box, hsv_color, angle=0.0, ss=SS,
               hue_shift=0.0):
    """在 box x box 的方形 ROI 里画一个（可旋转的）实心方块并贴到 canvas 上。

    hsv_color=(H,S,V)；canvas 坐标是 1x 尺度。
    hue_shift 用来模拟"白平衡/灯光漂移"：在**画的时候就**把色相转一个角度，
    S/V 不动，所以色度依然很高——这才是"同一块料被拍成了另一个色相"。
    （如果在画好之后整体平移 H 通道，那等于换一种颜色画上去：蓝 111 变 99
    就成了青色，色度大幅下降。拿它验漂移会得出错误结论——我第一版就是这么
    写的，结果"蓝块消失"，查了半天才发现是用例本身不成立。）
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

    h_draw = int(round((float(hsv_color[0]) + float(hue_shift)) % 180.0))
    bgr = cv2.cvtColor(np.uint8([[[h_draw, int(hsv_color[1]),
                                    int(hsv_color[2])]]]),
                       cv2.COLOR_HSV2BGR)[0, 0]
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


def _draw_cube(canvas, top_cx, top_cy, side, hsv_color, angle=0.0,
               view=(0.30, 0.38)):
    """画一个"斜视的立方体块"：上表面 + 可见侧面，返回上表面四角真值。

    为什么要专门造这么一个合成对象：本项目的核心痛点就是"剪影包含侧面"，
    而 _draw_rect 画的是平铺方块（上表面==剪影），**根本测不出**上表面检测
    到底有没有用。这里用最朴素的斜投影建模：
      上表面 = 边长 side、绕画面法线转 angle 的正方形，中心 (top_cx, top_cy)；
      底面   = 上表面整体平移 view*side（斜投影下立方体的竖直棱都平行且等长）；
      可见侧面 = 底面向 +x/+y 方向张出来的那两个四边形。
    和真实透视投影的差别是二阶的（正交 vs 透视），对本检测要验证的
    "顶面近边在哪"没有影响，但足以让剪影质心明显低于顶面中心。

    顶面颜色用正对的亮色，侧面用同色相压暗（0.78 倍 V）——这是 EVA 块最典型的
    表现（顶面正对镜头/光源，侧面斜对，一般更暗），也是检测器用来找近边的先验。
    """
    h_img, w_img = canvas.shape[:2]
    th = math.radians(angle)
    half = side / 2.0
    loc = np.array([[-half, -half], [half, -half], [half, half], [-half, half]],
                   np.float64)
    rot = np.array([[math.cos(th), -math.sin(th)],
                    [math.sin(th), math.cos(th)]])
    top = loc @ rot.T + np.array([top_cx, top_cy], np.float64)
    dx, dy = view[0] * side, view[1] * side
    base = top + np.array([dx, dy], np.float64)

    def _fill(quad, hsv):
        pts = np.round(quad).astype(np.int32)
        if pts[:, 0].min() < 0 or pts[:, 1].min() < 0 \
                or pts[:, 0].max() >= w_img or pts[:, 1].max() >= h_img:
            raise ValueError('合成立方体超出画面范围，检查真值坐标')
        bgr = cv2.cvtColor(np.uint8([[[int(hsv[0]), int(hsv[1]), int(hsv[2])]]]),
                           cv2.COLOR_HSV2BGR)[0, 0]
        cv2.fillPoly(canvas, [pts], (int(bgr[0]), int(bgr[1]), int(bgr[2])))

    h, s, v = (int(hsv_color[0]), int(hsv_color[1]), int(hsv_color[2]))
    # 侧面压暗到 0.70：实测 EVA 块顶面正对镜头/光源，侧面通常暗 20%~40%。
    # 刻意**不能**调到和桌面灰度一样（约 0.78 时正好撞上）——那样
    # "侧面→背景"会和"顶面→侧面"一样强，属于物理上真的分不出来的情况，
    # 应该由下面的"退化"分支处理，而不是拿来当常规用例。
    side_hsv = (h, s, max(20, int(v * 0.70)))
    # 先画侧面再画顶面：顶面在上，覆盖优先级更高（也更符合"上面盖住下面"的直觉）
    if dy > 0.5:
        _fill(np.array([top[3], top[2], base[2], base[3]]), side_hsv)  # 前面
    if dx > 0.5:
        _fill(np.array([top[1], top[2], base[2], base[1]]), side_hsv)  # 右侧
    _fill(top, (h, s, v))
    return top


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
        ['purple', 160, 340, 46, 130, 215, 210, -15.0, 0.0],
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
      A2. **跨 0/180 环绕边界**的色相量测与分类（现场红色块色相 175.9 就贴在这条线上）；
      B. 五个已知方块全部找到，颜色标签正确（含重复的黄）；
      C. 质心 / 朝向 / 边长的量测误差在容差内；
      D. 干扰物（细长反光条、低色度灰块、表外的青色块）都不被误报；
      E. 输出契约（下游 JSON 字段与取值范围）；
      F. 上表面检测：斜视立方体上"顶面中心/朝向/边长"必须比剪影更接近真值，
         并且正上方俯视（侧面不可见）时必须优雅退化成剪影；
      G. --probe-colors 的色表生成与回代闭环。
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
            # 探针色相直接用**内置参考中心**：这样"表 ↔ 量测"闭环检查
            # 在默认表被重新标定（改 DEFAULT_HUE_CENTERS）后依然成立，
            # 不需要在自检里再抄一份数字。
            probe_h = DEFAULT_HUE_CENTERS.get(label)
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

    # ---- A2. 0/180 环绕边界 ----
    # 现场数据：红色块色相 175.9（标准色相 351.8°，贴着接线）。
    # 如果 ang_diff_deg 写成朴素减法，这里会算出 175.9 而不是 4.1，
    # 红色块就会被 no_color_match 静默丢弃——这是实机真出过的故障。
    check(abs(ang_diff_deg(175.9, 0.0) - 4.1) < 0.05,
          f'环绕距离：175.9 到 0 的圆距离 = {ang_diff_deg(175.9, 0.0):.2f}（应为 4.1）',
          f'环绕距离算错了：175.9→0 得到 {ang_diff_deg(175.9, 0.0):.2f}，应为 4.1')
    check(abs(ang_diff_deg(179.0, 1.0) - 2.0) < 0.05
          and abs(ang_diff_deg(0.0, 90.0) - 90.0) < 0.05
          and abs(ang_diff_deg(10.0, 20.0) - 10.0) < 0.05,
          '环绕距离在 179/1、0/90（最远）、10/20（普通）三处都对',
          '环绕距离在边界或中点算错了')
    # 圆矢量平均：绕在接线两侧的取值（178 与 2）必须平均成 0 附近，不能是 90
    wrap_h, wrap_r, wrap_n = mean_circular_hue([178.0, 179.0, 0.0, 1.0, 2.0])
    check(ang_diff_deg(wrap_h, 0.0) <= 2.0 and wrap_r > 0.9,
          f'圆矢量平均跨环绕：均值 {wrap_h:.1f}（应在 0 附近）、集中度 {wrap_r:.3f}',
          f'圆矢量平均跨环绕失败：均值 {wrap_h:.1f}（应该≈0）、集中度 {wrap_r:.3f}')
    # 端到端：真的画一块"色相 179"的红块，走完整检测链，必须还能标成 red。
    # cv2 的 HSV→BGR 在 H=179 会给出暗红，所以这里直接指定 BGR 深红来造这个像素。
    wrap_img = _blank_frame()
    wrap_img[240 - 24:240 + 24, 320 - 24:320 + 24] = (0, 0, 255)
    wrap_blocks, _wm, _wi = detect_blocks(
        wrap_img, probe_args, [('red', 0.0), ('yellow', 30.0)], {},
        verbose=False)
    if not wrap_blocks:
        failures.append('  [FAIL] 环绕边界：色相≈179 的深红块没有被检出')
    else:
        wb = wrap_blocks[0]
        check(wb['color'] == 'red' and ang_diff_deg(wb['mean_hue'], 0.0) <= args.hue_tol,
              f'环绕边界端到端：色相≈179 的深红块判成 {wb["color"]}'
              f'（mean_hue={wb["mean_hue"]:.1f}，距 red:0 仅 '
              f'{ang_diff_deg(wb["mean_hue"], 0.0):.1f}）',
              f'环绕边界端到端失败：色相 {wb["mean_hue"]:.1f} 被标成 {wb["color"]}')

    # ---- B/C/D. 完整检测 ----
    # coverage 用来统计"关键代码路径有没有真的被跑到"。
    # 上一轮我交出的版本在这里翻车：合并逻辑里有个少了必填参数的
    # copyMakeBorder，但当时的自检用例从没走到那段距离判定，
    # 于是自检"全过"而一上真机就崩。**没被跑到的路径，测试等于没写。**
    coverage = {}
    blocks, mask, info = detect_blocks(frame, probe_args, color_table,
                                       hue_centers, verbose=True,
                                       coverage=coverage)
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
    keys = {'color', 'cx', 'cy', 'angle_deg', 'size_px', 'confidence',
            'top_cx', 'top_cy', 'top_angle_deg', 'top_size_px', 'side_visible'}
    check(isinstance(payload.get('stamp'), float)
          and all(set(b) == keys for b in payload['blocks']),
          f'JSON 字段与约定一致（stamp + {len(keys)} 项，含上表面口径）',
          f'JSON 字段不符合约定：{payload["blocks"][:1]}')
    h_img, w_img = frame.shape[:2]
    check(all(0.0 <= b['confidence'] <= 1.0 for b in blocks)
          and all(0.0 <= b['cx'] < w_img and 0.0 <= b['cy'] < h_img for b in blocks)
          and all(-45.0 <= b['angle_deg'] < 45.0 for b in blocks),
          '置信度∈[0,1]、质心在画面内、朝向∈[-45,45)：取值范围正确',
          '取值越界：' + str([(round(b['confidence'], 2), round(b['cx'], 1),
                             round(b['cy'], 1), round(b['angle_deg'], 1))
                            for b in blocks]))

    # ---- F. 上表面（顶面）检测 ----
    # 造一个斜视的立方体：剪影必然包含前面和右侧面，于是剪影质心低于顶面中心。
    # 这里要验证的是"顶面检测给出的中心比剪影更接近真值"，而不是"能画出四个点"。
    cube = _blank_frame()
    cube_top_c, cube_top_r = (300.0, 190.0)
    cube_side, cube_angle, cube_view = 56.0, 18.0, (0.30, 0.38)
    cube_hsv = (30, 225, 225)
    truth = _draw_cube(cube, cube_top_c, cube_top_r, cube_side, cube_hsv,
                       cube_angle, cube_view)
    truth_center = (float(truth[:, 0].mean()), float(truth[:, 1].mean()))
    cube_blocks, _cm, _ci = detect_blocks(cube, probe_args,
                                          [('yellow', 30.0)], {}, verbose=False)
    if not cube_blocks:
        failures.append('  [FAIL] 上表面：斜视立方体没有被检出')
    else:
        cb = cube_blocks[0]
        d_top = math.hypot(cb['top_cx'] - truth_center[0],
                           cb['top_cy'] - truth_center[1])
        d_sil = math.hypot(cb['cx'] - truth_center[0],
                           cb['cy'] - truth_center[1])
        check(cb['side_visible'],
              f'斜视立方体：判定侧面可见（top_source={cb["top_source"]}，'
              f'剪影↔顶面偏移 {cb["center_shift_px"]:.1f}px）',
              '斜视立方体：side_visible 应为 True，检测没有认出顶面')
        # 2.7px 量级的残余误差来自像素量化（56px 的方块，1px 就是 1.8%），
        # 不是算法问题；所以容差取 4px，同时要求它明显优于剪影口径。
        check(d_top <= 4.0,
              f'顶面中心误差 {d_top:.2f}px（真值 ({truth_center[0]:.1f},'
              f'{truth_center[1]:.1f})）',
              f'顶面中心误差过大：{d_top:.2f}px，实测 ({cb["top_cx"]:.1f},'
              f'{cb["top_cy"]:.1f}) vs 真值 ({truth_center[0]:.1f},'
              f'{truth_center[1]:.1f})')
        # 这一条才是"上表面检测有没有意义"的核心：
        # 剪影质心被侧面拖下去了，顶面中心必须明显更接近真值
        check(d_sil > d_top + 3.0,
              f'剪影质心确实被侧面拖偏（剪影误差 {d_sil:.1f}px > '
              f'顶面误差 {d_top:.1f}px），上表面检测有意义',
              f'剪影误差 {d_sil:.2f}px 并不比顶面误差 {d_top:.2f}px 大，'
              '合成立方体的侧面没有真的进剪影')
        d_ang_top = abs(cb['top_angle_deg'] - cube_angle) % 90.0
        d_ang_top = min(d_ang_top, 90.0 - d_ang_top)
        check(d_ang_top <= 3.0,
              f'顶面朝向 {cb["top_angle_deg"]:+.1f}°（真值 {cube_angle:+.1f}°，'
              f'差 {d_ang_top:.1f}°）',
              f'顶面朝向偏差过大：{cb["top_angle_deg"]:+.1f} vs {cube_angle:+.1f}')
        check(abs(cb['top_size_px'] - cube_side) <= 4.0,
              f'顶面边长 {cb["top_size_px"]:.1f}px（真值 {cube_side:.0f}px；'
              f'剪影 {cb["size_px"]:.1f}px 混进了侧面）',
              f'顶面边长偏差过大：{cb["top_size_px"]:.1f} vs {cube_side:.0f}')
        # 四边形必须自洽：4 个角、凸、面积和中心匹配
        corners = np.asarray(cb['corners'], np.float64)
        quad_ok = (corners.shape == (4, 2)
                   and cv2.isContourConvex(corners.astype(np.float32))
                   and _polygon_area(corners) > 0.35 * (cube_side ** 2))
        check(quad_ok,
              f'顶面四角几何自洽（4 点凸四边形，面积 '
              f'{_polygon_area(corners):.0f}px² vs 真值 {cube_side ** 2:.0f}px²）',
              f'顶面四角不自洽：{corners.tolist()}')

    # 正上方俯视：侧面不可见 → 必须优雅退化成剪影，不许编造角点
    flat = _blank_frame()
    _draw_rect(flat, 320, 240, 60, cube_hsv, 0.0)
    flat_blocks, _fm, _fi = detect_blocks(flat, probe_args,
                                          [('yellow', 30.0)], {}, verbose=False)
    if not flat_blocks:
        failures.append('  [FAIL] 上表面退化：俯视方块没有被检出')
    else:
        fb = flat_blocks[0]
        check(not fb['side_visible'] and fb['top_source'] == 'silhouette',
              f'俯视（侧面不可见）优雅退化为剪影：side_visible=False，'
              f'top_source={fb["top_source"]}',
              f'俯视时没有退化：side_visible={fb["side_visible"]}，'
              f'top_source={fb["top_source"]}')
        check(abs(fb['top_cx'] - fb['cx']) < 1.0
              and abs(fb['top_cy'] - fb['cy']) < 1.0,
              '退化后 top_* 与剪影口径一致（下游可以无脑用 top_*）',
              f'退化后 top_* 与剪影不一致：({fb["top_cx"]:.1f},{fb["top_cy"]:.1f})'
              f' vs ({fb["cx"]:.1f},{fb["cy"]:.1f})')

    # ---- H. 粘连分割（现场故障：红块+蓝块粘成一个连通域，R=0.445）----
    # 合成两块**互相接触**的不同颜色方块：红在左、蓝在右，各自与对方在掩码里
    # 连成一体（用中心距 42px < 边长 44px 保证真的粘上）。
    touch = _blank_frame()
    _draw_rect(touch, 280, 240, 44, (0, 235, 220))
    _draw_rect(touch, 322, 240, 44, (111, 225, 215))
    glue_lab = cv2.cvtColor(touch, cv2.COLOR_BGR2LAB)
    glue_ch = cv2.magnitude(glue_lab[:, :, 1].astype(np.float32) - 128.0,
                            glue_lab[:, :, 2].astype(np.float32) - 128.0)
    glue_thr = max(26.0, min(0.45 * float(np.percentile(glue_ch, 99.5)), 65.0))
    glue_mask = cv2.inRange(glue_ch, glue_thr, 255.0)
    glue_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    glue_mask = cv2.morphologyEx(glue_mask, cv2.MORPH_OPEN, glue_k)
    glue_mask = cv2.morphologyEx(glue_mask, cv2.MORPH_CLOSE, glue_k)
    n_glue, _gl, gstats, _gc = cv2.connectedComponentsWithStats(glue_mask, 8)
    glue_big = [int(gstats[i, cv2.CC_STAT_AREA]) for i in range(1, n_glue)
                if gstats[i, cv2.CC_STAT_AREA] > 200]
    check(len(glue_big) == 1,
          f'粘连用例成立：两块在掩码里连成 1 个连通域（{glue_big}）',
          f'粘连用例不成立：掩码里是 {len(glue_big)} 个连通域 {glue_big}，'
          '没测到分割')
    glue_table = [('red', 0.0), ('blue', 111.0)]
    glue_blocks, _gm, glue_info = detect_blocks(touch, probe_args, glue_table,
                                                {}, verbose=False)
    got = {b['color']: b for b in glue_blocks}
    check(set(got) == {'red', 'blue'},
          f'粘连分割：一个连通域拆出 {sorted(got)} 两块',
          f'粘连分割失败：只拆出 {sorted(got)}（红+蓝都该在），'
          f'拆分增益={glue_info["hue_split_gain"]}')
    if set(got) == {'red', 'blue'}:
        check(all(b['hue_concentration'] >= 0.9 for b in glue_blocks),
              '拆分后两块色相集中度都 >= 0.9（'
              + ', '.join(f'{c}:{b["hue_concentration"]:.3f}'
                          for c, b in sorted(got.items())) + '）',
              '拆分后集中度不够高：'
              + str({c: round(b['hue_concentration'], 3)
                     for c, b in got.items()}))
        check(got['red']['cx'] < got['blue']['cx'],
              f'左右顺序正确（红 {got["red"]["cx"]:.0f} < 蓝 '
              f'{got["blue"]["cx"]:.0f}）',
              '拆分后左右位置反了：'
              f'红 {got["red"]["cx"]:.0f}, 蓝 {got["blue"]["cx"]:.0f}')
        check(glue_info['hue_split_gain'] >= 1,
              f'记录了拆分增益 {glue_info["hue_split_gain"]}',
              '拆分增益没有记录')

    # ---- I. 色相漂移校正（现场：同一块黄料 27 → 37.7，超 hue_tol 被丢）----
    # 用 hue_shift 在**绘制时**转角，模拟灯光/白平衡让整幅画面的色相整体旋转。
    drift_table = [('yellow', 27.0), ('green', 62.0), ('blue', 111.0),
                   ('purple', 130.0)]

    def _drift_frame(shift):
        img = _blank_frame()
        for _cx, _cy, _h in ((150, 150, 27), (320, 150, 62),
                             (480, 150, 111), (220, 330, 130)):
            _draw_rect(img, _cx, _cy, 52, (_h, 220, 215), hue_shift=shift)
        return img

    # 漂移量取 ±10 度：再大就会把高饱和的蓝推到 RGB 色域边角上
    # （H=111 的蓝 chroma≈78，转到 H=99 只剩 40），那时掩码阈值先把它滤掉了，
    # 测的就不再是"颜色判据"而是"色域边界"——那是另一个问题，不该混进来。
    # 这一组用例把 --chroma-min 钉在 44，冻住色度自适应阈值。
    # 原因：合成图里的深紫（H=130 S=215）chroma 高达 110，会把自适应阈值
    # （0.45×p99.5）抬到 49；而蓝色被旋转到 H=101 时 chroma 掉到 43.6
    # （靠近 RGB 色域边角），于是**掩码阶段**就把蓝块滤掉了。
    # 那属于"色域 + 色度阈值"的问题，会把本条要验的"色相漂移"结论搞混。
    # 钉住下限之后，这里测的就是纯粹的"整表旋转 → 平移校正"这条链路；
    # 色度自适应阈值本身由主自检（B 段五块 + D 段干扰物）负责覆盖。
    drift_args = argparse.Namespace(**vars(probe_args))
    drift_args.chroma_min = 44.0

    for drift in (10, -10):
        drift_frame = _drift_frame(drift)
        auto_blocks, _dm, auto_info = detect_blocks(drift_frame, drift_args,
                                                    drift_table, {},
                                                    verbose=False)
        no_drift_args = argparse.Namespace(**vars(drift_args))
        no_drift_args.no_hue_drift = True
        off_blocks, _om, _oi = detect_blocks(drift_frame, no_drift_args,
                                             drift_table, {}, verbose=False)
        # 断言"恢复了绝大多数"而不是"四块全回来"：蓝色在 H=111 附近本来就贴着
        # RGB 色域的青色角，往 -10° 一转 chroma 掉到 37 左右，会被**掩码阈值**
        # 先滤掉（那属于色域/阈值问题，不是色相判据问题）。这条用例要证明的是
        # "整表漂移能被估出来并平移回去"，所以用"恢复数"衡量更贴切，
        # 也避免把色域边界问题伪装成色相问题。
        check(len(auto_blocks) >= 3 and len(auto_blocks) > len(off_blocks),
              f'漂移 {drift:+d}° 时自动校正 offset='
              f'{auto_info["hue_offset"]:+.1f}，恢复 '
              f'{len(auto_blocks)}/4 块（关掉校正只剩 {len(off_blocks)} 块）',
              f'漂移 {drift:+d}° 时校正没起作用：开着 {len(auto_blocks)} 块、'
              f'关着 {len(off_blocks)} 块')
        check(abs(auto_info['hue_offset'] - drift) <= 4.0,
              f'估出的漂移 {auto_info["hue_offset"]:+.1f} 与真值 {drift:+d} 相符',
              f'估出的漂移 {auto_info["hue_offset"]:+.1f} 偏离真值 {drift:+d} 太多')
    # 没漂移时必须**不要**乱校正（否则等于把好表挪歪）
    clean_blocks, _cm3, clean_info = detect_blocks(_drift_frame(0), drift_args,
                                                   drift_table, {},
                                                   verbose=False)
    check(abs(clean_info['hue_offset']) <= 3.0 and len(clean_blocks) == 4,
          f'无漂移时校正量接近 0（{clean_info["hue_offset"]:+.1f}），4 块照常检出',
          f'无漂移时校正量 {clean_info["hue_offset"]:+.1f} 偏大，'
          f'会把好表挪歪（检出 {len(clean_blocks)} 块）')

    # ---- J. 碎条合并（现场：绿块被高光割出 6 条 hue≈59.5 的细缝）----
    frag_img = _blank_frame()
    _draw_rect(frag_img, 300, 240, 56, (62, 230, 220))
    # 贴一条 40x8 的细长同色条（间距 6px：形态学闭运算补不上，但属于"紧贴"）
    _frag_bgr = cv2.cvtColor(np.uint8([[[62, 210, 200]]]),
                             cv2.COLOR_HSV2BGR)[0, 0]
    frag_img[240 - 20:240 + 20, 300 + 28 + 6:300 + 28 + 6 + 8] = _frag_bgr
    frag_table = [('green', 62.0)]
    merge_blocks, _fmg, merge_info = detect_blocks(frag_img, probe_args,
                                                   frag_table, {}, verbose=False,
                                                   coverage=coverage)
    no_merge_args = argparse.Namespace(**vars(probe_args))
    no_merge_args.no_merge = True
    _nb, _nmg, no_merge_info = detect_blocks(frag_img, no_merge_args,
                                             frag_table, {}, verbose=False)
    check(merge_info['regions_after_merge'] < merge_info['regions_raw']
          and no_merge_info['regions_after_merge']
          == no_merge_info['regions_raw'],
          f'碎条被并回大块（连通域 {merge_info["regions_raw"]}→'
          f'{merge_info["regions_after_merge"]}；关掉合并则保持 '
          f'{no_merge_info["regions_after_merge"]}）',
          f'碎条没有被合并：开着 {merge_info["regions_raw"]}→'
          f'{merge_info["regions_after_merge"]}，'
          f'关着 {no_merge_info["regions_raw"]}→'
          f'{no_merge_info["regions_after_merge"]}')
    check(len(merge_blocks) == 1 and merge_blocks[0]['color'] == 'green',
          f'合并后仍是 1 块绿（质心 {merge_blocks[0]["cx"]:.0f},'
          f'{merge_blocks[0]["cy"]:.0f}）' if merge_blocks else '合并后无检出',
          f'合并后检出异常：{[b["color"] for b in merge_blocks]}')
    # 反向保护 A：**小而远**的同色碎块必须被"距离闸"拒绝。
    # 注意这里必须让碎块比宿主**明显小**（area 比 < frag_ratio），
    # 否则循环会先在"只往更大的块上并"那一句 continue 掉，
    # 距离判定根本不会执行——断言就变成了空的（这个坑我踩过一次：
    # 原来用两个等大的块测"不误并"，它一直是靠面积比提前跳过而"通过"的）。
    far_img = _blank_frame()
    _draw_rect(far_img, 300, 240, 50, (62, 230, 220))
    far_bgr = cv2.cvtColor(np.uint8([[[62, 210, 200]]]),
                           cv2.COLOR_HSV2BGR)[0, 0]
    far_img[236:244, 160:200] = far_bgr          # 40x8 的小块，离宿主约 90px
    far_cov = {}
    far_blocks, _frm, far_info = detect_blocks(far_img, probe_args, frag_table,
                                               {}, verbose=False,
                                               coverage=far_cov)
    check(far_cov.get('dist_reject', 0) >= 1,
          f'"小而远"的同色碎块被距离闸拒绝（距离判定执行 '
          f'{far_cov.get("dist_check", 0)} 次、拒绝 '
          f'{far_cov.get("dist_reject", 0)} 次）',
          '距离闸没有被执行到——"不会把远处同色料并进来"这条结论没有依据'
          f'（dist_check={far_cov.get("dist_check", 0)}, '
          f'dist_reject={far_cov.get("dist_reject", 0)}）')
    check(far_info['regions_after_merge'] == far_info['regions_raw'],
          f'远处同色碎块没有被误并（连通域保持 {far_info["regions_raw"]}）',
          f'远处的同色小碎块被误并了：{far_info["regions_raw"]}→'
          f'{far_info["regions_after_merge"]}')

    # 反向保护 B：两个等大的同色块**不许**并成一个
    apart_img = _blank_frame()
    _draw_rect(apart_img, 200, 240, 50, (62, 230, 220))
    _draw_rect(apart_img, 300, 240, 50, (62, 230, 220))
    apart_cov = {}
    apart_blocks, _am, apart_info = detect_blocks(apart_img, probe_args,
                                                  frag_table, {}, verbose=False,
                                                  coverage=apart_cov)
    check(len(apart_blocks) == 2
          and apart_info['regions_after_merge'] == apart_info['regions_raw'],
          f'两个同色块相距 50px 时没有被误并（仍 {len(apart_blocks)} 块）',
          f'过度合并：两个同色块被并成了 {len(apart_blocks)} 块')

    # ---- K. 大块（机械臂）护栏 ----
    # 现场：黄色机械臂本体 9804px²、hue 与黄块一致。默认面积上限 + 尺寸上限
    # 必须把它挡住，否则机械臂姿态一变就可能被当成黄块。
    arm_img = _blank_frame()
    _draw_rect(arm_img, 320, 240, 130, (27, 200, 200))
    arm_blocks, _arm_m, _arm_i = detect_blocks(arm_img, probe_args,
                                               [('yellow', 27.0)], {},
                                               verbose=False)
    arm_area = 130 * 130
    check(not arm_blocks,
          f'130px 见方的大黄块被挡住（面积 {arm_area}px² > 默认上限 '
          f'{default_thresholds(arm_img.shape)[1]:.0f}px²）',
          f'大块没被挡住：检出了 {[(b["color"], b["area"]) for b in arm_blocks]}，'
          '机械臂姿态变化时有误判风险')

    # ---- L. 覆盖率守卫：关键路径必须真的被跑到 ----
    # 这条是给"测试通过但代码是坏的"兜底的。数字阈值都只是"这条路径至少
    # 被走过"的下限（不是精确计数），任何一条为 0 就说明自检有盲区，
    # 必须补用例而不是调阈值。
    check(coverage.get('dist_check', 0) >= 1,
          f'合并的距离判定被跑到（{coverage.get("dist_check", 0)} 次）',
          '合并的距离判定一次都没执行——这条路径没有测试覆盖，'
          '里面的任何错误都会漏到真机（上一轮就是这么崩的）')
    check(coverage.get('merge', 0) >= 1,
          f'合并动作真的执行过（{coverage.get("merge", 0)} 次）',
          '合并动作一次都没执行——自检里的"合并"断言其实是空的')
    check(coverage.get('dist_reject', 0) + far_cov.get('dist_reject', 0) >= 1,
          f'合并的"距离太远就拒绝"分支被跑到（'
          f'{coverage.get("dist_reject", 0) + far_cov.get("dist_reject", 0)} 次）',
          '合并的拒绝分支没被执行——无法确认"不会过度合并"')
    check(apart_cov.get('regions', 0) >= 2,
          '同色等大块用例产出了多个区域（结论才有意义）',
          '同色等大块用例区域数不足，"不误并"的结论不成立')
    check(coverage.get('split_attempt', 0) >= 1
          and coverage.get('regions', 0) >= 1,
          f'分割/区域提取被跑到（区域数累计 {coverage.get("regions", 0)}）',
          '区域提取一次都没执行')
    check(coverage.get('drift_checked', 0) >= 1,
          f'色相漂移估计被跑到（检查 {coverage.get("drift_checked", 0)} 次）',
          '色相漂移估计一次都没执行')

    # ---- G. 色表自标定（--probe-colors）----
    # 复现现场那个真故障：色表里**根本没有紫色**（现场是紫色写成 150 后距离超限，
    # 效果等价于"表里缺紫"）。此时紫块形状全过、只因颜色不匹配被判 no_color_match，
    # 正是 --probe-colors 要捞回来的对象。
    probe_table_partial = [('red', 0.0), ('yellow', 30.0), ('green', 60.0)]
    _wb, _wm2, wrong_info = detect_blocks(frame, probe_args,
                                          probe_table_partial, {},
                                          verbose=False, collect_rejected=True)
    probe_cands = [r for r in wrong_info['rejected']
                   if r['reason'] == 'no_color_match'
                   and r.get('mean_hue') is not None]
    entries, _ordered = probe_color_table(
        [{'mean_hue': float(r['mean_hue']) % 180.0,
          'concentration': float(r.get('concentration') or float('nan')),
          'area': float(r['area']), 'fill': float(r['fill']),
          'solidity': float(r['solidity']), 'bbox': tuple(r['bbox'])}
         for r in probe_cands], hue_tol=args.hue_tol)
    probe_str = ','.join(f"{e['label']}:{e['center']:.0f}" for e in entries)
    # 合成图里 5 块共 3 个不同色相（红 0/180、黄、紫），只有紫不在表里，
    # 所以候选应当恰好是 1 个色相，且它必须落在紫色真值附近。
    check(len(entries) == 1,
          f'色表标定找到被漏掉的色相：{probe_str}（表里只有红/黄/绿）',
          f'色表标定候选数不对：期望 1 个（只有紫不在表里），实测 {len(entries)}')
    if entries:
        d_purple = ang_diff_deg(entries[0]['center'], 130.0)
        check(d_purple <= 4.0,
              f'标定出的色相 {entries[0]["center"]:.1f} 与紫色真值 130 相差 '
              f'{d_purple:.1f}',
              f'标定出的色相 {entries[0]["center"]:.1f} 偏离紫色真值 130 达 '
              f'{d_purple:.1f}')
    check(all(e['suggested_tol'] >= 2.0 for e in entries) or not entries,
          '每个色档都给出了自适应的建议容差',
          '有色档的建议容差 < 2，标签边界不可用')
    back = [(e['label'], e['center']) for e in entries]
    closed = True
    for e in entries:
        label, dist, _second = classify_color(e['source']['mean_hue'], back)
        if label != e['label'] or dist > args.hue_tol:
            closed = False
    check(closed and bool(entries),
          f'回代闭环：{len(entries)} 个候选都能匹配上自己的标签',
          '回代闭环失败：生成的色表无法复现自己的量测')

    if args.out:
        imwrite(args.out, annotate(frame, blocks, mask, info))
        imwrite(str(args.out) + '.cube.png', annotate(cube, cube_blocks, None,
                                                      None))
        log(f'自检标注图：{args.out}（以及 .cube.png 上表面示例）')

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
    mode.add_argument('--probe-colors', action='store_true',
                      help='色表自标定：量出现场色相并打印可粘贴的 --colors')

    cam = ap.add_argument_group('相机')
    cam.add_argument('--camera', type=camera_source, default=default_camera(), help='摄像头索引')
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
                     help='轮廓面积上限（像素²）；默认按画面尺寸自适应'
                          '（画面的 5%，用于把机械臂本体挡住）')
    shp.add_argument('--max-block-px', type=float, default=140.0,
                     help='最小外接矩形最长边的上限（像素）；0=不限。'
                          '4cm 物块在 640x480 下约 35~90px，140 给足余量，'
                          '同时挡住机械臂那类大块')
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

    seg = ap.add_argument_group('粘连分割 / 碎条合并 / 色相漂移')
    seg.add_argument('--no-split', action='store_true',
                     help='关掉"按色相切分粘连连通域"（默认开）')
    seg.add_argument('--no-merge', action='store_true',
                     help='关掉"同色碎条并回大块"（默认开）')
    seg.add_argument('--merge-dist', type=float, default=0.35,
                     help='碎条合并的最大间距，单位是宿主块边长；'
                          '必须远小于相邻物块间距，否则会把两块料粘起来')
    seg.add_argument('--no-hue-drift', action='store_true',
                     help='关掉在线色相漂移校正（默认开）')
    seg.add_argument('--hue-drift-max', type=float, default=15.0,
                     help='允许自动校正的最大漂移（H 单位）；超过就拒绝校正并告警，'
                          '避免把色表整体带偏')

    st = ap.add_argument_group('自检')
    st.add_argument('--no-selftest-hue-check', action='store_true',
                    help='自检时跳过"色相表闭环"检查')

    top = ap.add_argument_group('上表面（顶面）检测')
    top.add_argument('--no-top-face', action='store_true',
                     help='关掉上表面检测，只出剪影口径的结果（省一点 CPU）')

    probe = ap.add_argument_group('色表自标定（--probe-colors 的附加参数）')
    probe.add_argument('--probe-frames', type=int, default=15,
                       help='走相机标定时取多少帧做中值（单帧偶发曝光跳变会让标定值不可复现）')

    args = ap.parse_args(argv)
    # --probe-colors 自带一种模式：配 --image 用照片标定，或直接开相机标定
    if not (args.ros or args.selftest or args.image or args.probe_colors):
        ap.error('必须指定一种模式：--image 图片 / --ros / --selftest / --probe-colors')
    return args


def main(argv=None):
    args = parse_args(argv)

    # 色表标定不依赖现有颜色表（它要推翻/校正的正是这张表），所以放在前面
    if args.probe_colors:
        return run_probe_colors(args, dict(DEFAULT_HUE_CENTERS))

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
