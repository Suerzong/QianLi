#!/usr/bin/env python3
"""RViz 场景渲染：桌面 + 垫台 + 网格 + 工作区

为什么这些数字是可信的
----------------------
不是"摆个好看的模型"，每个尺寸都有实测出处：

    桌面 z      -69.09 mm   夹爪最低点碰桌、6 点拟合平面，残差 RMS 0.469mm
    平面倾角     0.202°     同上（法向实测 (0.0030, -0.0019, 0.99999)）
    垫台高度    66.69 mm    base_link 安装面(-2.40mm，从 base_so101_v2.stl 算)
                           与实测桌面之差；旧"5cm"是估算，差 16.7mm

为什么餐桌板要半透明
--------------------
安全闸门会在桌面附近画"最低点→桌面"的间隙竖线。桌面画成不透明实体
会把那些标记吞掉 —— 这个坑在绿球被灰板吞掉那次已经踩过。

物块和棋盘为什么不在这里
------------------------
它们的位置依赖**外参**（像素 → base_link）。外参标定完成前，
往这里放物块只能是编造坐标，反而会让人以为已经标好了。
"""

from __future__ import annotations

import argparse
import math
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def quat_from_normal(n):
    """把"平面法向"转成让 +Z 对齐该法向的四元数。

    桌面实测有 0.202° 倾角，直接画成水平会掩盖这个事实。
    虽然 0.2° 在 1 米跨度上只有 3.5mm，但既然测出来了就照实画。

    注意：返回的必须是**原生 Python float**。numpy 的 float64 赋给 ROS 消息
    字段会触发 `PyFloat_Check(field)` 断言失败直接 abort（实测踩到，
    整个节点 core dump，而且报错信息只说断言失败、不说是哪个字段）。
    """
    n = np.asarray(n, float)
    n = n / np.linalg.norm(n)
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(z, n)
    c = float(z @ n)
    if np.linalg.norm(v) < 1e-12:
        q = (0.0, 0.0, 0.0, 1.0) if c > 0 else (1.0, 0.0, 0.0, 0.0)
        return q
    axis = v / np.linalg.norm(v)
    ang = math.acos(max(-1.0, min(1.0, c)))
    s = math.sin(ang / 2.0)
    return (float(axis[0] * s), float(axis[1] * s), float(axis[2] * s),
            float(math.cos(ang / 2.0)))


def box(mid, ns, center, size, color, quat=(0, 0, 0, 1)):
    m = Marker()
    m.header.frame_id = 'base_link'
    m.ns = ns
    m.id = mid
    m.type = Marker.CUBE
    m.action = Marker.ADD
    # 统一 float() 强转：数值可能来自 numpy，直接赋给 ROS 字段会 abort
    m.pose.position.x = float(center[0])
    m.pose.position.y = float(center[1])
    m.pose.position.z = float(center[2])
    m.pose.orientation.x = float(quat[0])
    m.pose.orientation.y = float(quat[1])
    m.pose.orientation.z = float(quat[2])
    m.pose.orientation.w = float(quat[3])
    m.scale.x = float(size[0])
    m.scale.y = float(size[1])
    m.scale.z = float(size[2])
    m.color.r = float(color[0])
    m.color.g = float(color[1])
    m.color.b = float(color[2])
    m.color.a = float(color[3])
    return m


def line(mid, ns, pts, color, width=0.002):
    m = Marker()
    m.header.frame_id = 'base_link'
    m.ns = ns
    m.id = mid
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = float(width)
    m.color.r = float(color[0])
    m.color.g = float(color[1])
    m.color.b = float(color[2])
    m.color.a = float(color[3])
    m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                for p in pts]
    return m


def text(mid, ns, pos, color, txt, size=0.02):
    m = Marker()
    m.header.frame_id = 'base_link'
    m.ns = ns
    m.id = mid
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.pose.position.x = float(pos[0])
    m.pose.position.y = float(pos[1])
    m.pose.position.z = float(pos[2])
    m.pose.orientation.w = 1.0
    m.scale.z = float(size)
    m.color.r = float(color[0])
    m.color.g = float(color[1])
    m.color.b = float(color[2])
    m.color.a = float(color[3])
    m.text = txt
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--table-z', type=float, default=-0.06909)
    ap.add_argument('--table-tilt', type=float, default=0.202,
                    help='实测平面倾角（度）')
    ap.add_argument('--pedestal-top', type=float, default=-0.0024,
                    help='base_link 安装面（URDF 里 base_so101_v2.stl 最低点）')
    ap.add_argument('--table-size', default='1.40,1.00',
                    help='桌面尺寸 长,宽 (m)')
    ap.add_argument('--table-thick', type=float, default=0.028)
    ap.add_argument('--pedestal-size', default='0.090,0.100')
    ap.add_argument('--grid-step', type=float, default=0.05)
    ap.add_argument('--grid-span', type=float, default=0.50)
    ap.add_argument('--with-grid', action='store_true', default=True,
                    help='在桌面上画 5cm 网格线（方便目测尺度）')
    ap.add_argument('--no-grid', dest='with_grid', action='store_false')
    args = ap.parse_args()

    tsx, tsy = (float(v) for v in args.table_size.split(','))
    psx, psy = (float(v) for v in args.pedestal_size.split(','))
    ped_h = args.pedestal_top - args.table_z

    # 桌面法向：绕 Y 轴倾 tilt 度（实测法向里 x 分量 0.0030 占主导）
    tilt = math.radians(args.table_tilt)
    normal = (math.sin(tilt), 0.0, math.cos(tilt))
    q = quat_from_normal(normal)

    rclpy.init()
    node = Node('scene_markers')
    pub = node.create_publisher(MarkerArray, '/scene_markers', 1)
    pub_info = node.create_publisher(String, '/scene_info', 1)

    arr = MarkerArray()

    # ---- 桌面（板体，上表面正好在 table_z）----
    arr.markers.append(box(
        1, 'table', [0.0, 0.0, args.table_z - args.table_thick / 2.0],
        [tsx, tsy, args.table_thick], (0.78, 0.76, 0.72, 0.55), q))
    # 桌面边缘描一圈，深色，让轮廓清楚
    hx, hy = tsx / 2.0, tsy / 2.0
    zt = args.table_z + 0.0002
    edge = [(hx, hy, zt), (-hx, hy, zt), (-hx, -hy, zt),
            (hx, -hy, zt), (hx, hy, zt)]
    arr.markers.append(line(2, 'table_edge', edge, (0.35, 0.35, 0.38, 1.0),
                            0.003))

    # ---- 垫台：从桌面顶到 base_link 安装面 ----
    arr.markers.append(box(
        3, 'pedestal', [0.0, 0.0, (args.table_z + args.pedestal_top) / 2.0],
        [psx, psy, max(ped_h, 1e-4)], (0.22, 0.23, 0.25, 1.0)))
    arr.markers.append(line(4, 'pedestal_edge', [
        (psx / 2, psy / 2, args.pedestal_top), (-psx / 2, psy / 2,
                                                args.pedestal_top),
        (-psx / 2, -psy / 2, args.pedestal_top),
        (psx / 2, -psy / 2, args.pedestal_top),
        (psx / 2, psy / 2, args.pedestal_top)],
        (0.55, 0.57, 0.60, 1.0), 0.002))

    # ---- 桌面上的 5cm 网格（目测尺度用）----
    if args.with_grid:
        mid = 10
        span = args.grid_span
        step = args.grid_step
        n = int(span / step)
        for i in range(-n, n + 1):
            x = i * step
            arr.markers.append(line(mid, 'table_grid',
                                    [(x, -span, zt), (x, span, zt)],
                                    (0.45, 0.48, 0.52, 0.35), 0.0012))
            mid += 1
            arr.markers.append(line(mid, 'table_grid',
                                    [(-span, x, zt), (span, x, zt)],
                                    (0.45, 0.48, 0.52, 0.35), 0.0012))
            mid += 1

    # ---- 尺寸标注 ----
    arr.markers.append(text(60, 'labels', [0.30, 0.16, args.table_z + 0.02],
                            (0.9, 0.9, 0.95, 1.0),
                            f'table  z={args.table_z*1000:+.2f}mm  '
                            f'tilt {args.table_tilt:.3f}°', 0.016))
    arr.markers.append(text(61, 'labels', [0.12, 0.0,
                                           (args.table_z + args.pedestal_top) / 2],
                            (1.0, 1.0, 1.0, 1.0),
                            f'pedestal  {ped_h*1000:.2f}mm', 0.011))

    pub.publish(arr)
    node.get_logger().info(
        f'场景已发布：桌面 {tsx*1000:.0f}x{tsy*1000:.0f}mm '
        f'z={args.table_z*1000:+.2f}mm 倾角 {args.table_tilt:.3f}°；'
        f'垫台 {psx*1000:.0f}x{psy*1000:.0f}x{ped_h*1000:.2f}mm')

    msg = String()
    import json
    msg.data = json.dumps({
        'table_z_mm': round(args.table_z * 1000, 2),
        'table_tilt_deg': args.table_tilt,
        'pedestal_mm': round(ped_h * 1000, 2),
        'pedestal_top_mm': round(args.pedestal_top * 1000, 2),
        'table_size_mm': [tsx * 1000, tsy * 1000],
    })
    pub_info.publish(msg)

    # 场景是静态的，不需要高频重发。但 RViz 订阅者可能晚于我们启动，
    # 所以先密集发几秒再降到低频"心跳"。
    import time
    t0 = time.time()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        dt = time.time() - t0
        if dt < 20.0 or int(dt) % 5 == 0:
            pub.publish(arr)
            pub_info.publish(msg)
        time.sleep(0.5)

    try:
        node.destroy_node()
    except Exception:  # noqa: BLE001
        pass
    if rclpy.ok():
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
