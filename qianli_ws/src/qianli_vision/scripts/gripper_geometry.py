#!/usr/bin/env python3
"""计算夹爪几何：爪尖相对 gripper_frame_link 的位置（从 URDF 网格精确算）

目的：
  抓取高度到底该把 gripper_frame_link 放到哪？
  需要知道"爪尖"在 frame 坐标系里沿工具轴（frame +Z）偏多少。

方法：
  读 URDF 里 gripper_link / moving_jaw_so101_v1_link 的所有 <visual>，
  把对应 STL 顶点变换到 gripper_frame_link 坐标系，统计三个方向的极值。

坐标系链：
  gripper_link → gripper_frame_link: origin(-0.0079,-0.000218,-0.0981274),
                                     rpy(0, π, 0)
  gripper_link → moving_jaw:         origin(0.0202,0.0188,-0.0234),
                                     rpy(π/2, 0, 0)
"""

from project_paths import so101_path

import math
import os
import struct
import xml.etree.ElementTree as ET

import numpy as np

URDF = os.path.expanduser(
    so101_path('urdf/so101.urdf'))


def load_stl(path):
    with open(path, 'rb') as f:
        head = f.read(80)
        n = struct.unpack('<I', f.read(4))[0]
        data = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
        v = data[:, 12:48].copy().view('<f4').reshape(-1, 3)
    return v.astype(float)


def rpy_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx          # URDF rpy = Rz(y)Ry(p)Rx(r)


def parse_xyz(s, default=(0, 0, 0)):
    if not s:
        return np.array(default, dtype=float)
    return np.array([float(v) for v in s.split()], dtype=float)


def main():
    tree = ET.parse(URDF)
    root = tree.getroot()
    assets = os.path.join(os.path.dirname(URDF), 'assets')

    # 找 gripper_frame_joint，得到 gripper_link → frame 的变换
    T_link_to_frame = None
    T_link_to_jaw = None
    for j in root.findall('joint'):
        if j.get('name') == 'gripper_frame_joint':
            o = j.find('origin')
            t = parse_xyz(o.get('xyz') if o is not None else None)
            r = parse_xyz(o.get('rpy') if o is not None else None)
            R = rpy_matrix(*r)
            T_link_to_frame = (R, t)
        if j.get('name') == 'gripper':
            o = j.find('origin')
            t = parse_xyz(o.get('xyz') if o is not None else None)
            r = parse_xyz(o.get('rpy') if o is not None else None)
            R = rpy_matrix(*r)
            T_link_to_jaw = (R, t)

    def to_frame(R_link, t_link, link_name):
        """把某 link 下所有网格顶点变换到 frame 坐标。"""
        link = None
        for l in root.findall('link'):
            if l.get('name') == link_name:
                link = l
                break
        if link is None:
            return np.zeros((0, 3))
        pts = []
        for vis in link.findall('visual'):
            o = vis.find('origin')
            tv = parse_xyz(o.get('xyz') if o is not None else None)
            rv = parse_xyz(o.get('rpy') if o is not None else None)
            Rv = rpy_matrix(*rv)
            mesh = vis.find('geometry/mesh')
            if mesh is None:
                continue
            fn = os.path.basename(mesh.get('filename'))
            path = os.path.join(assets, fn)
            if not os.path.exists(path):
                continue
            v = load_stl(path)
            # mesh → link
            v_link = (Rv @ v.T).T + tv
            # link → frame
            v_frame = (R_link.T @ (v_link - t_link).T).T
            pts.append(v_frame)
        return np.vstack(pts) if pts else np.zeros((0, 3))

    R_f, t_f = T_link_to_frame
    R_j, t_j = T_link_to_jaw

    # gripper_link 自身的网格（含固定爪）
    p_link = to_frame(R_f, t_f, 'gripper_link')
    # moving_jaw 的网格先到 gripper_link 再转 frame
    link = None
    for l in root.findall('link'):
        if l.get('name') == 'moving_jaw_so101_v1_link':
            link = l
    pts = []
    for vis in link.findall('visual'):
        o = vis.find('origin')
        tv = parse_xyz(o.get('xyz') if o is not None else None)
        rv = parse_xyz(o.get('rpy') if o is not None else None)
        Rv = rpy_matrix(*rv)
        mesh = vis.find('geometry/mesh')
        if mesh is None:
            continue
        path = os.path.join(assets, os.path.basename(mesh.get('filename')))
        if not os.path.exists(path):
            continue
        v = load_stl(path)
        v_jaw = (Rv @ v.T).T + tv                 # mesh → jaw link
        v_link = (R_j @ v_jaw.T).T + t_j          # jaw link → gripper_link
        v_frame = (R_f.T @ (v_link - t_f).T).T    # gripper_link → frame
        pts.append(v_frame)
    p_jaw = np.vstack(pts) if pts else np.zeros((0, 3))

    allp = np.vstack([p for p in (p_link, p_jaw) if len(p)])
    print(f'gripper_frame_link 坐标系下：共 {len(allp)} 个顶点')
    print(f'  沿工具轴 (frame +Z): {allp[:,2].min():+.4f} ~ {allp[:,2].max():+.4f} m')
    print(f'  左右 (frame +X):     {allp[:,0].min():+.4f} ~ {allp[:,0].max():+.4f} m')
    print(f'  前后 (frame +Y):     {allp[:,1].min():+.4f} ~ {allp[:,1].max():+.4f} m')
    print()
    print('解读：')
    print(f'  · 若 frame +Z 向下（工具朝下），则爪尖在 frame 的 +Z={allp[:,2].max():+.4f} 处')
    print(f'    → 即爪尖比 frame 原点再低 {allp[:,2].max()*1000:.1f} mm')
    print(f'  · 夹爪张开方向沿 frame +X（固定爪在 +X 侧，活动爪在 -X 侧）')


if __name__ == '__main__':
    main()
