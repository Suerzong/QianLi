#!/usr/bin/env python3
"""夹爪几何模型：正运动学 + "夹爪最低点"计算（桌面标定 / 防撞闸门共用）

核心概念
--------
**碰到障碍物的是"夹爪几何最低点"，不是 TCP。**

* TCP 是爪口里的抓取中心，工具竖直时它在爪尖上方约 4.6mm；
* TCP 沿工具轴横向偏约 7mm，工具倾斜 θ 时额外产生 7·sin(θ) 的高度差；
* 倾斜更大时最低点会从爪尖换成夹爪机身。

所以"防碰桌子"必须用最低点，用 TCP 的 Z 会随姿态漂移。

ikpy 的坑（踩过）
-----------------
``ikpy`` 链里 ``link.name`` 是**关节名**，不是 URDF 的 link 名。
更早一版直接 ``T.get('gripper_link')`` 永远取不到，静默退化成"整条链的乘积"，
于是 gripper_link 的位姿被算成了 tcp 的位姿 —— 数值看起来像模像样，其实是错的。
这里显式用 URDF 的 joint→child 映射来取名。

另外 ``ikpy`` 从 base 到最深叶子只走**一条链**，``gripper``（活动爪）是旁支，
不在链里，必须自己从 ``gripper_link`` 再乘一次关节变换。
"""

from __future__ import annotations

import math
import os
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

URDF = os.path.expanduser(
    '~/legacy/arm/arm-final/ros2_ws/install/so101_bringup/share/so101_bringup'
    '/urdf/so101.urdf')
JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
          'wrist_roll', 'gripper']
JAW_LINK = 'moving_jaw_so101_v1_link'
JAW_JOINT = 'gripper'
FLANGE_LINK = 'gripper_link'


def load_stl(path):
    with open(path, 'rb') as fh:
        fh.read(80)
        count = struct.unpack('<I', fh.read(4))[0]
        data = np.frombuffer(fh.read(count * 50), dtype=np.uint8).reshape(count, 50)
        return data[:, 12:48].copy().view('<f4').reshape(-1, 3).astype(float)


def rpy_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _xyz(text, default=(0, 0, 0)):
    return (np.array([float(v) for v in text.split()]) if text
            else np.array(default, float))


def _homog(R, t):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


class GripperModel:
    """夹爪的正运动学与最低点计算。"""

    def __init__(self, urdf_path=URDF, stride=4):
        from ikpy.chain import Chain

        self.urdf = urdf_path
        root = ET.parse(urdf_path).getroot()
        self.assets = Path(urdf_path).parent / 'assets'
        self.joints = {j.get('name'): j for j in root.findall('joint')}
        self.joint_child = {j.get('name'): j.find('child').get('link')
                            for j in root.findall('joint')}
        self.links = {l.get('name'): l for l in root.findall('link')}

        self.chain = Chain.from_urdf_file(urdf_path, base_elements=['base_link'])
        self.names = [l.name for l in self.chain.links]
        # 链上第 i 个 link 实际对应哪个 URDF link
        self.chain_child = [self.joint_child.get(n, n) for n in self.names]

        self.parts = {}
        for link in (FLANGE_LINK, JAW_LINK):
            p = self.link_points(link)
            if len(p):
                self.parts[link] = p[::max(1, stride)]

    # ---------------- URDF 网格 ----------------
    def link_points(self, link):
        el = self.links.get(link)
        if el is None:
            return np.zeros((0, 3))
        pts = []
        for vis in el.findall('visual'):
            mesh = vis.find('geometry/mesh')
            if mesh is None:
                continue
            p = self.assets / os.path.basename(mesh.get('filename'))
            if not p.exists():
                continue
            o = vis.find('origin')
            t = _xyz(o.get('xyz') if o is not None else None)
            r = _xyz(o.get('rpy') if o is not None else None)
            pts.append((rpy_matrix(*r) @ load_stl(p).T).T + t)
        return np.vstack(pts) if pts else np.zeros((0, 3))

    def joint_tf(self, name, angle=0.0):
        j = self.joints[name]
        o = j.find('origin')
        t = _xyz(o.get('xyz') if o is not None else None)
        r = _xyz(o.get('rpy') if o is not None else None)
        R = rpy_matrix(*r)
        if j.get('type') == 'revolute':
            R = R @ rot_z(angle)
        return R, t

    # ---------------- 正运动学 ----------------
    def solve(self, joints):
        """{关节名: 角度} → {URDF link 名: 4x4 变换到 base_link}"""
        q = np.zeros(len(self.chain.links))
        for i, link in enumerate(self.chain.links):
            if link.name in joints:
                q[i] = joints[link.name]
        fk = self.chain.forward_kinematics(q, full_kinematics=True)
        # 注意：ikpy 在 full_kinematics=True 时返回的每一项**已经是
        # base→该 link 的累积变换**，不能再自己累乘一遍（踩过这个坑：
        # 再累乘一次会让所有位姿都是错的，但看起来仍然"像那么回事"）。
        out = {}
        for i, name in enumerate(self.chain_child):
            out[name] = np.asarray(fk[i], dtype=float).copy()
        # 链首是 base element，它的名字在 URDF 里是 base_link
        out.setdefault('base_link', np.eye(4))
        # 活动爪不在链上，从 gripper_link 再乘一次 gripper 关节
        if FLANGE_LINK in out:
            R, t = self.joint_tf(JAW_JOINT, joints.get(JAW_JOINT, 0.0))
            out[JAW_LINK] = out[FLANGE_LINK] @ _homog(R, t)
        return out

    def lowest_point(self, joints):
        """返回 (最低点 xyz, 所在 link, 各点 z 的最小值)。"""
        T = self.solve(joints)
        best, best_link = None, None
        for link, pts in self.parts.items():
            M = T.get(link)
            if M is None:
                continue
            world = (M[:3, :3] @ pts.T).T + M[:3, 3]
            k = int(np.argmin(world[:, 2]))
            if best is None or world[k, 2] < best[2]:
                best, best_link = world[k].copy(), link
        return best, best_link

    def tcp(self, joints):
        T = self.solve(joints)
        M = T.get('tcp_link')
        return None if M is None else M[:3, 3]

    # ---------------- 自检 ----------------
    def selftest(self):
        """验证 FK 真的落在该落的 link 上（防"名字对不上静默退化"）。"""
        fails = []
        z = {n: 0.0 for n in JOINTS}
        T = self.solve(z)
        for name in ('gripper_link', 'gripper_frame_link', 'tcp_link', JAW_LINK):
            if name not in T:
                fails.append(f'{name} 不在 FK 结果里')
        if not fails:
            # URDF 里 tcp_joint 相对 gripper_frame_link 的固定偏移
            R, t = self.joint_tf('tcp_joint')
            expect = T['gripper_frame_link'][:3, 3] + \
                T['gripper_frame_link'][:3, :3] @ t
            got = T['tcp_link'][:3, 3]
            err = np.linalg.norm(expect - got) * 1000
            if err > 0.01:
                fails.append(f'tcp_link 位姿与固定偏移不符，差 {err:.3f} mm')
            # gripper_link 与 tcp_link 必须不同（之前的 bug 就是它们相等）
            d = np.linalg.norm(T['gripper_link'][:3, 3] - got) * 1000
            if d < 1.0:
                fails.append(f'gripper_link 与 tcp_link 重合（差 {d:.3f} mm）'
                             f'—— 典型的 ikpy 命名退化')
            # 活动爪应当随 gripper 关节角移动 —— 注意要比**爪上的网格点**，
            # 不能比 link 原点：关节是绕自己的原点转的，原点本来就不动。
            pts = self.parts.get(JAW_LINK)
            if pts is None or not len(pts):
                fails.append('活动爪没有网格点，无法验证')
            else:
                probe = pts[::max(1, len(pts) // 200)]
                def jaw_point(angle):
                    T = self.solve({**z, JAW_JOINT: angle})[JAW_LINK]
                    return (T[:3, :3] @ probe.T).T + T[:3, 3]
                move = np.linalg.norm(jaw_point(0.0) - jaw_point(1.2),
                                      axis=1).mean() * 1000
                if move < 5.0:
                    fails.append(f'活动爪没随 gripper 关节动（平均只移动 '
                                 f'{move:.2f} mm）')
        return fails


if __name__ == '__main__':
    import sys
    m = GripperModel(stride=8)
    print('链:', m.names)
    print('链→URDF link:', m.chain_child)
    fails = m.selftest()
    if fails:
        print('❌ 自检失败:')
        for f in fails:
            print('   ·', f)
        sys.exit(1)
    print('✅ FK 自检通过')
    z = {n: 0.0 for n in JOINTS}
    low, link = m.lowest_point(z)
    print(f'零位姿态：TCP Z={m.tcp(z)[2]*1000:+.2f} mm  '
          f'最低点 Z={low[2]*1000:+.2f} mm  属于 {link}')
    sys.exit(0)
