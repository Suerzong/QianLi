#!/usr/bin/env python3
"""Z 微调前端（Tkinter）—— 一点点下放夹爪，实时看坐标

功能：
  · Z 滑块（-75~+60 mm，1mm 分辨率）：拖动即实时下发
  · 微调按钮：Z ±1mm / ±10mm，X/Y ±1mm / ±5mm（对准物块）
  · 夹爪：张开(1.2) / 闭合(0.0)
  · 使能 / 松扭矩；急停
  · 实时显示当前 TCP 坐标 + 参考线（示教抓取高度、桌面安全下限）

安全：
  · z 硬限位（默认 = 示教算出的 safe_z_min，即桌面 +3mm）
  · 松扭矩后不再下发目标
  · 窗口关闭即停止下发（机械臂保持当前位置）

参考数据来自 /tmp/grasp_pose.txt（teach_grasp.py 产出）。
"""

import os
import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

TCP_FRAME = 'gripper_frame_link'
POSE_FILE = '/tmp/grasp_pose.txt'
Z_MIN_MM, Z_MAX_MM = -75, 60


def load_ref():
    """读取示教基准，返回 dict。"""
    ref = {'grasp_z': None, 'board_z': None, 'safe_z_min': -0.072}
    if os.path.exists(POSE_FILE):
        try:
            for line in open(POSE_FILE):
                line = line.strip()
                if '=' in line and not line.startswith('#'):
                    k, v = line.split('=', 1)
                    if k in ref:
                        ref[k] = float(v)
        except OSError:
            pass
    return ref


class Shared:
    """GUI 与 ROS 线程共享状态。"""

    def __init__(self, ref):
        self.ref = ref
        self.tx = self.ty = self.tz = None   # 目标
        self.cx = self.cy = self.cz = None   # 当前（TF）
        self.enabled = False
        self.grip_cmd = None                 # 待发送的夹爪指令
        self.quit = False
        self.safe_z = ref['safe_z_min'] if ref['safe_z_min'] else -0.072


class JogNode(Node):
    def __init__(self, st):
        super().__init__('z_jog')
        self.st = st
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.pub = self.create_publisher(PointStamped, '/arm/target_position',
                                         10)
        self.pub_g = self.create_publisher(Float64, '/gripper_command', 10)
        self.create_subscription(String, '/arm/status', self._on_status, 10)
        self.cli = self.create_client(SetBool, '/arm/enable')
        self.create_timer(0.1, self._tick)        # 10Hz 下发
        self.create_timer(0.2, self._read_tcp)    # 5Hz 读回

    def _on_status(self, msg):
        self.st.enabled = '"enabled": true' in msg.data

    def _read_tcp(self):
        try:
            tr = self.buf.lookup_transform('base_link', TCP_FRAME,
                                           rclpy.time.Time())
            t = tr.transform.translation
            self.st.cx, self.st.cy, self.st.cz = t.x, t.y, t.z
        except Exception:
            pass

    def set_enable(self, on):
        if not self.cli.service_is_ready():
            self.get_logger().warn('/arm/enable 未就绪')
            return
        req = SetBool.Request()
        req.data = on
        self.cli.call_async(req)

    def _tick(self):
        st = self.st
        if st.quit:
            return
        # 夹爪指令（按需发送）
        if st.grip_cmd is not None:
            m = Float64()
            m.data = st.grip_cmd
            for _ in range(3):
                self.pub_g.publish(m)
            st.grip_cmd = None
        if not st.enabled or st.tx is None:
            return
        m = PointStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.point.x, m.point.y, m.point.z = st.tx, st.ty, st.tz
        self.pub.publish(m)


def main():
    ref = load_ref()
    st = Shared(ref)

    rclpy.init()
    node = JogNode(st)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    # 等第一帧 TF
    import time
    for _ in range(50):
        if st.cx is not None:
            break
        time.sleep(0.1)
    if st.cx is None:
        print('❌ 读不到 TCP 位置')
        return
    st.tx, st.ty, st.tz = st.cx, st.cy, st.cz

    root = tk.Tk()
    root.title('QianLi  Z 微调（一点点下放）')
    root.geometry('+80+80')

    cur = tk.StringVar()
    tgt = tk.StringVar()
    refv = tk.StringVar()
    en = tk.StringVar()

    tk.Label(root, textvariable=cur, font=('monospace', 12),
             justify='left').pack(padx=10, pady=(10, 2), anchor='w')
    tk.Label(root, textvariable=tgt, font=('monospace', 12),
             justify='left').pack(padx=10, anchor='w')
    tk.Label(root, textvariable=refv, font=('monospace', 10), fg='#666',
             justify='left').pack(padx=10, pady=(2, 8), anchor='w')

    zscale = tk.Scale(root, from_=Z_MIN_MM, to=Z_MAX_MM, resolution=1,
                      orient='horizontal', length=460, label='Z (mm)',
                      font=('monospace', 11))
    zscale.set(int(round(st.tz * 1000)))
    zscale.pack(padx=10)

    def set_tz(mm):
        mm = max(Z_MIN_MM, min(Z_MAX_MM, mm))
        if mm / 1000.0 < st.safe_z:
            mm = int(st.safe_z * 1000)      # 安全下限
        zscale.set(mm)
        st.tz = mm / 1000.0

    zscale.config(command=lambda v: set_tz(int(float(v))))

    row = tk.Frame(root)
    row.pack(pady=4)
    for txt, dz in (('−10mm', -10), ('−1mm', -1), ('+1mm', 1), ('+10mm', 10)):
        tk.Button(row, text=txt, width=7,
                  command=lambda d=dz: set_tz(int(round(st.tz * 1000)) + d)
                  ).pack(side='left', padx=2)

    row2 = tk.Frame(root)
    row2.pack(pady=4)
    tk.Label(row2, text='对准 X/Y:', font=('monospace', 10)).pack(side='left')
    for txt, axis, d in (('Y−', 'y', -0.005), ('Y+', 'y', 0.005),
                         ('X−', 'x', -0.005), ('X+', 'x', 0.005)):
        def nudge(a=axis, dd=d):
            if a == 'x':
                st.tx += dd
            else:
                st.ty += dd
        tk.Button(row2, text=txt, width=5, command=nudge).pack(side='left',
                                                               padx=2)

    row3 = tk.Frame(root)
    row3.pack(pady=6)
    tk.Button(row3, text='张开夹爪', width=10,
              command=lambda: setattr(st, 'grip_cmd', 1.2)).pack(side='left',
                                                                 padx=3)
    tk.Button(row3, text='闭合夹爪', width=10,
              command=lambda: setattr(st, 'grip_cmd', 0.0)).pack(side='left',
                                                                 padx=3)
    tk.Button(row3, text='使能', width=8,
              command=lambda: node.set_enable(True)).pack(side='left', padx=3)
    tk.Button(row3, text='松扭矩', width=8,
              command=lambda: node.set_enable(False)).pack(side='left', padx=3)

    tk.Label(root, textvariable=en, font=('monospace', 10)).pack(pady=(0, 8))

    def refresh():
        if st.cx is not None:
            cur.set(f'当前  x={st.cx:+.4f}  y={st.cy:+.4f}  z={st.cz:+.4f}')
            tgt.set(f'目标  x={st.tx:+.4f}  y={st.ty:+.4f}  z={st.tz:+.4f}')
        r = st.ref
        refv.set('示教抓取 z=%.4f   桌面 z=%.4f   安全下限 z=%.4f'
                 % (r['grasp_z'] or 0, r['board_z'] or 0, st.safe_z))
        en.set('运动: ' + ('已使能 ✅' if st.enabled else '未使能 ⛔（点"使能"）'))
        root.after(200, refresh)

    def on_close():
        st.quit = True
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', on_close)
    refresh()
    print('GUI 启动。关闭窗口即停止下发目标。')
    root.mainloop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
