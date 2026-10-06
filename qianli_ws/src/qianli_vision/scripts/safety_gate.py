#!/usr/bin/env python3
"""安全闸门：任何关节指令在执行前都要过"整机 vs 桌面"的模型检查

插在链路里的位置
----------------
    auto_grasp / 其它  →  ik_node  →  [safety_gate]  →  driver_node
                                        ↑ 这里
ik_node 用 ROS remap 把输出改到 /joint_commands_raw：

    ros2 run so101_bringup ik_node --ros-args \
        -r /joint_commands:=/joint_commands_raw

这样**任何**来源的指令都绕不过闸门 —— 不用去改每个调用方。

判定规则（关键）
----------------
不能简单地"净空不足就拒"，否则一旦停在贴桌姿态就再也动不了。
正确规则是：**不允许"朝桌子方向"越过裕度线**。

    clearance(candidate) >= margin                  → 放行
    clearance(candidate) >  clearance(current)      → 放行（在远离桌子）
    否则                                             → 拒绝

被拒绝时**继续发上一个安全指令**，让机械臂保持原位 ——
直接不发布的话，1 秒后驱动看门狗会断扭矩、机械臂失力下垂。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gripper_model import GripperModel, JOINTS  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--table-z', type=float, default=-0.06909)
    ap.add_argument('--margin-mm', type=float, default=8.0,
                    help='安全裕度：目标净空 >= 这个值才无条件放行')
    ap.add_argument('--improve-eps-mm', type=float, default=0.5,
                    help='净空比当前改善超过这个量就放行（哪怕仍低于裕度）')
    ap.add_argument('--stride', type=int, default=40)
    ap.add_argument('--dry-run', action='store_true',
                    help='只报告不拦截（先观察用）')
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    model = GripperModel(stride=args.stride)
    fails = model.selftest()
    if fails:
        print('❌ FK 自检未通过，拒绝启动闸门：')
        for f in fails:
            print('   ·', f)
        return 1
    model.prepare_clearance(stride=args.stride)
    print(f'净空模型：{len(model._clr)} 个 link、'
          f'{sum(len(v) for v in model._clr.values())} 点')

    rclpy.init()
    node = Node('safety_gate')
    pub = node.create_publisher(JointState, '/joint_commands', 10)
    pub_state = node.create_publisher(String, '/safety_gate_state', 10)

    st = {'measured': None, 'last_safe': None, 'accepted': 0,
          'rejected': 0, 'last_reason': ''}

    def on_measured(m):
        if len(m.name) == len(m.position) and len(m.position) == 6:
            st['measured'] = {k: float(v) for k, v in zip(m.name, m.position)}

    def clearance_of(qvec):
        d = {k: float(v) for k, v in zip(JOINTS, qvec)}
        low, link = model.lowest_over_all(d)
        return low[2] - args.table_z, link, low

    def on_candidate(m):
        try:
            _handle(m)
        except Exception as exc:  # noqa: BLE001
            # 闸门自己绝不能崩 —— 崩了就等于安全策略失效，而且机械臂会因
            # 看门狗断扭矩失力下垂。出错就保守放行当前指令并报警。
            st['errors'] = st.get('errors', 0) + 1
            node.get_logger().error(
                f'闸门内部异常（已放行本帧）: {type(exc).__name__}: {exc}',
                throttle_duration_sec=2.0)

    def _handle(m):
        if len(m.position) != 6:
            return
        # 关节顺序以消息自带的名字为准，缺名字时按 URDF 顺序。
        # 注意 last_safe / measured 统一都用**列表**存，别混 dict，
        # 否则 ref[k] 这种字符串下标会直接抛 TypeError（踩过）。
        if len(m.name) == 6:
            idx = {n: i for i, n in enumerate(m.name)}
            cand = [float(m.position[idx[k]]) if k in idx
                    else float(m.position[i]) for i, k in enumerate(JOINTS)]
        else:
            cand = [float(v) for v in m.position]
        st['last_candidate'] = cand

        c_cand, link_cand, low_cand = clearance_of(cand)
        ref = st['last_safe']
        if ref is None:
            if st['measured'] is None:
                # 还没有任何参考，先放行并记账，避免启动时卡死
                st['last_safe'] = cand
                pub.publish(m)
                st['accepted'] += 1
                return
            ref = [st['measured'][k] for k in JOINTS]
        c_cur, _, _ = clearance_of(ref)

        # 规则：净空够 → 放行；否则**只要不朝桌子方向恶化**也放行。
        # 不能写成"低于裕度一律拒" —— 那样一旦停在贴桌姿态就再也动不了，
        # 连"保持不动"的指令都会被拒。
        eps = args.improve_eps_mm / 1000.0
        ok = (c_cand >= args.margin_mm / 1000.0) or (c_cand >= c_cur - eps)
        reason = (f'cand {c_cand*1000:+.2f}mm vs margin '
                  f'{args.margin_mm:.1f}mm, current {c_cur*1000:+.2f}mm, '
                  f'受限部件 {link_cand}')
        st['last_reason'] = reason

        if ok or args.dry_run:
            st['last_safe'] = cand
            pub.publish(m)
            st['accepted'] += 1
            if args.dry_run and not ok:
                node.get_logger().warning(
                    f'[DRY-RUN 本应拦截] {reason}', throttle_duration_sec=2.0)
        else:
            st['rejected'] += 1
            node.get_logger().error(
                f'⛔ 拒绝指令（会撞桌面）: {reason}', throttle_duration_sec=1.0)
            # 保持上一个安全姿态，别让看门狗把扭矩断掉
            hold = JointState()
            hold.header.stamp = node.get_clock().now().to_msg()
            hold.name = list(JOINTS)
            hold.position = [float(v) for v in ref]
            pub.publish(hold)

    node.create_subscription(JointState, '/joint_states', on_measured, 10)
    node.create_subscription(JointState, '/joint_commands_raw',
                             on_candidate, 10)

    # 周期性播报状态
    def report():
        msg = String()
        msg.data = json.dumps({
            'accepted': st['accepted'], 'rejected': st['rejected'],
            'dry_run': bool(args.dry_run),
            'margin_mm': args.margin_mm,
            'last_reason': st['last_reason'],
            'table_z_mm': round(args.table_z * 1000, 2),
        })
        pub_state.publish(msg)

    node.create_timer(1.0, report)
    print(f'安全闸门已启动：桌面 z={args.table_z*1000:+.2f}mm  '
          f'裕度 {args.margin_mm:.1f}mm  '
          f'{"[DRY-RUN 只报告不拦截]" if args.dry_run else "[拦截生效]"}')
    print('  输入 /joint_commands_raw  →  输出 /joint_commands')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print(f"\n统计：放行 {st['accepted']}  拦截 {st['rejected']}")
    try:
        node.destroy_node()
    except Exception:  # noqa: BLE001
        pass
    if rclpy.ok():
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
