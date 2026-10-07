#!/usr/bin/env python3
"""检查两个 robot_state_publisher 各自发布的 TF 树差异。

标定脚本依赖 TF 里的 tcp_link / gripper_link / moving_jaw。
两个 RSP 竞争发布会污染 TF —— 需要确认哪个该留。
"""
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

FRAMES = ['tcp_link', 'gripper_link', 'moving_jaw_so101_v1_link',
          'gripper_frame_link']


def main():
    rclpy.init()
    node = Node('tf_tree_check')
    buf = Buffer()
    tf = TransformListener(buf, node)
    time.sleep(2.0)
    for f in FRAMES:
        ok = False
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
            try:
                tr = buf.lookup_transform('base_link', f, rclpy.time.Time())
                t = tr.transform.translation
                print(f'{f:>28}: ({t.x:+.4f}, {t.y:+.4f}, {t.z:+.4f})  ✅')
                ok = True
                break
            except Exception:
                pass
        if not ok:
            print(f'{f:>28}: 查不到  ❌')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
