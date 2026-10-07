#!/usr/bin/env python3
"""检查两个 RSP 参数文件里的 robot_description 是否含 tcp_link / so101 特征。"""
import re
import sys

for path, label in [('/tmp/launch_params_mu88joxo', 'PID12099(mu88joxo)'),
                    ('/tmp/launch_params_newbhqf4', 'PID19641(newbhqf4)')]:
    try:
        raw = open(path, encoding='utf-8', errors='replace').read()
    except Exception as e:
        print(f'{label}: 读取失败 {e}')
        continue
    has_tcp = 'tcp_link' in raw
    has_so101 = ('so101' in raw.lower())
    has_gripper = 'gripper_link' in raw
    has_moving = 'moving_jaw' in raw
    # 找 URDF 路径引用
    urdf_refs = re.findall(r'[A-Za-z0-9_./~-]+\.(?:urdf|xacro)', raw)[:4]
    print(f'{label}:')
    print(f'  tcp_link={has_tcp}  so101={has_so101}  gripper_link={has_gripper}'
          f'  moving_jaw={has_moving}')
    print(f'  urdf refs: {urdf_refs}')
    print()
