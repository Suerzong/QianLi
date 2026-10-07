#!/usr/bin/env python3
"""打印 URDF 里 gripper_link / moving_jaw 的 mesh 与 origin。"""
import re
import sys

u = open('/home/ros/legacy/arm/arm-final/ros2_ws/install/so101_bringup/'
         'share/so101_bringup/urdf/so101.urdf').read()

for name in ('gripper_link', 'moving_jaw_so101_v1_link', 'gripper_frame_link'):
    m = re.search(r'<link name="%s".*?</link>' % name, u, re.S)
    print(f'=== {name} ===')
    if not m:
        print('  NOT FOUND')
        continue
    for line in m.group(0).splitlines():
        line = line.strip()
        if ('link name' in line or 'mesh' in line or 'origin xyz' in line
                or 'visual' in line):
            print(' ', line)
    print()
