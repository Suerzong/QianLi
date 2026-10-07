#!/usr/bin/env python3
"""检查 digital_twin 模块暴露的常量。"""

from project_paths import project_path
import os
import sys

sys.path.insert(0, project_path('qianli_ws/src/qianli_vision/scripts'))
import digital_twin as dt

print('digital_twin 模块路径:', dt.__file__)
print('BOARD 相关:', [x for x in dir(dt) if 'BOARD' in x])
print('CELL 相关:', [x for x in dir(dt) if 'CELL' in x])
print('TABLE_Z:', dt.TABLE_Z)
print('BOARD_ORIGIN:', dt.BOARD_ORIGIN)
