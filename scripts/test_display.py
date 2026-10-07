#!/usr/bin/env python3
"""测试 DISPLAY=:0 上能否启动 GUI。"""
import os
import sys

import tkinter

try:
    root = tkinter.Tk()
    root.withdraw()
    print('TK OK: GUI 可启动')
    root.destroy()
except Exception as e:
    print(f'TK FAIL: {e}')
