#!/usr/bin/env python3
"""分析 /tmp/enc_trace.csv 的阶梯签名。"""
import ast
import csv
import sys

import numpy as np

rows = []
with open(sys.argv[1]) as f:
    for r in csv.DictReader(f):
        rows.append((float(r['t']), ast.literal_eval(r['cmd0..cmd5']),
                     ast.literal_eval(r['act0..act5'])))
t = np.array([x[0] for x in rows])
cmd = np.array([x[1] for x in rows])
act = np.array([x[2] for x in rows])
names = ['pan', 'lift', 'elbow', 'wrist_f', 'roll', 'gripper']
print('=== 阶梯签名：命令在动但实际停住 ===')
for j, nm in enumerate(names):
    dcmd = np.diff(cmd[:, j])
    dact = np.diff(act[:, j])
    cmd_moving = np.abs(dcmd) >= 2
    act_stuck = np.abs(dact) < 1
    stuck = int(np.sum(cmd_moving & act_stuck))
    print(f'  {nm:8s} 命令动{int(cmd_moving.sum())}/219拍  其中实际停住 {stuck} 拍')
print('\n=== wrist_f 前 30 拍: 命令 vs 实际 ===')
for i in range(min(30, len(cmd) - 1)):
    print(f'  {i:3d} cmd={cmd[i+1,3]:5d} act={act[i+1,3]:5d} '
          f'Δact={int(act[i+1,3]-act[i,3]):+4d}')
