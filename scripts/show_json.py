#!/usr/bin/env python3
"""打印 /tmp/blocks_located.json 的字段结构。"""
import json

d = json.load(open('/tmp/blocks_located.json'))
print('条数', len(d))
print('字段', list(d[0].keys()))
print(json.dumps(d[0], ensure_ascii=False, indent=1))
