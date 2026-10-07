#!/bin/bash

QI_PROJECT_ROOT="${QI_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
export QI_PROJECT_ROOT
source "$QI_PROJECT_ROOT/scripts/setup/source_env.sh" || exit 1
# 部署修复版 check_at_00.py + watch.sh，保留现有参考掩码，重启值守
bash /tmp/stop_all.sh
cd "$QI_PROJECT_ROOT/scripts" || exit 1
cp /tmp/check_at_00.py /tmp/watch.sh . || exit 1
python3 -m py_compile "$QI_PROJECT_ROOT/scripts/check_at_00.py" && echo "check_at_00.py 语法OK"
echo "现有参考掩码：$(python3 -c "
import cv2,os
p=os.path.join(os.environ['QI_PROJECT_ROOT'], 'config/at00_mask.png')
m=cv2.imread(p,cv2.IMREAD_GRAYSCALE)
print('(none)' if m is None else f'{cv2.countNonZero(m)} px')" 2>/dev/null)"
setsid bash watch.sh 2 0 > /tmp/watch_boot.log 2>&1 < /dev/null &
sleep 8
echo "--- 值守日志 ---"
tail -5 /tmp/watch.log
pgrep -f 'bash watch.sh' >/dev/null && echo "值守在跑" || echo "值守未运行"
exit 0
