#!/bin/bash

QI_PROJECT_ROOT="${QI_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
export QI_PROJECT_ROOT
source "$QI_PROJECT_ROOT/scripts/setup/source_env.sh" || exit 1
# 设备恢复后全面验证
cd "$QI_PROJECT_ROOT/scripts" || exit 1
echo "=== 1) 串口 ==="
bash /tmp/check_arm.sh
echo
echo "=== 2) 相机取帧 ==="
python3 - <<'EOF' 2>&1 | grep -vE 'Warning|warn|VIDIOC|obsensor' | tail -3
import cv2
from project_paths import default_camera
cap = cv2.VideoCapture(default_camera())
ok, img = cap.read()
cap.release()
print('取帧', 'OK' if ok and img is not None else 'FAIL')
if ok:
    cv2.imwrite('/tmp/recheck.jpg', img)
    print('帧尺寸', img.shape[1], 'x', img.shape[0])
EOF
echo
echo "=== 3) (0,0) 掩码检查 ==="
python3 "$QI_PROJECT_ROOT/scripts/check_at_00.py" 0.10 2>/dev/null
echo "=== 4) 参考掩码面积 ==="
python3 - <<'EOF' 2>/dev/null
import cv2, os
p = os.path.join(os.environ['QI_PROJECT_ROOT'], 'config/at00_mask.png')
m = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
print('(none)' if m is None else f'{cv2.countNonZero(m)} px')
EOF
exit 0
