#!/usr/bin/env bash
# Keep short desktop checks visible after they finish.
set -uo pipefail
ROOT="${QI_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)}"
action="${1:-sim}"
bash "$ROOT/scripts/tools/qianli.sh" "$@"
result=$?
case "$action" in
  check|training-smoke|camera|devices)
    if [[ "$result" == 0 ]]; then
      echo '[OK] 完成。按回车关闭窗口。'
    else
      echo "[ERROR] 检查未通过（退出码 $result），请查看上面的输出。按回车关闭窗口。"
    fi
    read -r _ || true
    ;;
esac
exit "$result"
