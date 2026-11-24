#!/usr/bin/env bash
# comm_test.sh - 伺服失能状态下的通信质量测试（不使能、不运动）
#
# 用法:
#   comm_test.sh [-c arms.json] [-j left|right|all] [-t 秒] [-f "1000 2000"] [-r CPU]
#
# 按各频率依次运行 arms_test -e "commtest <秒>"，最后输出结论。
# 日志: /tmp/arms_comm_<时间>.log
set -uo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
CONFIG="$ROOT/config/arms.json"
JOINTS=all
DURATION=300
FREQS="1000 2000"
RT_CPU=""

while [[ $# -gt 0 ]]; do
  case $1 in
    -c) CONFIG=$2; shift 2 ;;
    -j) JOINTS=$2; shift 2 ;;
    -t) DURATION=$2; shift 2 ;;
    -f) FREQS=$2; shift 2 ;;
    -r) RT_CPU=$2; shift 2 ;;
    -h|--help) sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "未知参数 $1（-h 查看用法）" >&2; exit 2 ;;
  esac
done
if [[ $EUID -ne 0 ]]; then
  exec sudo "$0" -c "$CONFIG" -j "$JOINTS" -t "$DURATION" -f "$FREQS" ${RT_CPU:+-r "$RT_CPU"}
fi
if pgrep -x motor_console >/dev/null || pgrep -f 'hw_lib_test_' >/dev/null || pgrep -x arms_test >/dev/null; then
  echo "检测到其他 EtherCAT 控制程序正在运行，请先退出" >&2
  exit 1
fi

LOG=/tmp/arms_comm_$(date +%Y%m%d_%H%M%S).log
exec > >(tee -a "$LOG") 2>&1
echo "== 通信测试 $(date '+%F %T')  关节 $JOINTS  每轮 ${DURATION}s  频率 $FREQS"

ARGS=(-c "$CONFIG" -j "$JOINTS")
[[ -n $RT_CPU ]] && ARGS+=(-r "$RT_CPU")

declare -A RESULT
for f in $FREQS; do
  echo
  echo "== ${f} Hz"
  if "$ROOT/arms_test" "${ARGS[@]}" -T -f "$f" -e "info" -e "commtest $DURATION" -e "bus"; then
    RESULT[$f]=通过
  else
    RESULT[$f]=未通过
  fi
  sleep 3
done

echo
echo "== 结论"
for f in $FREQS; do echo "  ${f} Hz: ${RESULT[$f]}"; done
echo "日志: $LOG"
