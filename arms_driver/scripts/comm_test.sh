#!/usr/bin/env bash
# comm_test.sh - 伺服失能状态下的通信质量测试（不使能、不运动）
#
# 用法:
#   comm_test.sh [-c arms.json] [-j left|right|all] [-t 秒] [-f "1000 2000"] [-r CPU]
#
# 时长 1..86400，仅接受整数；频率可选 2000/1000/500/250/200/125，按给定顺序分别测试。
# 日志: /tmp/arms_comm_<时间>_<随机后缀>.log；任一频率测试失败时返回非零
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
# -t 会拼入一条交互命令；校验后再提权，禁止把选项带入失能测试。
if ! [[ $DURATION =~ ^[0-9]{1,5}$ ]] || (( 10#$DURATION < 1 || 10#$DURATION > 86400 )); then
  echo "-t 需要 1..86400 的整数秒数" >&2; exit 2
fi
read -r -a rates <<< "$FREQS"
(( ${#rates[@]} )) || { echo "-f 需要至少一个频率" >&2; exit 2; }
for f in "${rates[@]}"; do
  case $f in 2000|1000|500|250|200|125) ;; *) echo "不支持的频率: $f" >&2; exit 2 ;; esac
done
if [[ $EUID -ne 0 ]]; then
  exec sudo "$0" -c "$CONFIG" -j "$JOINTS" -t "$DURATION" -f "$FREQS" ${RT_CPU:+-r "$RT_CPU"}
fi
if pgrep -x motor_console >/dev/null || pgrep -f 'hw_lib_test_' >/dev/null || pgrep -x arms_test >/dev/null; then
  echo "检测到其他 EtherCAT 控制程序正在运行，请先退出" >&2
  exit 1
fi

LOG=$(mktemp --suffix=.log "/tmp/arms_comm_$(date +%Y%m%d_%H%M%S)_XXXXXX") || exit 1
exec > >(tee -a "$LOG") 2>&1
echo "== 通信测试 $(date '+%F %T')  关节 $JOINTS  每轮 ${DURATION}s  频率 $FREQS"

ARGS=(-c "$CONFIG" -j "$JOINTS")
[[ -n $RT_CPU ]] && ARGS+=(-r "$RT_CPU")

rc=0
declare -A RESULT
for f in "${rates[@]}"; do
  echo
  echo "== ${f} Hz"
  if "$ROOT/arms_test" "${ARGS[@]}" -T -f "$f" -e "info" -e "commtest $DURATION" -e "bus"; then
    RESULT[$f]=通过
  else
    rc=1
    RESULT[$f]=未通过
  fi
  sleep 3
done

echo
echo "== 结论"
for f in "${rates[@]}"; do echo "  ${f} Hz: ${RESULT[$f]}"; done
echo "日志: $LOG"
exit "$rc"
