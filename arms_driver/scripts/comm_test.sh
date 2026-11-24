#!/usr/bin/env bash
# comm_test.sh - 伺服失能状态下的通信质量测试（不使能、不运动）
#
# 用法:
#   sudo comm_test.sh [-c arms.json] [-j left|right|all] [-t 秒] [-f "1000 2000"] [-r CPU]
#
# 流程: 启动 EtherLab 主站（未启动时）-> rt_setup.sh apply -> 按各频率运行
#       arms_test --power -e "commtest <秒>" -> rt_setup.sh revert -> 输出结论。
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
    -h|--help) sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "未知参数 $1（-h 查看用法）" >&2; exit 2 ;;
  esac
done
[[ $EUID -eq 0 ]] || { echo "需要 root 权限: sudo $0" >&2; exit 1; }
if pgrep -x motor_console >/dev/null || pgrep -f 'hw_lib_test_' >/dev/null || pgrep -x arms_test >/dev/null; then
  echo "检测到其他 EtherCAT 控制程序正在运行，请先退出" >&2
  exit 1
fi

LOG=/tmp/arms_comm_$(date +%Y%m%d_%H%M%S).log
exec > >(tee -a "$LOG") 2>&1
echo "== 通信测试 $(date '+%F %T')  关节 $JOINTS  每轮 ${DURATION}s  频率 $FREQS"

if [[ ! -e /sys/module/ec_master/parameters/main_devices ]]; then
  echo "-- 启动 EtherLab 主站"
  /etc/init.d/ethercat start || { echo "EtherLab 主站启动失败" >&2; exit 1; }
  sleep 2
fi
echo "-- 主站绑定: $(cat /sys/module/ec_master/parameters/main_devices)"

SETUP_ARGS=(-c "$CONFIG")
TEST_ARGS=(-c "$CONFIG" -j "$JOINTS")
if [[ -n $RT_CPU ]]; then
  SETUP_ARGS+=(--rt-cpu "$RT_CPU" --irq-cpu "$RT_CPU")
  TEST_ARGS+=(-r "$RT_CPU")
fi
"$ROOT/scripts/rt_setup.sh" apply "${SETUP_ARGS[@]}" || exit 1
trap '"$ROOT/scripts/rt_setup.sh" revert' EXIT

declare -A RESULT
for f in $FREQS; do
  echo
  echo "== ${f} Hz"
  if "$ROOT/build/arms_test" "${TEST_ARGS[@]}" -f "$f" --power --power-wait 3000 \
       -e "info" -e "commtest $DURATION" -e "bus"; then
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
