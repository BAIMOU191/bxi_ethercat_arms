#!/usr/bin/env bash
# phase1_accept.sh - 阶段 1 验收：失能通信测试（不使能、不运动）
#
#   sudo scripts/phase1_accept.sh [-j left|right|all] [-t 秒] [-f "1000 2000"]
#
# 流程: 启动 EtherLab 主站（如未启动）→ rt_setup apply → 依次以各频率运行
#       arms_test --power -e "commtest <秒>" → rt_setup revert → 汇总
# 结果日志: /tmp/arms_phase1_<时间>.log
set -uo pipefail

HERE=$(cd "$(dirname "$0")/.." && pwd)
JOINTS=left
SECONDS_PER_RUN=300
FREQS="1000 2000"

while [[ $# -gt 0 ]]; do
  case $1 in
    -j) JOINTS=$2; shift 2 ;;
    -t) SECONDS_PER_RUN=$2; shift 2 ;;
    -f) FREQS=$2; shift 2 ;;
    -h|--help) sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "未知参数 $1（-h 查看用法）" >&2; exit 2 ;;
  esac
done
[[ $EUID -eq 0 ]] || { echo "需要 root：sudo $0" >&2; exit 1; }
if pgrep -x motor_console >/dev/null || pgrep -f 'hw_lib_test_' >/dev/null; then
  echo "检测到 motor_console 或 hw_lib_test_* 正在运行；先退出它们" >&2; exit 1
fi

LOG=/tmp/arms_phase1_$(date +%Y%m%d_%H%M%S).log
exec > >(tee -a "$LOG") 2>&1
echo "== 阶段 1 验收 $(date '+%F %T')  关节=$JOINTS  每轮 ${SECONDS_PER_RUN}s  频率=$FREQS"

if [[ ! -e /sys/module/ec_master/parameters/main_devices ]]; then
  echo "-- 启动 EtherLab 主站"
  /etc/init.d/ethercat start || { echo "EtherLab 主站启动失败" >&2; exit 1; }
  sleep 2
fi
echo "-- 主站绑定: $(cat /sys/module/ec_master/parameters/main_devices)"

"$HERE/scripts/rt_setup.sh" apply
trap '"$HERE/scripts/rt_setup.sh" revert' EXIT

declare -A RESULT
n=0
for f in $FREQS; do
  n=$((n + 1))
  echo
  echo "== ${f} Hz"
  if "$HERE/build/arms_test" -c "$HERE/config/arms.json" -f "$f" -j "$JOINTS" --power --power-wait 3000 \
       -e "info" -e "commtest $SECONDS_PER_RUN" -e "bus"; then
    RESULT[$f]=通过
  else
    RESULT[$f]=未通过
  fi
  sleep 3
done

echo
echo "== 汇总"
for f in $FREQS; do echo "  ${f} Hz: ${RESULT[$f]}"; done
echo "日志: $LOG"
