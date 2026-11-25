#!/usr/bin/env bash
# rt_setup.sh - 配置/还原双臂 EtherCAT 实时运行环境（运行时设置，重启后失效）
#
# 用法:
#   sudo rt_setup.sh apply  [-c arms.json] [--rt-cpu N] [--irq-cpu N] [--ifaces eth0,eth1] [--softirq-prio P]
#   sudo rt_setup.sh revert
#   rt_setup.sh status [-c arms.json]
#
# apply:  EtherCAT 网口中断绑定到 irq_cpu；ksoftirqd/<irq_cpu> 设为 SCHED_FIFO；
#         其他线程和非绑定工作队列限制到其余 CPU；实时 CPU 调频设为 performance；
#         暂停 kerneloops。参数默认取配置文件 realtime 段和 arms 段，命令行参数优先。
#         rt_cpu=-1 时不修改系统设置。
# revert: 按状态文件逐项还原。
set -euo pipefail
export LC_ALL=C

STATE=/run/arms_rt_setup.state
SELF_DIR=$(cd "$(dirname "$0")" && pwd)
CONFIG=""
RT_CPU=""
IRQ_CPU=""
IFACES=""
SOFTIRQ_PRIO=""

usage() { sed -n '/^# /,/^# revert:/p' "$0" | sed 's/^# \{0,1\}//'; }

need_root() { [[ $EUID -eq 0 ]] || { echo "需要 root 权限: sudo $0 $1" >&2; exit 1; }; }

default_config() {
  local c
  for c in "${ARMS_CONFIG:-}" /etc/bxi/arms.json ./config/arms.json "$SELF_DIR/../config/arms.json"; do
    [[ -n $c && -r $c ]] && { echo "$c"; return; }
  done
  return 0
}

load_config() {
  [[ -z $CONFIG ]] && CONFIG=$(default_config)
  [[ -z $CONFIG ]] && return 0
  [[ -r $CONFIG ]] || { echo "无法读取配置文件 $CONFIG" >&2; exit 1; }
  local vals
  vals=$(python3 - "$CONFIG" <<'PY'
import json, sys
with open(sys.argv[1], encoding="utf-8-sig") as stream:
    c = json.load(stream)
rt = c.get("realtime", {})
arms = c.get("arms", {})
ifs = ",".join(a["interface"] for a in (arms.get("left"), arms.get("right")) if a and a.get("interface"))
print(rt.get("rt_cpu", -1), rt.get("irq_cpu", -1), rt.get("softirq_priority", 70), ifs or "-")
PY
)
  read -r c_rt c_irq c_prio c_ifs <<< "$vals"
  [[ -z $RT_CPU ]] && RT_CPU=$c_rt
  [[ -z $IRQ_CPU ]] && IRQ_CPU=$c_irq
  [[ -z $SOFTIRQ_PRIO ]] && SOFTIRQ_PRIO=$c_prio
  [[ -z $IFACES && $c_ifs != "-" ]] && IFACES=$c_ifs
  return 0
}

cpu_list_without() {
  local skip=" $* " out=() c
  for c in $(seq 0 $(( $(nproc --all) - 1 ))); do
    [[ $skip == *" $c "* ]] || out+=("$c")
  done
  (IFS=,; echo "${out[*]}")
}

cpu_mask_without() {
  # 内核 cpumask 是高位在前的 32 位分组，不能用 Bash 单个有符号整数表示大板。
  python3 - "$(nproc --all)" "$@" <<'PY'
import sys
n, *excluded = map(int, sys.argv[1:])
mask = (1 << n) - 1
for cpu in excluded:
    mask &= ~(1 << cpu)
groups = [f'{(mask >> shift) & 0xffffffff:08x}' for shift in range(0, n, 32)]
print(','.join(reversed(groups)))
PY
}

irqs_for() { awk -v ifc="$1" '$NF==ifc {sub(":","",$1); print $1}' /proc/interrupts; }

check_params() {
  local n; n=$(nproc --all)
  [[ $RT_CPU =~ ^[0-9]+$ && $RT_CPU -lt $n ]] || { echo "rt_cpu 无效: '$RT_CPU'" >&2; exit 2; }
  [[ $IRQ_CPU =~ ^[0-9]+$ && $IRQ_CPU -lt $n ]] || { echo "irq_cpu 无效: '$IRQ_CPU'" >&2; exit 2; }
  [[ $SOFTIRQ_PRIO =~ ^[0-9]+$ && $SOFTIRQ_PRIO -ge 1 && $SOFTIRQ_PRIO -le 98 ]] ||
    { echo "softirq 优先级无效: '$SOFTIRQ_PRIO'" >&2; exit 2; }
  [[ -n $IFACES ]] || { echo "未指定网口" >&2; exit 2; }
}

apply() {
  need_root apply
  [[ -e $STATE ]] && { echo "已有未还原的设置 ($STATE)，请先执行 revert" >&2; exit 1; }
  load_config
  if [[ $RT_CPU == -1 ]]; then echo "配置: rt_cpu=-1，跳过 CPU/IRQ 调优"; return 0; fi
  check_params
  local housekeeping; housekeeping=$(cpu_list_without "$RT_CPU" "$IRQ_CPU")
  [[ -n $housekeeping ]] || { echo "没有可用的非实时 CPU，未修改系统设置" >&2; return 2; }
  : > "$STATE"
  echo "配置: ${CONFIG:-无}  实时 CPU $RT_CPU  中断 CPU $IRQ_CPU  网口 $IFACES"
  IFS=, read -ra ifs <<< "$IFACES"
  for ifc in "${ifs[@]}"; do
    for irq in $(irqs_for "$ifc"); do
      echo "irq $irq $(cat /proc/irq/$irq/smp_affinity_list)" >> "$STATE"
      echo "$IRQ_CPU" > /proc/irq/$irq/smp_affinity_list
      echo "  中断 $irq ($ifc) -> CPU $IRQ_CPU"
    done
  done
  local ks pol prio
  ks=$(pgrep -x "ksoftirqd/$IRQ_CPU")
  pol=$(chrt -p "$ks" | awk -F': ' 'NR==1{print $2}')
  prio=$(chrt -p "$ks" | awk -F': ' 'NR==2{print $2}')
  case "$pol" in SCHED_OTHER|SCHED_BATCH|SCHED_IDLE|SCHED_FIFO|SCHED_RR) ;;
    *) echo "不支持还原原调度策略 $pol，未修改该线程" >&2; return 1 ;;
  esac
  echo "ksoftirqd $ks $pol $prio" >> "$STATE"
  chrt -f -p "$SOFTIRQ_PRIO" "$ks"
  echo "  ksoftirqd/$IRQ_CPU -> SCHED_FIFO $SOFTIRQ_PRIO"
  if [[ -w /sys/devices/virtual/workqueue/cpumask ]]; then
    echo "wq $(cat /sys/devices/virtual/workqueue/cpumask)" >> "$STATE"
    cpu_mask_without "$RT_CPU" "$IRQ_CPU" > /sys/devices/virtual/workqueue/cpumask
  fi
  local moved=0 tid aff
  for tid in $(ps -eLo tid=); do
    aff=$(taskset -pc "$tid" 2>/dev/null | awk -F': ' '{print $2}') || continue
    [[ -z $aff ]] && continue
    if taskset -pc "$housekeeping" "$tid" >/dev/null 2>&1; then
      echo "task $tid $aff" >> "$STATE"
      moved=$((moved + 1))
    fi
  done
  echo "  $moved 个线程限制到 CPU $housekeeping"
  for c in $(printf '%s\n' "$RT_CPU" "$IRQ_CPU" | sort -u); do
    local g=/sys/devices/system/cpu/cpu$c/cpufreq/scaling_governor
    if [[ -w $g ]]; then
      echo "gov $c $(cat "$g")" >> "$STATE"
      echo performance > "$g"
    fi
  done
  if systemctl is-active --quiet kerneloops 2>/dev/null; then
    echo "svc kerneloops" >> "$STATE"
    systemctl stop kerneloops
    echo "  kerneloops 已暂停"
  fi
  echo "完成"
}

revert() {
  need_root revert
  [[ -e $STATE ]] || { echo "没有需要还原的设置"; exit 0; }
  local kind a b c restored=0 gone=0
  while read -r kind a b c; do
    case $kind in
      irq) echo "$b" > /proc/irq/$a/smp_affinity_list ;;
      ksoftirqd)
        case "$b" in
          SCHED_FIFO) chrt -f -p "$c" "$a" ;;
          SCHED_RR)   chrt -r -p "$c" "$a" ;;
          SCHED_BATCH) chrt -b -p 0 "$a" ;;
          SCHED_IDLE) chrt -i -p 0 "$a" ;;
          SCHED_OTHER) chrt -o -p 0 "$a" ;;
          *) echo "无法还原调度策略 $b，保留状态文件" >&2; return 1 ;;
        esac ;;
      wq) echo "$a" > /sys/devices/virtual/workqueue/cpumask ;;
      task)
        if taskset -pc "$b" "$a" >/dev/null 2>&1; then restored=$((restored + 1)); else gone=$((gone + 1)); fi ;;
      gov) echo "$b" > /sys/devices/system/cpu/cpu$a/cpufreq/scaling_governor ;;
      svc) systemctl start "$a" ;;
    esac
  done < "$STATE"
  rm -f "$STATE"
  echo "已还原（线程 $restored 个，已退出 $gone 个）"
}

status() {
  load_config
  echo "状态: $([[ -e $STATE ]] && echo "已应用 ($STATE)" || echo "未应用")"
  echo "配置: ${CONFIG:-无}  实时 CPU ${RT_CPU:--}  中断 CPU ${IRQ_CPU:--}  网口 ${IFACES:--}"
  if [[ -n $IFACES ]]; then
    IFS=, read -ra ifs <<< "$IFACES"
    for ifc in "${ifs[@]}"; do
      for irq in $(irqs_for "$ifc"); do
        echo "  中断 $irq ($ifc): CPU $(cat /proc/irq/$irq/effective_affinity_list 2>/dev/null || cat /proc/irq/$irq/smp_affinity_list)"
      done
    done
  fi
  if [[ $IRQ_CPU =~ ^[0-9]+$ ]]; then
    local ks; ks=$(pgrep -x "ksoftirqd/$IRQ_CPU" || true)
    [[ -n $ks ]] && echo "  ksoftirqd/$IRQ_CPU: $(chrt -p "$ks" | awk -F': ' '{printf "%s ", $2}')"
  fi
  [[ -r /sys/devices/virtual/workqueue/cpumask ]] && echo "  workqueue cpumask: $(cat /sys/devices/virtual/workqueue/cpumask)"
  echo "  kerneloops: $(systemctl is-active kerneloops 2>/dev/null || true)"
}

cmd=${1:-}
[[ $# -gt 0 ]] && shift
while [[ $# -gt 0 ]]; do
  case $1 in
    -c|--config) CONFIG=$2; shift 2 ;;
    --rt-cpu) RT_CPU=$2; shift 2 ;;
    --irq-cpu) IRQ_CPU=$2; shift 2 ;;
    --ifaces) IFACES=$2; shift 2 ;;
    --softirq-prio) SOFTIRQ_PRIO=$2; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数 $1" >&2; usage >&2; exit 2 ;;
  esac
done
case $cmd in
  apply) apply ;;
  revert) revert ;;
  status) status ;;
  -h|--help|"") usage ;;
  *) echo "未知子命令 $cmd" >&2; usage >&2; exit 2 ;;
esac
