#!/usr/bin/env bash
# rt_setup.sh - prepare (and undo) the realtime environment for arms_test / arms_node.
#
#   sudo scripts/rt_setup.sh apply  [--rt-cpu 5] [--irq-cpu 5] [--ifaces eth0,eth1]
#   sudo scripts/rt_setup.sh revert
#   scripts/rt_setup.sh status
#
# apply:  EtherCAT NIC IRQs -> irq_cpu, ksoftirqd/<irq_cpu> -> SCHED_FIFO 80,
#         every other task and the unbound workqueues -> off rt_cpu/irq_cpu,
#         cpufreq governor of those CPUs -> performance, kerneloops stopped.
# revert: restores everything recorded in the state file.
# These are runtime settings only; a reboot also restores the defaults.
set -euo pipefail

STATE=/run/arms_rt_setup.state
RT_CPU=5
IRQ_CPU=5
IFACES="eth0,eth1"
FIFO_PRIO=80

usage() { sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//'; }

need_root() { [[ $EUID -eq 0 ]] || { echo "需要 root：sudo $0 $*" >&2; exit 1; }; }

cpu_list_without() { # all online CPUs except the given ones -> list "0-3,6,7"
  local skip=" $* " out=() c
  for c in $(seq 0 $(( $(nproc --all) - 1 ))); do
    [[ $skip == *" $c "* ]] || out+=("$c")
  done
  (IFS=,; echo "${out[*]}")
}

cpu_mask_without() { # hex mask of all CPUs except the given ones
  local skip=" $* " m=0 c
  for c in $(seq 0 $(( $(nproc --all) - 1 ))); do
    [[ $skip == *" $c "* ]] || m=$(( m | (1 << c) ))
  done
  printf '%x' "$m"
}

irqs_for() { awk -v ifc="$1" '$NF==ifc {sub(":","",$1); print $1}' /proc/interrupts; }

apply() {
  need_root apply
  [[ -e $STATE ]] && { echo "已有未还原的设置（$STATE）；先运行 revert" >&2; exit 1; }
  local housekeeping; housekeeping=$(cpu_list_without "$RT_CPU" "$IRQ_CPU")
  : > "$STATE"
  echo "rt_cpu=$RT_CPU irq_cpu=$IRQ_CPU ifaces=$IFACES"
  IFS=, read -ra ifs <<< "$IFACES"
  for ifc in "${ifs[@]}"; do
    for irq in $(irqs_for "$ifc"); do
      echo "irq $irq $(cat /proc/irq/$irq/smp_affinity_list)" >> "$STATE"
      echo "$IRQ_CPU" > /proc/irq/$irq/smp_affinity_list
      echo "  IRQ $irq ($ifc) -> CPU $IRQ_CPU"
    done
  done
  local ks; ks=$(pgrep -x "ksoftirqd/$IRQ_CPU")
  echo "ksoftirqd $ks $(chrt -p "$ks" | awk -F'[:：]' 'NR==1{print $2}' | tr -d ' ') $(chrt -p "$ks" | awk -F'[:：]' 'NR==2{print $2}' | tr -d ' ')" >> "$STATE"
  chrt -f -p "$FIFO_PRIO" "$ks"
  echo "  ksoftirqd/$IRQ_CPU (pid $ks) -> SCHED_FIFO $FIFO_PRIO"
  if [[ -w /sys/devices/virtual/workqueue/cpumask ]]; then
    echo "wq $(cat /sys/devices/virtual/workqueue/cpumask)" >> "$STATE"
    cpu_mask_without "$RT_CPU" "$IRQ_CPU" > /sys/devices/virtual/workqueue/cpumask
  fi
  local moved=0 tid aff
  for tid in $(ps -eLo tid=); do
    aff=$(taskset -pc "$tid" 2>/dev/null | awk -F': ' '{print $2}') || continue
    [[ -z $aff ]] && continue
    if taskset -pc "$housekeeping" "$tid" >/dev/null 2>&1; then
      echo "task $tid $aff" >> "$STATE"; moved=$((moved + 1))
    fi
  done
  echo "  已把 $moved 个线程限制到 CPU $housekeeping"
  for c in $(echo "$RT_CPU $IRQ_CPU" | tr ' ' '\n' | sort -u); do
    local g=/sys/devices/system/cpu/cpu$c/cpufreq/scaling_governor
    if [[ -w $g ]]; then echo "gov $c $(cat "$g")" >> "$STATE"; echo performance > "$g"; fi
  done
  if systemctl is-active --quiet kerneloops 2>/dev/null; then
    echo "svc kerneloops" >> "$STATE"; systemctl stop kerneloops
    echo "  kerneloops 已暂停"
  fi
  echo "完成。测试结束后运行: sudo $0 revert"
}

revert() {
  need_root revert
  [[ -e $STATE ]] || { echo "没有需要还原的设置"; exit 0; }
  local kind a b c restored=0 gone=0
  while read -r kind a b c; do
    case $kind in
      irq) echo "$b" > /proc/irq/$a/smp_affinity_list ;;
      ksoftirqd)
        case $b in
          SCHED_OTHER) chrt -o -p 0 "$a" ;;
          SCHED_FIFO) chrt -f -p "${c:-1}" "$a" ;;
          *) chrt -o -p 0 "$a" ;;
        esac ;;
      wq) echo "$a" > /sys/devices/virtual/workqueue/cpumask ;;
      task) if taskset -pc "$b" "$a" >/dev/null 2>&1; then restored=$((restored+1)); else gone=$((gone+1)); fi ;;
      gov) echo "$b" > /sys/devices/system/cpu/cpu$a/cpufreq/scaling_governor ;;
      svc) systemctl start "$a" ;;
    esac
  done < "$STATE"
  rm -f "$STATE"
  echo "已还原（线程 $restored 个，已退出 $gone 个）"
}

status() {
  echo "状态文件: $([[ -e $STATE ]] && echo "$STATE（已应用）" || echo "无（未应用）")"
  IFS=, read -ra ifs <<< "$IFACES"
  for ifc in "${ifs[@]}"; do
    for irq in $(irqs_for "$ifc"); do
      echo "IRQ $irq ($ifc): CPU $(cat /proc/irq/$irq/effective_affinity_list 2>/dev/null || cat /proc/irq/$irq/smp_affinity_list)"
    done
  done
  local ks; ks=$(pgrep -x "ksoftirqd/$IRQ_CPU" || true)
  [[ -n $ks ]] && echo "ksoftirqd/$IRQ_CPU: $(chrt -p "$ks" | tr '\n' ' ')"
  [[ -r /sys/devices/virtual/workqueue/cpumask ]] && echo "workqueue cpumask: $(cat /sys/devices/virtual/workqueue/cpumask)"
  echo "kerneloops: $(systemctl is-active kerneloops 2>/dev/null || true)"
}

cmd=${1:-}; shift || true
while [[ $# -gt 0 ]]; do
  case $1 in
    --rt-cpu) RT_CPU=$2; shift 2 ;;
    --irq-cpu) IRQ_CPU=$2; shift 2 ;;
    --ifaces) IFACES=$2; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数 $1" >&2; usage; exit 2 ;;
  esac
done
case $cmd in
  apply) apply ;;
  revert) revert ;;
  status) status ;;
  -h|--help|"") usage ;;
  *) echo "未知子命令 $cmd" >&2; usage; exit 2 ;;
esac
