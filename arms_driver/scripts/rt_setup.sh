#!/usr/bin/env bash
# rt_setup.sh - 配置/还原双臂 EtherCAT 实时运行环境（x86 NUC，运行时设置，重启后失效）
#
# 用法:
#   sudo rt_setup.sh apply  [-c arms.json] [--rt-cpu N] [--irq-cpu N] [--ifaces enp86s0] [--softirq-prio P]
#   sudo rt_setup.sh revert
#   rt_setup.sh status [-c arms.json]
#
# apply:  暂停 irqbalance；EtherCAT 网口收发队列减为 1 个、关闭中断合并；网口中断绑定到 irq_cpu，
#         其他设备中断移出保留 CPU；ksoftirqd/<irq_cpu> 设为 SCHED_FIFO；保留 CPU（rt_cpu、irq_cpu
#         及其超线程兄弟）以外运行其他线程和非绑定工作队列；rt_cpu、irq_cpu 调频设为 performance，
#         关闭退出延迟超过 20 us 的空闲状态；暂停 kerneloops；EtherCAT 网口退出 NetworkManager 管理、
#         去掉 IPv4 地址、关闭 IPv6，链路保持 UP。参数默认取配置文件 realtime 段和 arms 段，命令行参数优先。
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
IDLE_MAX_LATENCY_US=20

usage() { sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'; }

need_root() { [[ $EUID -eq 0 ]] || { echo "需要 root 权限: sudo $0 $1" >&2; exit 1; }; }

default_config() {
  local c
  for c in "${ARMS_CONFIG:-}" /etc/bxi/arms.json "$SELF_DIR/../config/arms.json"; do
    [[ -n $c && -r $c ]] && { echo "$c"; return; }
  done
}

load_config() {
  [[ -z $CONFIG ]] && CONFIG=$(default_config)
  [[ -z $CONFIG ]] && return
  [[ -r $CONFIG ]] || { echo "无法读取配置文件 $CONFIG" >&2; exit 1; }
  local vals
  vals=$(python3 - "$CONFIG" <<'PY'
import json, sys
c = json.load(open(sys.argv[1]))
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

# CPU 列表 "4-5,8" 展开为 "4 5 8"
expand_list() {
  local part out=()
  IFS=, read -ra parts <<< "$1"
  for part in "${parts[@]}"; do
    if [[ $part == *-* ]]; then out+=($(seq "${part%-*}" "${part#*-}")); else out+=("$part"); fi
  done
  echo "${out[*]}"
}

# 保留 CPU：rt_cpu、irq_cpu 及其超线程兄弟（同一物理核的另一个逻辑 CPU 保持空闲）
reserved_cpus() {
  local c f out=""
  for c in "$RT_CPU" "$IRQ_CPU"; do
    f=/sys/devices/system/cpu/cpu$c/topology/thread_siblings_list
    if [[ -r $f ]]; then out+=" $(expand_list "$(cat "$f")")"; else out+=" $c"; fi
  done
  printf '%s\n' $out | sort -nu | tr '\n' ' '
}

cpu_list_without() {
  local skip=" $* " out=() c
  for c in $(seq 0 $(( $(nproc --all) - 1 ))); do
    [[ $skip == *" $c "* ]] || out+=("$c")
  done
  (IFS=,; echo "${out[*]}")
}

cpu_mask_without() {
  local skip=" $* " m=0 c
  for c in $(seq 0 $(( $(nproc --all) - 1 ))); do
    [[ $skip == *" $c "* ]] || m=$(( m | (1 << c) ))
  done
  printf '%x' "$m"
}

# 网口的全部中断：名称为网口名，或以 "网口名-" 开头（igc 的 MSI-X 队列中断，如 enp86s0-TxRx-0）
irqs_for() {
  awk -v ifc="$1" '$NF==ifc || index($NF, ifc "-")==1 {sub(":","",$1); print $1}' /proc/interrupts
}

# NetworkManager 是否在管理该网口（nmcli 不可用或未运行时视为不管理）
nm_managed() {
  command -v nmcli >/dev/null 2>&1 || return 1
  local st; st=$(nmcli -t -f DEVICE,STATE device 2>/dev/null | awk -F: -v d="$1" '$1==d {print $2}')
  [[ -n $st && $st != unmanaged ]]
}

# 网口收发队列减为 1 个并关闭接收中断合并；须在绑定中断之前执行（修改队列数会重新分配中断）
tune_nic() {
  local ifc=$1 cur usecs
  [[ -d /sys/class/net/$ifc ]] || return 0
  command -v ethtool >/dev/null 2>&1 || { echo "  未安装 ethtool，跳过 $ifc 队列和中断合并设置"; return 0; }
  cur=$(ethtool -l "$ifc" 2>/dev/null | awk '/Current hardware settings/{f=1} f && /^Combined:/{print $2; exit}' || true)
  if [[ $cur =~ ^[0-9]+$ && $cur -gt 1 ]]; then
    if ethtool -L "$ifc" combined 1 2>/dev/null; then
      echo "chan $ifc $cur" >> "$STATE"
      echo "  $ifc 收发队列 $cur -> 1"
    fi
  fi
  usecs=$(ethtool -c "$ifc" 2>/dev/null | awk '/^rx-usecs:/{print $2; exit}' || true)
  if [[ $usecs =~ ^[0-9]+$ && $usecs -ne 0 ]]; then
    if ethtool -C "$ifc" rx-usecs 0 2>/dev/null; then
      echo "coal $ifc $usecs" >> "$STATE"
      echo "  $ifc 接收中断合并 rx-usecs $usecs -> 0"
    fi
  fi
}

# 主机网络栈停用 EtherCAT 网口；EtherLab 通用驱动收发原始帧，不使用 IP 地址
isolate_iface() {
  local ifc=$1 addr v6=/proc/sys/net/ipv6/conf/$1/disable_ipv6
  [[ -d /sys/class/net/$ifc ]] || return 0
  if nm_managed "$ifc"; then
    echo "nm $ifc managed" >> "$STATE"
    nmcli device set "$ifc" managed no
    echo "  $ifc 退出 NetworkManager 管理"
  else
    # 记录不由 NetworkManager 管理的地址，还原时重新添加
    for addr in $(ip -4 -o addr show dev "$ifc" | awk '{print $4}'); do
      echo "addr4 $ifc $addr" >> "$STATE"
    done
  fi
  ip -4 addr flush dev "$ifc"
  ip link set "$ifc" up # EtherLab 通用驱动要求网口处于 UP 状态
  if [[ -w $v6 ]]; then
    echo "ipv6 $ifc $(cat "$v6")" >> "$STATE"
    echo 1 > "$v6"
  fi
  echo "  $ifc 已去掉 IPv4 地址并关闭 IPv6（链路保持 UP）"
}

check_params() {
  local n; n=$(nproc --all)
  [[ $RT_CPU =~ ^[0-9]+$ && $RT_CPU -lt $n ]] || { echo "rt_cpu 无效: '$RT_CPU'" >&2; exit 2; }
  [[ $IRQ_CPU =~ ^[0-9]+$ && $IRQ_CPU -lt $n ]] || { echo "irq_cpu 无效: '$IRQ_CPU'" >&2; exit 2; }
  [[ $SOFTIRQ_PRIO =~ ^[0-9]+$ && $SOFTIRQ_PRIO -ge 1 && $SOFTIRQ_PRIO -le 98 ]] ||
    { echo "softirq 优先级无效: '$SOFTIRQ_PRIO'" >&2; exit 2; }
  [[ -n $IFACES ]] || { echo "未指定网口" >&2; exit 2; }
}

stop_service() {
  if systemctl is-active --quiet "$1" 2>/dev/null; then
    echo "svc $1" >> "$STATE"
    systemctl stop "$1"
    echo "  $1 已暂停"
  fi
}

apply() {
  need_root apply
  [[ -e $STATE ]] && { echo "存在未还原的设置 ($STATE)，需先执行 revert" >&2; exit 1; }
  load_config
  check_params
  local reserved housekeeping
  reserved=$(reserved_cpus)
  housekeeping=$(cpu_list_without $reserved)
  : > "$STATE"
  echo "配置: ${CONFIG:-无}  实时 CPU $RT_CPU  中断 CPU $IRQ_CPU  保留 CPU ${reserved% }  网口 $IFACES"
  stop_service irqbalance
  IFS=, read -ra ifs <<< "$IFACES"
  for ifc in "${ifs[@]}"; do
    tune_nic "$ifc"
  done
  local ecat_irqs="" irq
  for ifc in "${ifs[@]}"; do
    for irq in $(irqs_for "$ifc"); do
      echo "irq $irq $(cat /proc/irq/$irq/smp_affinity_list)" >> "$STATE"
      echo "$IRQ_CPU" > /proc/irq/$irq/smp_affinity_list
      ecat_irqs+=" $irq "
      echo "  中断 $irq ($ifc) -> CPU $IRQ_CPU"
    done
  done
  # 其他设备中断移出保留 CPU；内核管理的中断（如 NVMe 每核队列）不可修改，跳过
  local moved_irq=0 f aff
  for f in /proc/irq/[0-9]*/smp_affinity_list; do
    irq=$(basename "$(dirname "$f")")
    [[ $ecat_irqs == *" $irq "* ]] && continue
    aff=$(cat "$f" 2>/dev/null) || continue
    [[ $aff == "$housekeeping" ]] && continue
    if echo "$housekeeping" > "$f" 2>/dev/null; then
      echo "irq $irq $aff" >> "$STATE"
      moved_irq=$((moved_irq + 1))
    fi
  done
  echo "  $moved_irq 个其他中断限制到 CPU $housekeeping"
  local ks pol prio
  ks=$(pgrep -x "ksoftirqd/$IRQ_CPU")
  pol=$(chrt -p "$ks" | awk -F': ' 'NR==1{print $2}')
  prio=$(chrt -p "$ks" | awk -F': ' 'NR==2{print $2}')
  echo "ksoftirqd $ks $pol $prio" >> "$STATE"
  chrt -f -p "$SOFTIRQ_PRIO" "$ks"
  echo "  ksoftirqd/$IRQ_CPU -> SCHED_FIFO $SOFTIRQ_PRIO"
  if [[ -w /sys/devices/virtual/workqueue/cpumask ]]; then
    echo "wq $(cat /sys/devices/virtual/workqueue/cpumask)" >> "$STATE"
    cpu_mask_without $reserved > /sys/devices/virtual/workqueue/cpumask
  fi
  local moved=0 tid
  for tid in $(ps -eLo tid=); do
    aff=$(taskset -pc "$tid" 2>/dev/null | awk -F': ' '{print $2}') || continue
    [[ -z $aff ]] && continue
    if taskset -pc "$housekeeping" "$tid" >/dev/null 2>&1; then
      echo "task $tid $aff" >> "$STATE"
      moved=$((moved + 1))
    fi
  done
  echo "  $moved 个线程限制到 CPU $housekeeping"
  local c g s disabled=0
  for c in $(printf '%s\n' "$RT_CPU" "$IRQ_CPU" | sort -u); do
    g=/sys/devices/system/cpu/cpu$c/cpufreq/scaling_governor
    if [[ -w $g ]]; then
      echo "gov $c $(cat "$g")" >> "$STATE"
      echo performance > "$g"
    fi
    for s in /sys/devices/system/cpu/cpu$c/cpuidle/state[0-9]*; do
      [[ -w $s/disable ]] || continue
      (( $(cat "$s/latency") > IDLE_MAX_LATENCY_US )) || continue
      [[ $(cat "$s/disable") == 0 ]] || continue
      echo "idle $c $(basename "$s")" >> "$STATE"
      echo 1 > "$s/disable"
      disabled=$((disabled + 1))
    done
  done
  echo "  CPU $RT_CPU、$IRQ_CPU 调频 performance，关闭 $disabled 个退出延迟超过 $IDLE_MAX_LATENCY_US us 的空闲状态"
  stop_service kerneloops
  for ifc in "${ifs[@]}"; do
    isolate_iface "$ifc"
  done
  echo "完成"
}

revert() {
  need_root revert
  [[ -e $STATE ]] || { echo "没有需要还原的设置"; exit 0; }
  local kind a b c restored=0 gone=0
  # 逆序还原：网口队列在中断亲和性之后恢复，irqbalance 最后启动
  while read -r kind a b c; do
    case $kind in
      irq) echo "$b" > /proc/irq/$a/smp_affinity_list 2>/dev/null || true ;;
      ksoftirqd)
        if [[ $b == SCHED_FIFO || $b == SCHED_RR ]]; then chrt -f -p "${c:-1}" "$a"; else chrt -o -p 0 "$a"; fi ;;
      wq) echo "$a" > /sys/devices/virtual/workqueue/cpumask ;;
      task)
        if taskset -pc "$b" "$a" >/dev/null 2>&1; then restored=$((restored + 1)); else gone=$((gone + 1)); fi ;;
      gov) echo "$b" > /sys/devices/system/cpu/cpu$a/cpufreq/scaling_governor ;;
      idle) echo 0 > /sys/devices/system/cpu/cpu$a/cpuidle/$b/disable ;;
      svc) systemctl start "$a" ;;
      ipv6) [[ -w /proc/sys/net/ipv6/conf/$a/disable_ipv6 ]] && echo "$b" > /proc/sys/net/ipv6/conf/$a/disable_ipv6 ;;
      addr4) ip -4 addr add "$b" dev "$a" 2>/dev/null || true ;;
      nm) nmcli device set "$a" managed yes 2>/dev/null || echo "  $a 交还 NetworkManager 失败" ;;
      coal) ethtool -C "$a" rx-usecs "$b" 2>/dev/null || echo "  $a 中断合并还原失败" ;;
      chan) ethtool -L "$a" combined "$b" 2>/dev/null || echo "  $a 收发队列还原失败" ;;
    esac
  done < <(tac "$STATE")
  rm -f "$STATE"
  echo "已还原（线程 $restored 个，已退出 $gone 个）"
}

status() {
  load_config
  echo "状态: $([[ -e $STATE ]] && echo "已应用 ($STATE)" || echo "未应用")"
  echo "配置: ${CONFIG:-无}  实时 CPU ${RT_CPU:--}  中断 CPU ${IRQ_CPU:--}  网口 ${IFACES:--}"
  if [[ $RT_CPU =~ ^[0-9]+$ && $IRQ_CPU =~ ^[0-9]+$ ]]; then
    echo "  保留 CPU: $(reserved_cpus)"
  fi
  if [[ -n $IFACES ]]; then
    IFS=, read -ra ifs <<< "$IFACES"
    for ifc in "${ifs[@]}"; do
      for irq in $(irqs_for "$ifc"); do
        echo "  中断 $irq ($(awk -v i="$irq:" '$1==i {print $NF}' /proc/interrupts)): CPU $(cat /proc/irq/$irq/effective_affinity_list 2>/dev/null || cat /proc/irq/$irq/smp_affinity_list)"
      done
      if [[ -d /sys/class/net/$ifc ]]; then
        echo "  $ifc: NetworkManager $(nm_managed "$ifc" && echo 管理中 || echo 不管理)  IPv4 $(ip -4 -o addr show dev "$ifc" | awk '{printf "%s ", $4}')  IPv6 $([[ $(cat /proc/sys/net/ipv6/conf/$ifc/disable_ipv6 2>/dev/null) == 1 ]] && echo 关闭 || echo 开启)"
        if command -v ethtool >/dev/null 2>&1; then
          echo "  $ifc: 收发队列 $(ethtool -l "$ifc" 2>/dev/null | awk '/Current hardware settings/{f=1} f && /^Combined:/{print $2; exit}')  rx-usecs $(ethtool -c "$ifc" 2>/dev/null | awk '/^rx-usecs:/{print $2; exit}')"
        fi
      fi
    done
  fi
  if [[ $IRQ_CPU =~ ^[0-9]+$ ]]; then
    local ks; ks=$(pgrep -x "ksoftirqd/$IRQ_CPU" || true)
    [[ -n $ks ]] && echo "  ksoftirqd/$IRQ_CPU: $(chrt -p "$ks" | awk -F': ' '{printf "%s ", $2}')"
  fi
  [[ -r /sys/devices/virtual/workqueue/cpumask ]] && echo "  workqueue cpumask: $(cat /sys/devices/virtual/workqueue/cpumask)"
  echo "  irqbalance: $(systemctl is-active irqbalance 2>/dev/null || true)  kerneloops: $(systemctl is-active kerneloops 2>/dev/null || true)"
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
