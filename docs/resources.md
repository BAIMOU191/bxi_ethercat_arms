# 运行资源说明

平台：RK3588（CPU0–3 Cortex-A55 1.8 GHz，CPU4–7 Cortex-A76 2.3 GHz），Linux 6.1（非 PREEMPT_RT），EtherLab 1.6，`ec_generic` 网卡驱动。

## CPU 与线程

| 线程 | 调度 | CPU | 说明 |
|---|---|---|---|
| `arms_rt` | SCHED_FIFO 80（`realtime.priority`） | `realtime.rt_cpu`，默认 3 | 独立模式下的周期线程；外部驱动模式下不创建，由调用方线程执行 `arms_rx()`/`arms_tx()` |
| `ksoftirqd/<irq_cpu>` | SCHED_FIFO 70（`realtime.softirq_priority`） | `realtime.irq_cpu`，默认 3 | 由 `rt_setup.sh` 设置，必须低于周期线程优先级 |
| 主线程 | SCHED_OTHER | 非实时 CPU | 终端、配置、启动流程 |

实时 CPU 上不应运行普通任务。`rt_setup.sh apply` 在运行期把其他线程和非绑定工作队列限制到其余 CPU；
长期部署建议改用内核启动参数 `isolcpus=<rt_cpu> nohz_full=<rt_cpu> irqaffinity=<其余 CPU>`。

## 中断

| 网口 | 中断（本机） | 绑定 |
|---|---|---|
| eth0 | 83、84 | `irq_cpu` |
| eth1 | 105、106 | `irq_cpu` |

中断号随内核和设备树变化，`rt_setup.sh` 按 `/proc/interrupts` 中的网口名查找，不写死。
eth0、eth1 由 EtherLab 主站独占，只承载 EtherCAT 帧。

## 周期耗时（实测）

条件：左右臂共 14 个关节，伺服失能，CPU3，`rt_setup.sh` 已应用，每轮 300 s。日志见 `test_logs/comm_all_cpu3_1000_2000_20261124_1226.log`。

| 频率 | 周期数 | 单周期最大耗时 | 唤醒最大延迟 | 周期抖动 RMS | 周期范围 | WKC 异常 |
|---|---|---|---|---|---|---|
| 1000 Hz | 300 319 | 100.0 µs | 23.4 µs | 0.63 µs | 989.3–1010.4 µs | 0 |
| 2000 Hz | 600 606 | 76.7 µs | 67.9 µs | 0.64 µs | 445.7–554.8 µs | 0 |

单臂收发耗时最大值：接收 38.2 µs，发送 48.4 µs。按最大耗时计，2000 Hz 下周期线程占用实时 CPU 不超过约 15%。

## 内存

启动时调用 `mlockall(MCL_CURRENT | MCL_FUTURE)`，常驻并锁定约 20 MB（含线程栈预留）。周期内不分配内存。

## 外部依赖

| 资源 | 用途 |
|---|---|
| `/dev/EtherCAT0`、`/dev/EtherCAT1` | EtherLab 主站 0/1，按网口 MAC 解析主站号 |
| BXI PCI SDK（`motor_pwr_set`） | `arms_test` 默认管理电源；`arms_ros` 仅在 `manage_power=true` 的实机会话接管。核心 C 库本身不操作电源 |
| `/run/arms_rt_setup.state` | `rt_setup.sh` 的还原记录 |

同一主站同时只能被一个程序占用；`motor_console`、`hw_lib_test_*` 等程序运行时无法打开。
