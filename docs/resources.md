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

## 网口与中断

双臂经 EtherCAT 集线器 ZLG ZSW-8004（1 进 3 出）接在 `eth0` 上，由 EtherLab 主站 0 统一收发，一个周期一帧；`eth1` 不使用。

| 链上位置 | 设备 |
|---|---|
| 0 | 集线器（不配置过程数据） |
| 1–7 | 左臂关节 15–21（OUT1，ESC 端口 3） |
| 8–14 | 右臂关节 22–28（OUT2，ESC 端口 1） |

集线器外壳输出口与 ESC 端口的对应为 OUT1 = 3、OUT2 = 1、OUT3 = 2，与帧的处理顺序一致，OUT3 日后接入的设备排在右臂之后。
`arms.hub = { "slave": 0, "left": 1, "right": 2 }` 在启动时核对各输出口连接的首个驱动器及各臂驱动器的连接顺序。
该核对只能发现输出口未连接或支路断开；14 个驱动器型号相同，左右臂互换插口无法从拓扑上区分。
WKC 期望值为 42（14 个驱动器 × 3）。通信失败、离线保护按整条链路处理，两臂同时失能。

`eth0` 的中断由 `rt_setup.sh` 按 `/proc/interrupts` 中的网口名查找并绑定到 `irq_cpu`，不写死中断号；运行期间 `eth0` 只承载 EtherCAT 帧。

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
| `/dev/EtherCAT0` | EtherLab 主站 0（`eth0`），按网口 MAC 解析主站号 |
| BXI PCI SDK（`motor_pwr_set`） | `arms_test`、`arms_node` 控制电机总电源，与其他程序共用，谁接通谁断开。SDK 以 `/tmp/bxi_pci.lock` 独占 PCI；`bxi_pci_exit` 会把 GPIO 输出全部写 0（即断电），本程序只对自己接通的电源断电，释放 PCI 时只调用 `canfd_exit` 和 `bxi_pci_unlock`；`arms_power on|off|status` 单独开关电源 |
| `/run/arms_rt_setup.state` | `rt_setup.sh` 的还原记录 |

同一主站同时只能被一个程序占用；`motor_console`、`hw_lib_test_*` 等程序运行时无法打开。
