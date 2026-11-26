# 运行资源说明

平台：ASUS NUC 13 Pro（NUC13ANKi7），Intel Core i7-1360P，Ubuntu 22.04，Linux 6.8.0-generic（`PREEMPT_DYNAMIC`，默认 voluntary，`HZ=1000`），EtherLab 1.6.13，`ec_generic` 网卡驱动。

## CPU

| 物理核 | 逻辑 CPU | 类型 | 用途 |
|---|---|---|---|
| P0、P1 | 0–3 | 性能核，超线程 | 普通任务 |
| P2 | 4（5 空闲） | 性能核 | `irq_cpu`：EtherCAT 网口中断与 `ksoftirqd/4` |
| P3 | 6（7 空闲） | 性能核 | `rt_cpu`：周期线程 `arms_rt` |
| E 核 | 8–15 | 能效核，无超线程 | 普通任务 |

保留 CPU 为 `rt_cpu`、`irq_cpu` 及其超线程兄弟（4–7）。同一物理核的两个逻辑 CPU 共用执行单元，兄弟 CPU 保持空闲。

## 线程

| 线程 | 调度 | CPU | 说明 |
|---|---|---|---|
| `arms_rt` | SCHED_FIFO 80（`realtime.priority`） | `realtime.rt_cpu` = 6 | 独立模式下的周期线程；外部驱动模式下不创建，由调用方线程执行 `arms_rx()`/`arms_tx()` |
| `ksoftirqd/<irq_cpu>` | SCHED_FIFO 70（`realtime.softirq_priority`） | `realtime.irq_cpu` = 4 | 由 `rt_setup.sh` 设置，必须低于周期线程优先级 |
| `EtherCAT-OP` 等 EtherLab 内核线程 | SCHED_OTHER | 保留 CPU 以外 | 主站激活后由驱动限制 |
| 主线程及其他线程 | SCHED_OTHER | 保留 CPU 以外 | 终端、配置、监视、指令刷新 |

## rt_setup.sh（运行期设置，退出时还原）

| 项目 | 设置 |
|---|---|
| irqbalance | 暂停 |
| EtherCAT 网口 | 收发队列 4 → 1，接收中断合并 `rx-usecs` → 0；退出 NetworkManager 管理，去掉 IPv4，关闭 IPv6 |
| 网口中断 | `enp86s0` 与 `enp86s0-TxRx-*` 绑定到 `irq_cpu` |
| 其他设备中断 | 限制到保留 CPU 以外（内核管理的中断除外） |
| 线程与非绑定工作队列 | 限制到保留 CPU 以外 |
| 调频 | `rt_cpu`、`irq_cpu` 设为 performance |
| 空闲状态 | `rt_cpu`、`irq_cpu` 关闭退出延迟超过 20 µs 的状态（C2、C3） |
| kerneloops | 暂停 |

长期部署可改用内核启动参数 `preempt=full isolcpus=4-7 nohz_full=6 rcu_nocbs=4-7 irqaffinity=0-3,8-15`。

## 网口

双臂经 EtherCAT 分支器接在 `enp86s0`（Intel I226，`igc`）上，由 EtherLab 主站 0 统一收发，一个周期一帧。

| 链上位置 | 设备 |
|---|---|
| 0 | 分支器（不配置过程数据） |
| 1–7 | 左臂关节 15–21 |
| 8–14 | 右臂关节 22–28 |

集线器为 ZLG ZSW-8004（1 进 3 出）。外壳输出口与 ESC 端口的对应为 OUT1 = 端口 3、OUT2 = 端口 1、OUT3 = 端口 2，
与帧的处理顺序 0 → 3 → 1 → 2 一致，因此 OUT3 日后接入的设备排在右臂之后，不改变左右臂的链上位置。

| 集线器输出口 | ESC 端口 | 设备 |
|---|---|---|
| OUT1 | 3 | 左臂 |
| OUT2 | 1 | 右臂 |
| OUT3 | 2 | 空 |

`arms.left.interface` 与 `arms.right.interface` 相同即为共用链路；此时 `mac`、`master` 也必须相同，链上位置在两臂之间不能重复。

端口绑定由 `arms.hub` 配置：`{ "slave": 0, "left": 1, "right": 2 }` 表示集线器在链上位置 0，左臂接 OUT1，右臂接 OUT2。
启动扫描后驱动读取集线器各端口连接的下一个从站，要求各臂所接输出口连接该臂链上位置最小的驱动器，且该臂驱动器按配置位置依次相连；
不符时启动失败并给出实际所接的输出口（例如“左臂应接 OUT1，实际接在 OUT2”）。被屏蔽或有关节不在线的臂不做核对。
WKC 期望值为 42（14 个驱动器 × 3）。通信失败、离线保护按整条链路处理，两臂同时失能。

## 内存

启动时调用 `mlockall(MCL_CURRENT | MCL_FUTURE)`，周期内不分配内存。

## 外部依赖

| 资源 | 用途 |
|---|---|
| `/dev/EtherCAT0` | EtherLab 主站 0，按网口 MAC 解析主站号 |
| BXI PCI SDK x86-64（`third_party/bxi_pci_drv/x86_64`，`motor_pwr_set`） | `arms_test`、`arms_node` 控制电机总电源。SDK 以 `/tmp/bxi_pci.lock` 独占 PCI；`bxi_pci_exit` 会把 GPIO 输出全部写 0（即断电），本程序只对自己接通的电源断电，释放 PCI 时只调用 `canfd_exit` 和 `bxi_pci_unlock` |
| `/run/arms_rt_setup.state` | `rt_setup.sh` 的还原记录 |

同一主站同时只能被一个程序占用。
