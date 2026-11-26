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

| 臂 | 网口 | 状态 |
|---|---|---|
| 左臂 | `enp86s0`（Intel I226，`igc`） | 使用中，EtherLab 主站 0 |
| 右臂 | 未定（配置中为占位名 `eth_right`） | 关节 22–28 由 `safety.ignore_mask = 0x1fc00000` 屏蔽，不打开 |

接入右臂时：在 `arms.right.interface` 填入实际网口，在 `/etc/sysconfig/ethercat` 增加 `MASTER1_DEVICE`，并从 `ignore_mask` 中去掉 22–28。

## 内存

启动时调用 `mlockall(MCL_CURRENT | MCL_FUTURE)`，周期内不分配内存。

## 外部依赖

| 资源 | 用途 |
|---|---|
| `/dev/EtherCAT0` | EtherLab 主站 0，按网口 MAC 解析主站号 |
| BXI PCI SDK x86-64（`third_party/bxi_pci_drv/x86_64`，`motor_pwr_set`） | `arms_test`、`arms_node` 控制电机总电源 |
| `/run/arms_rt_setup.state` | `rt_setup.sh` 的还原记录 |

同一主站同时只能被一个程序占用。
