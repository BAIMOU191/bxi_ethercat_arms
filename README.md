# bxi_ethercat_arms

BXI 双臂 EtherCAT 驱动。RK3588 主控，左右臂共 14 个 KGU 执行器，EtherLab 主站，MIT 模式。
一个实时线程在同一周期内完成两条臂的收发。

| 目录 | 内容 |
|---|---|
| `arms_driver/` | C 库 `libarms_driver.a`、测试工具 `arms_test`、脚本与单元测试 |
| `arms_ros/` | ROS2 节点 |

## 关节编号

| 关节号 | 名称 |
|---|---|
| 15–21 | `l_shoulder_y` `l_shoulder_x` `l_shoulder_z` `l_elbow_y` `l_wrist_x` `l_wrist_y` `l_wrist_z` |
| 22–28 | `r_shoulder_y` `r_shoulder_x` `r_shoulder_z` `r_elbow_y` `r_wrist_x` `r_wrist_y` `r_wrist_z` |

`slave` 为 EtherLab 链上位置，从 0 开始。关节与网口、链上位置的对应关系在 `arms_driver/config/arms.json` 中配置。

## 编译

```bash
cd arms_driver
cmake -S . -B build
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

依赖：EtherLab 1.6（`/usr/local/etherlab`）。aarch64 下链接 `third_party/bxi_pci_drv` 以支持 `--power`。

## 运行

```bash
./build/arms_test -h                           # 启动参数
./build/arms_test --mock                       # 模拟后端
sudo scripts/rt_setup.sh apply                 # 实时运行环境（参数取自配置文件）
sudo ./build/arms_test --power                 # 上电、扫描、预热，进入交互终端
sudo scripts/rt_setup.sh revert
sudo scripts/comm_test.sh -j all -t 300        # 通信质量测试，1000/2000 Hz 各一轮
```

交互终端中输入 `help` 查看命令，`<命令> -h` 查看详细用法。

不要与 `motor_console`、`hw_lib_test_*` 或其他占用同一主站的程序同时运行。

## 配置

`realtime` 段：

| 字段 | 说明 |
|---|---|
| `mode` | `standalone`：驱动自建实时线程；`external`：由调用方实时循环调用 `arms_rx()`/`arms_tx()` |
| `rt_cpu` | 实时线程 CPU，-1 表示不绑定 |
| `irq_cpu` | EtherCAT 网口中断 CPU，由 `rt_setup.sh` 使用 |
| `priority` | 实时线程 SCHED_FIFO 优先级 |
| `softirq_priority` | `ksoftirqd/<irq_cpu>` 的 SCHED_FIFO 优先级，须低于 `priority` |

## 外部驱动模式

```c
arms_options opt;
arms_options_init(&opt);
opt.mode = ARMS_MODE_EXTERNAL;
arms_open(&opt, &arms);

/* 调用方实时循环，周期 arms_period_ns(arms) */
for (;;) {
    wait_next_period();
    arms_rx(arms, now_ns());
    /* 读取反馈、计算、写入目标 */
    arms_tx(arms, now_ns());
}
```

`arms_ready()` 返回 true 表示全部关节已进入 OP 且通信稳定。
