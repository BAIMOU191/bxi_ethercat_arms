# bxi_ethercat_arms

BXI 双臂 EtherCAT 驱动。RK3588 主控，左右臂共 14 个 KGU 执行器，EtherLab 主站，MIT 模式。
一个实时线程在同一周期内完成两条臂的收发。

| 目录 | 内容 |
|---|---|
| `arms_driver/` | C 库 `libarms_driver.a`、测试工具 `arms_test`、脚本与单元测试 |
| `arms_ros/` | ROS 2 节点、消息桥接、关节服务与隔离域测试 |

## 关节编号

| 关节号 | 名称 |
|---|---|
| 15–21 | `l_shoulder_y` `l_shoulder_x` `l_shoulder_z` `l_elbow_y` `l_wrist_x` `l_wrist_y` `l_wrist_z` |
| 22–28 | `r_shoulder_y` `r_shoulder_x` `r_shoulder_z` `r_elbow_y` `r_wrist_x` `r_wrist_y` `r_wrist_z` |

`slave` 为 EtherLab 链上位置，从 0 开始。关节与网口、链上位置的对应关系在 `arms_driver/config/arms.json` 中配置。

## 运行

```bash
cd arms_driver
./arms_test
```

`./arms_test` 先增量编译，再启动程序；程序在需要时通过 sudo 获取 root 权限，依次启动 EtherLab 主站、
配置实时运行环境、电机上电、扫描并预热，然后进入交互终端（全部关节保持伺服失能）。
`quit` 退出时停止通信、断电并还原实时运行环境。

```bash
./arms_test -h                     # 启动参数
./arms_test --mock                 # 模拟后端，不访问硬件
./arms_test -f 2000 -j left        # 2 kHz，只打开左臂
./arms_test -e "commtest 300"      # 执行命令后退出
./arms_test -T                     # 每一行输出都加时间戳
scripts/comm_test.sh -j all -t 300 # 通信质量测试，1000/2000 Hz 各一轮
```

交互终端中输入 `help` 查看命令，`<命令> -h` 查看详细用法。按 **Tab** 可补全命令、关节和常用选项：唯一匹配会补齐，多个匹配会补公共前缀或列出候选；只有回车才执行。

校零示例：先支撑手臂，`free 15` 后手动停在目标零位，再输入 `zero 15`。已使能就自动失能，已失能不重复操作；驱动确认失能和静止后才写软件零位，完成后不重新使能。`zero 15 --hw` 是硬件校零，不是普通软件校零。

`pn_save 15` 只对 15 号关节按需失能并等待静止，再发送 Pn097 保存脉冲；不检查或改变抱闸模式，也不会重新使能。强制松闸时请先可靠支撑负载。

参数维护：`pn list` 查看已收录的手册参数；`pn help Pn026` 查取值、单位和注意事项。`pn 15 Pn026`、`sdo 15 0x2026 0`、`brake_voltage 15` 都读 15 号制动器保持电压。`torque_scale 15` 读 Pn082，同时显示原始值和 A/N·m。两个别名都可追加**原始整数**写入，也可用作 `pn 15 <别名>`。普通写一次只操作一个已失能关节，不自动保存；目录仅供查询，不是白名单。未列出的 Pn 同样可按编号访问；`sdo` 可按索引、子索引和显式字宽直接读写（例如 `u32:123` 或 `hex:01020304`）。普通写成功只表示从站已接收，可另行读取核对。详见 [参数维护规则](docs/phase4-5.md#pn-参数目录)。

输出中的时间戳为本地时间 `[HH:MM:SS.mmm]`：启动、电源、实时环境、命令开始、通信测试进度等事件行默认带时间戳；
`-T` 使每一行都带时间戳，`scripts/comm_test.sh` 默认启用。

### 运动控制与测试命令

建议先用 `--mock` 核对关节选择和目标值。实机使能前请确认机械臂有足够的运动空间，且可随时急停。
`<关节>` 可写 `all`、`left`、`right`、单个编号或名称、范围 `15-18` 或列表 `21,28`。
位置和幅值单位为 rad，速度为 rad/s，前馈力矩为 N·m；`-m` 的单位为秒，`enable -t` 的单位为毫秒。

| 命令 | 行为 |
|---|---|
| `enable <关节> [-t 毫秒]` | 使能并显示模式和实际位置。省略超时时间或指定 0 时使用驱动默认值。 |
| `disable [关节]` / `reset <关节>` | 失能（默认全部已打开关节）或故障复位。复位后需重新使能才能运动。 |
| `mit <关节> <位置> [kp] [kd] [速度] [前馈]` | 增益默认取各关节配置，速度和前馈默认为零。指令下一周期直接生效，驱动与工具均不做轨迹插值（与参考 hw_lib 的 `set_motor_cmd` 一致），目标较远时关节会快速跳向目标；非零速度或前馈以 100 Hz 持续刷新。 |
| `hold [关节]` | 保持当前**指令位置**，保留增益，清零速度和前馈。若此前是 `free`，`kp` 仍为零，不能靠 `hold` 承重。默认全部已打开关节。 |
| `home [关节]` | 以默认增益直接下发各自的 `home`，不做插值，缺省只选已使能关节。关节停稳（速度持续 0.2 s 低于 0.02 rad/s，最多等 5 s）后报告目标、实际位置及误差；回车或 Ctrl+C 中止并保持。 |
| `swing <关节> <幅值> [周期秒] [次数] [kp] [kd]` | 以当前指令位置为中心，200 Hz 发送正弦位置和对应速度；默认周期 4 秒、3 次、各自默认增益。每秒报告最大跟踪误差，结束时回中心且速度为零；回车或 Ctrl+C 中止时保持当前指令位置。 |
| `power on` / `power off` / `power status` | 仅用于支持 PCI 电源控制的硬件会话。断电前先使已使能关节失能并等待 50 ms。状态反映本工具的电源请求，不是硬件回读。 |
| `commtest <秒> [--enabled]` | 不加选项时要求全部关节已失能。加 `--enabled` 时使能全部已打开关节并保持原位，正常结束或提前中止后全部失能。 |

`mit`、`home`、`swing` 在下发前检查所选关节的使能状态和限位；若任一目标不合法，整组均不下发。
纯位置 `mit`、`hold`、`home`、`swing`、`disable`、`reset` 或失去使能均会停止相应关节的动态指令刷新。
例如，可在模拟会话中依次执行 `enable 17`、`mit 17 0.1`、`home 17`、`disable 17`。

提示符的 `E` 后是已使能关节数。`status` 新列 `cmd`、`kp`、`kd` 为当前指令位置和增益；
`flag` 中 `L` 表示限幅、`T` 表示指令超时，无标志时为 `-`；`flt` 为累计故障次数。
后台以 10 Hz 检查故障计数增加、关节退出使能和臂通信失败。发生变化时输出带时间戳的该臂关节及 WKC 快照。

## 编译与测试

```bash
cmake -S . -B build && cmake --build build -j4
ctest --test-dir build --output-on-failure
```

依赖：EtherLab 1.6（`/usr/local/etherlab`）。aarch64 下链接 `third_party/bxi_pci_drv` 用于电机电源控制。

不要与 `motor_console`、`hw_lib_test_*` 或其他占用同一主站的程序同时运行。

## 配置

`realtime` 段：

| 字段 | 说明 |
|---|---|
| `mode` | `standalone`：驱动自建实时线程；`external`：由调用方实时循环调用 `arms_rx()`/`arms_tx()` |
| `rt_cpu` | 实时线程 CPU，-1 表示不绑定 |
| `irq_cpu` | EtherCAT 网口中断 CPU，由 `scripts/rt_setup.sh` 使用（`arms_test` 启动时自动调用） |
| `priority` | 实时线程 SCHED_FIFO 优先级 |
| `softirq_priority` | `ksoftirqd/<irq_cpu>` 的 SCHED_FIFO 优先级，须低于 `priority` |

`safety` 段中与在线检查相关的字段：

| 字段 | 说明 |
|---|---|
| `check_motors` | `true`：启动时未屏蔽的关节必须全部在线，否则打开失败（`arms_test` 随即下电）；`false`：不在线的关节本次不使用 |
| `ignore_mask` | 31 位屏蔽掩码，第 n 位对应关节 n。被屏蔽的关节不打开、不检查，掉线不触发保护。`arms_test -m` 可覆盖 |
| `offline_action` | `power_off`：运行中未屏蔽的关节掉线时全部失能并锁定保护（`arms_test` 接管电源时请求断电；外部供电须由电源所有者处理），`power on` 重新上电并检查后解除；`disable`：只失能，恢复后自动重连 |
| `overspeed_ms` / `overtorque_ms` | 反馈超限保护的累计时长，默认 500 / 100 ms。使能中的关节反馈速度超过 `limit.vel_fault` 或力矩超过 `limit.torque_fault` 时计数加 1，否则减 1，达到该时长即全部关节失能并锁定，`reset` 清除 |

每个关节的配置项：`joint`、`name`、`arm`、`slave` 必填；`direction`、`offset_rad`、`home`、`limit`（`pos`、`vel`、`tff` 为指令限幅，`vel_fault`、`torque_fault` 为反馈保护阈值）、`gain_max`、`gain_default` 可选。

EtherCAT 为串联链路，链中一个电机断电通常会使其后的电机一并离线，屏蔽链中间的关节不能避免保护触发。

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

## 启动检查、维护和 ROS 集成

检查开关支持显式开启和关闭：

```bash
cd arms_driver
./arms_test --mock --check-motor 1 --motor-disable 0x10200000 -e check
./arms_test --mock -e 'help' -e 'identify 21' -e 'status 21 --raw'
```

维护、自检、校零、设备身份校验和现场验收见 [阶段 4/5 使用说明](docs/phase4-5.md)。ROS 构建、服务和话题见 [arms_ros](arms_ros/README.md)。开机服务模板在 deploy/，不会在构建或安装源码时启用。
