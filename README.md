# bxi_ethercat_arms

BXI 双臂 EtherCAT 驱动（x86 分支）。主控为 ASUS NUC 13 Pro（Intel i7-1360P，Ubuntu 22.04），左右臂共 14 个 KGU 执行器，EtherLab 主站，MIT 模式。

左右臂经 EtherCAT 分支器接在同一个网口 `enp86s0` 上（一个主站、一条链）：`arms.left` 与 `arms.right` 配置为同一网口时，两臂关节由同一条链路收发，`bus`、通信事件日志和 ROS 诊断中该链路显示为 `left+right`，提示符显示为 `LR<OP>/<总数>`。链上位置 0 为集线器 ZSW-8004，左臂接 OUT1（链上位置 1–7），右臂接 OUT2（8–14），由 `arms.hub` 在启动时核对接线。资源分配见 `docs/resources.md`。
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

校零示例：先支撑手臂，`free 15` 后手动停在目标零位，再输入 `zero 15`。已使能就自动失能，已失能不重复操作；驱动确认失能和静止后才校零，完成后不重新使能。`zero` 默认是硬件校零（Pn101，零点保存在驱动器中），同时把配置中该关节的 `offset_rad` 写为 0；驱动器重新上电后生效，所以接着执行 `power off`、`power on`，再用 `status 15` 核对，上电前该关节拒绝 `enable` / `free`。`zero 15 --soft` 是软件校零，只改配置文件的 `offset_rad` 并立即生效。

`pn_save 15` 只对 15 号关节按需失能并等待静止，再发送 Pn097 保存脉冲；不检查或改变抱闸模式，也不会重新使能。强制松闸时请先可靠支撑负载。

参数维护：`pn list` 查看已收录的手册参数；`pn help Pn026` 查取值、单位和注意事项。`pn 15 Pn026`、`sdo 15 0x2026 0`、`brake_voltage 15` 都读 15 号制动器保持电压。`torque_scale 15` 读 Pn082，同时显示原始值和 A/N·m。两个别名都可追加**原始整数**写入，也可用作 `pn 15 <别名>`。普通写一次只操作一个已失能关节，不自动保存；目录仅供查询，不是白名单。未列出的 Pn 同样可按编号访问；`sdo` 可按索引、子索引和显式字宽直接读写（例如 `u32:123` 或 `hex:01020304`）。普通写成功只表示从站已接收，可另行读取核对。详见 [参数维护规则](docs/phase4-5.md#pn-参数目录)。

输出中的时间戳为本地时间 `[HH:MM:SS.mmm]`：启动、电源、实时环境、命令开始、通信测试进度等事件行默认带时间戳；
`-T` 使每一行都带时间戳，`scripts/comm_test.sh` 默认启用。

### 运动控制与测试命令

建议先用 `--mock` 核对关节选择和目标值。实机使能前请确认机械臂有足够的运动空间，且可随时急停。
`<关节>` 可写 `all`、`left`、`right`、单个编号或名称、范围 `15-18` 或列表 `21,28`。
位置和幅值单位为 rad，速度为 rad/s，前馈力矩为 N·m，kp 单位为 N·m/rad，kd 单位为 N·m·s/rad；`enable -t` 的单位为毫秒。

| 命令 | 行为 |
|---|---|
| `enable <关节> [-t 毫秒]` | 使能并显示模式和实际位置。使能后指令为零（kp=0、kd=0、速度和前馈为 0，与参考 hw_lib 一致），关节不出力，需随后用 `mit` 给出增益。省略超时时间或指定 0 时使用驱动默认值。 |
| `disable [关节]` / `reset <关节>` | 失能（默认全部已打开关节）或故障复位。复位后需重新使能才能运动。 |
| `mit <关节> <位置> <kp> <kd> [速度] [前馈]` | kp、kd 必填，没有默认值；速度和前馈默认为零。指令下一周期原样生效，不做轨迹插值，也不截断（与参考 hw_lib 的 `set_motor_cmd` 一致），目标较远时关节会快速跳向目标；非零速度或前馈以 100 Hz 持续刷新。kp、kd 为负数时按 0 下发。 |
| `hold [关节]` | 保持当前**指令位置**，保留增益，清零速度和前馈。若此前是 `free` 或刚 `enable`，`kp` 仍为零，不能靠 `hold` 承重。默认全部已打开关节。 |
| `swing <关节> <幅值> <周期秒> <次数> <kp> <kd>` | 以当前指令位置为中心，200 Hz 发送正弦位置和对应速度。每秒报告最大跟踪误差，结束时回中心且速度为零；回车或 Ctrl+C 中止时保持当前指令位置。 |
| `power on` / `power off` / `power status` | 电机总电源与其他程序（如腰部控制程序）共用，谁接通谁断开。电源已由其他程序接通、或 PCI 正被其他程序使用时，本程序不接管电源，退出和离线保护时不断电；本程序接通电源后立即释放 PCI（停止 CAN、解除锁，不改动电源），其他程序可以随后使用。`power off` 只断开本程序接通的电源，此时若其他程序正在使用 PCI，电源保持接通。 |
| `commtest <秒> [--enabled <kp> <kd>]` | 不加选项时要求全部关节已失能。加 `--enabled` 时使能全部已打开关节，并以给定 kp、kd 保持在使能时的实际位置，正常结束或提前中止后全部失能。例如 `commtest 60 --enabled 5 0.5`。 |

`mit`、`swing` 在下发前只检查所选关节是否已使能；若有关节未使能，整组均不下发。目标位置、速度、前馈和增益不做限幅。
软位置限位 `pos_limit`：关节实际位置越过上限后，驱动屏蔽正向的速度和前馈（只留下位置项和阻尼），越过下限时屏蔽负向的，位置指令照常下发；`status` 中以 `P` 标志显示。
纯位置 `mit`、`hold`、`swing`、`disable`、`reset` 或失去使能均会停止相应关节的动态指令刷新。
例如，可在模拟会话中依次执行 `enable 17`、`mit 17 0.1 5 0.5`、`hold 17`、`disable 17`。

提示符的 `E` 后是已使能关节数。`status` 新列 `cmd`、`kp`、`kd` 为当前指令位置和增益；
`flag` 中 `P` 表示软位置限位正在屏蔽朝外的速度或前馈、`T` 表示指令超时，无标志时为 `-`；`flt` 为累计故障次数。
后台以 10 Hz 检查故障计数增加、关节退出使能和臂通信失败。发生变化时输出带时间戳的该臂关节及 WKC 快照。

## 编译与测试

```bash
cmake -S . -B build && cmake --build build -j4
ctest --test-dir build --output-on-failure
```

依赖：EtherLab 1.6（`/usr/local/etherlab`），`third_party/bxi_pci_drv/x86_64` 中的 BXI PCI SDK 用于电机电源控制。单元测试使用 `tests/mock_arms.json`（双臂、模拟后端），不读取 `config/arms.json`。

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
| `overspeed_ms` / `overtorque_ms` | 反馈超限保护的累计时长，默认 500 / 100 ms。使能中的关节反馈速度超过 `vel_limit` 或力矩超过 `tor_limit` 时计数加 1，否则减 1，达到该时长即全部关节失能并锁定，`reset` 清除 |

`joint_defaults` 段给出全部关节共用的保护参数，单个关节可写同名字段覆盖：

| 字段 | 说明 |
|---|---|
| `pos_limit` | 软位置限位 `[下限, 上限]`（rad，默认 `[-3.0, 3.0]`）。不截断指令；实际位置越界后只屏蔽朝外的速度和前馈（同参考 hw_lib 的 `pos_limit_min/max`） |
| `vel_limit` | 反馈超速保护阈值（rad/s，默认 3.0），配合 `overspeed_ms` |
| `tor_limit` | 反馈超力矩保护阈值（N·m，默认 30.0），配合 `overtorque_ms` |

每个关节的配置项：`joint`、`name`、`arm`、`slave` 必填；`direction`、`offset_rad` 以及上述三个保护字段可选。
旧格式的 `limit`、`gain_max`、`gain_default`、`home` 已删除，出现时启动报错。旧文件可用 `python3 arms_driver/scripts/migrate_config.py arms_driver/config/arms.json` 转换：保留 `offset_rad` 等字段，`limit.vel_fault` / `limit.torque_fault` 转为 `vel_limit` / `tor_limit`，原文件备份为 `arms.json.bak`；加 `--dry-run` 只打印结果。

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

## 通信事件日志

`arms_test` 和 `arms_node` 运行时，驱动在实时线程中记录最近 64 个周期的时序，发生以下事件时把它连同两臂链路、WKC 计数和各从站 AL 状态追加到日志文件：WKC 异常（每臂每秒最多一条）、通信失败、通信恢复、周期延迟（两次周期开始间隔超过 1.5 个周期，每秒最多一条）、离线保护触发。

默认文件为 `arms_driver/logs/comm_events.log`（配置文件所在 `config/` 目录旁的 `logs/`），`arms_test --event-log FILE` 或 ROS 参数 `event_log` 可指定其他路径；无法写入时改用 `/tmp/arms_comm_events.log`。每条事件给出发生时刻和对照内核日志的命令，表中“发送→接收us”是上一周期发送结束到本周期接收的间隔，正常接近一个周期；接近 0 说明帧尚未返回就被读取。

## 驱动器监控

`safety.drive_monitor`（默认 true）开启后，实时线程用异步请求轮流读取每个从站，不阻塞控制周期：

- **执行器温度**：写 Pn002=77 选择监控量，等待 100 ms 后读对象 0x3000；每个关节约 2 秒更新一次，显示在 `status` 末列和 ROS `actuator_states.driver_temperature`。达到 `safety.temp_warn_c`（默认 75 ℃）记录告警；达到 `safety.temp_max_c`（默认 90 ℃，须低于驱动器自身的 Pn039 报警阈值）全部关节失能并锁定，降温后 `reset`。轮询会改变驱动器数码管显示的监控量，轮询期间不要执行 `pn_save`。
- **驱动器自报丢帧次数**：Pn002=63，上电清零；增加时记录事件。
- **ESC 错误计数**：每 10 ms 读一个从站的寄存器 0x0300..0x0313（各端口无效帧、物理层错误、转发错误、链路丢失，以及处理单元和 PDI 错误）。预热完成、通信恢复和 `diag reset` 时清零；增加时记录事件。端口 0 接主站一侧，端口 1 接下一个从站。
- **CoE 紧急报文**：每个从站保留 8 条，写事件日志时取出。

`diag [关节] [reset]` 查看驱动器丢帧次数和 ESC 错误计数（温度见 `status`）；`commtest` 开始时清零 ESC 计数，结束时列出计数非零或驱动器丢帧增加的关节，并判为不通过。ROS 诊断增加 `max_temp_c`、`drive_frame_loss`、`esc_error_sum`。这些事件都写入通信事件日志。

### ESC 错误计数分类与 EtherCAT 网口隔离

- **线路类**（各端口无效帧、物理层错误、链路丢失）：指向网线、接头或从站网口，每次增加都写一条“线路错误计数增加”事件，`commtest` 判为不通过。
- **处理类**（转发错误、处理单元错误、PDI 错误）：帧在电气上完好，只是被从站拒绝。只有这一类时不逐条记录，按 `safety.esc_summary_s`（默认 60 秒）汇总一条“从站拒绝的帧（汇总）”，给出被拒帧数和最先拒绝的从站；`commtest` 只提示，不判为不通过。

EtherLab 通用驱动下网口对 Linux 仍然可见，NetworkManager 的 DHCP、avahi 的 mDNS、IPv6 报文会被发到 EtherCAT 链上并被第一个从站拒绝，表现为全部从站处理类计数同步增加。`rt_setup.sh apply`（`arms_test` 默认执行，ROS 节点需 `setup_rt:=true`）因此会让配置中的 EtherCAT 网口退出 NetworkManager 管理、去掉 IPv4 地址并关闭 IPv6，链路保持 UP；`revert`（程序退出时执行）交还 NetworkManager 或恢复原有地址和 IPv6 设置。其他网卡（如 wlan0）不受影响。`rt_setup.sh status` 显示各网口当前状态。程序被强制结束未能还原时，重启 NetworkManager 或板子即恢复。
