# bxi_ethercat_arms

BXI 双臂 EtherCAT 驱动：RK3588 + 左右臂共 14 个 KGU 关节（EtherLab 主站，MIT 模式）。
一个实时线程在一个 CPU 核上同时驱动两条臂。

| 目录 | 内容 |
|---|---|
| `arms_driver/` | C 库 `libarms_driver.a` + 测试工具 `arms_test`（不依赖 ROS） |
| `arms_ros/` | ROS2 节点（阶段 5，尚未开始） |

## 关节编号

BXI 31 关节标准顺序：左臂 15–21（肩 y/x/z、肘 y、腕 x/y/z），右臂 22–28，顺序相同。
`slave` 是 EtherLab 链上位置，从 0 开始（testmotor 显示为 slave+1）。映射写在 `arms_driver/config/arms.json`。

## 编译

```bash
cd ~/bxi_ws/bxi_ethercat_arms/arms_driver
cmake -S . -B build && cmake --build build -j4
ctest --test-dir build --output-on-failure     # 单元测试 + mock 会话
```

## 使用

```bash
./build/arms_test -h                 # 启动参数说明
./build/arms_test --mock             # 不接硬件熟悉命令
sudo scripts/rt_setup.sh apply       # 实时环境（中断绑核、CPU 隔离）
sudo ./build/arms_test -c config/arms.json -e "commtest 300"
sudo scripts/rt_setup.sh revert      # 测完还原
```

进入终端后输入 `help`；任一命令加 `-h` 查看详细用法。

安全：阶段 1 只做通信，所有关节始终失能（控制字 0x0000）。不要和 `motor_console`、
`hw_lib_test_*` 或 ROS 硬件节点同时运行。

## 阶段

1. 配置、mock、命令表与帮助、EtherLab 后端、单核实时循环、`status/bus/commtest`（当前）
2. 双臂同一实时循环长测
3. CiA402 使能、MIT、限幅、`mit/swing`
4. 释力/锁抱闸（Pn075）、校准（软零位/Pn101）、自检、`identify`
5. ROS2 节点、systemd 开机配置
