# arms_ros（ROS 2 Humble）

依赖板端现有的 `communication` 消息包，直接链接同一源码树中的 arms_driver，不拷贝另一份驱动。以下命令从项目根目录执行；示例只访问 mock。

```bash
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/local_setup.bash
colcon build --base-paths arms_ros --install-base install --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash
ros2 run arms_ros arms_node --ros-args -p mock:=true -p config:=$(realpath arms_driver/config/arms.json)
```

参数：config、mock、check_motor（-1 取配置，0/1 覆盖）、motor_disable（-1 取配置）、command_max_age_ms（默认 100）、setup_rt、manage_power。后两者默认 false；不拥有电源时节点不能切断外部电源。节点启动完成后保持失能。伺服使能、解抱闸或写校准均不随话题自动触发。

| 接口 | 类型 / 约定 |
|---|---|
| hardware/actuators_cmds | communication/msg/ActuatorCmds；按 actuators_name 匹配，支持整机帧；五个数值数组与名称等长。取最新帧，检查 header.stamp、数值、重复双臂关节和已使能关节限位。时间戳须与节点同一时钟；未使能关节不会被命令隐式使能。 |
| hardware/joint_states | sensor_msgs/msg/JointState，100 Hz，仅发布本会话已打开关节 |
| hardware/actuator_states | communication/msg/ActuatorStates，100 Hz；PDO 不含温度，温度用 NaN 表示未知，不用零伪装有效值 |
| hardware/arms/diagnostics | diagnostic_msgs/msg/DiagnosticArray，2 Hz，按臂报告链路、WKC、故障关节掩码（fault_joints）、保护关节、屏蔽、启动缺失和被拒绝的命令计数 |
| arms/enable、disable、reset、selftest、free、brake_normal、brake_release、brake_hold、zero | arms_ros/srv/JointOperation：请求只含 `joints`（关节位掩码，selftest 忽略）和 `hardware_zero`（仅对 zero 有效，默认 false 为软件校零）。free 为零力矩拖动（kp=0、kd=0）。brake_* 临时设置 Pn075 = 0/1/2，不保存；三种模式均可在使能或失能时切换，不改变伺服状态或撤销已有目标。任一已打开关节不是 normal 或无法读回模式时，新的 enable 和 free 请求仍被拒绝。zero 按关节判断：已使能先失能，已失能不重复操作，反馈确认静止后再写入；完成后不重新使能。软件零位立即生效。 |

实机调用抱闸服务前，请先停止运动指令，确认关节静止并支撑负载。服务成功仅表示 Pn075 写入并回读一致，不是机械到位反馈。

例如在上述 mock 节点查询健康状态：`ros2 service call /arms/selftest arms_ros/srv/JointOperation "{}"`。升级到 0.3.0 时，请用新服务定义重新构建节点和依赖它的客户端，重新加载安装环境并重启进程；不要混用新旧接口。

状态和目标使用 SensorDataQoS / depth 1，诊断使用可靠 QoS。发生 power_off 保护时，独立监视线程请求失能并锁定会话；如节点接管电源则请求断电。断电请求失败会报错并在节点退出时重试，必须现场核实实际供电状态。恢复接线并确认安全后重新启动节点，不自动恢复使能。运行中丢失上位机目标时沿用驱动配置的超时保持或失能策略。

离线端到端回归（已 source install/local_setup.bash）：

```bash
python3 arms_ros/test/smoke.py arms_driver/config/arms.json
```

测试自建 localhost ROS 域和独立命名空间；节点强制使用 mock，不连接 EtherCAT 或 PCI。
