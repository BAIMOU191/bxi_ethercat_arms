#ifndef ARMS_CONFIG_H
#define ARMS_CONFIG_H

#include "arms/arms.h"

#define ARMS_NAME_LEN 48
#define ARMS_PATH_LEN 512

typedef struct {
    bool present;
    char interface[32];
    char mac[24];
    int master; /* -1 表示按 /sys/module/ec_master/parameters/main_devices 解析 */
} arms_arm_config;

typedef struct {
    bool present;
    int joint;
    char name[ARMS_NAME_LEN];
    int arm;
    int link;               /* 所在链路（主站）：通常等于 arm；双臂共用网口时右臂关节也为 ARMS_ARM_LEFT */
    int slave;              /* 链路上的位置 */
    int direction;          /* +1 / -1 */
    double offset_rad;
    /* 限位均依据反馈判断，不截断指令 */
    double pos_limit_min, pos_limit_max; /* 软位置限位：实际位置越界时朝外的速度和前馈置 0 */
    double vel_limit, tor_limit;         /* 反馈速度 / 力矩保护阈值 */
} arms_joint_config;

typedef struct {
    char path[ARMS_PATH_LEN];
    int version;
    unsigned int cycle_hz;
    arms_run_mode mode;     /* ARMS_MODE_STANDALONE / ARMS_MODE_EXTERNAL */
    int rt_cpu;             /* -1 = 不绑定 */
    int irq_cpu;            /* -1 = 不调整；由 rt_setup.sh 使用 */
    int rt_priority;
    int softirq_priority;   /* 由 rt_setup.sh 使用 */
    unsigned int wkc_tolerance;
    unsigned int command_timeout_ms;
    char timeout_action[16];
    bool check_motors;      /* 启动时要求全部未屏蔽关节在线 */
    uint32_t ignore_mask;   /* 第 n 位对应关节 n：不打开、不检查 */
    char offline_action[16];/* "power_off" | "disable" */
    unsigned int overspeed_ms;  /* 速度超过 vel_limit 的累计时长阈值 */
    unsigned int overtorque_ms; /* 力矩超过 tor_limit 的累计时长阈值 */
    unsigned int op_timeout_ms;
    bool drive_monitor;         /* 轮询执行器温度、驱动器丢帧次数和从站错误计数 */
    int temp_warn_c;            /* 执行器温度告警阈值 [℃] */
    int temp_max_c;             /* 执行器温度保护阈值 [℃]，达到即全部失能并锁定 */
    unsigned int esc_summary_s; /* 处理类 ESC 错误（从站拒绝的帧）汇总记录的时间窗口 [s] */
    uint32_t vendor_id;
    uint32_t product_code;
    arms_arm_config arm[ARMS_ARM_COUNT];
    bool shared_link;       /* 左右臂配置为同一网口：两臂在同一条 EtherCAT 链上，由链路 0 统一收发 */
    int hub_slave;          /* 集线器的链上位置，-1 表示未配置（arms.hub） */
    int hub_out[ARMS_ARM_COUNT]; /* 各臂所接集线器输出口 OUT1..OUT3，0 表示不检查 */
    arms_joint_config joint[ARMS_JOINT_LAST + 1]; /* 按关节号索引 */
} arms_config;

/* BXI 标准关节名称，按关节号 0..30 索引 */
extern const char *const arms_standard_joint_names[31];

bool arms_store_offset(const char *path, int joint, double value);

bool arms_cycle_hz_supported(unsigned int hz);

/* 读取并校验配置；失败时返回 false，原因写入 error */
bool arms_config_load(const char *path, arms_config *config, char *error, size_t error_size);

/* 默认查找顺序: $ARMS_CONFIG、/etc/bxi/arms.json、./config/arms.json、
 * <程序目录>/../config/arms.json；均不可读时返回 false */
bool arms_config_default_path(char *out, size_t size);

#endif
