#ifndef ARMS_CONFIG_H
#define ARMS_CONFIG_H

#include "arms/arms.h"

#define ARMS_NAME_LEN 48
#define ARMS_PATH_LEN 512

typedef struct {
    bool present;
    char interface[32];
    char mac[24];
    int master; /* -1 = resolve from /sys/module/ec_master/parameters/main_devices */
} arms_arm_config;

typedef struct {
    bool present;
    int joint;
    char name[ARMS_NAME_LEN];
    int arm;
    int slave;
    int direction;          /* +1 / -1 */
    double offset_rad;
    double pos_min, pos_max;
    double vel_max, tff_max;    /* 指令限幅 */
    double vel_fault, tor_fault;/* 反馈超限保护阈值 */
    double kp_max, kd_max;
    double kp_default, kd_default;
    double home;            /* 回零目标位置 [rad] */
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
    unsigned int overspeed_ms;  /* 速度持续超过 vel_fault 的累计时长阈值 */
    unsigned int overtorque_ms; /* 力矩持续超过 torque_fault 的累计时长阈值 */
    unsigned int op_timeout_ms;
    bool drive_monitor;         /* 轮询执行器温度、驱动器丢帧次数和从站错误计数 */
    int temp_warn_c;            /* 执行器温度告警阈值 [℃] */
    int temp_max_c;             /* 执行器温度保护阈值 [℃]，达到即全部失能并锁定 */
    unsigned int esc_summary_s; /* 处理类 ESC 错误（从站拒绝的帧）汇总记录的时间窗口 [s] */
    uint32_t vendor_id;
    uint32_t product_code;
    arms_arm_config arm[ARMS_ARM_COUNT];
    arms_joint_config joint[ARMS_JOINT_LAST + 1]; /* indexed by joint number */
} arms_config;

/* Standard BXI joint names, indexed by joint number 0..30. */
extern const char *const arms_standard_joint_names[31];

bool arms_store_offset(const char *path, int joint, double value);

bool arms_cycle_hz_supported(unsigned int hz);

/* Load and validate. On failure returns false and writes a message to error. */
bool arms_config_load(const char *path, arms_config *config, char *error, size_t error_size);

/* Default search: $ARMS_CONFIG, /etc/bxi/arms.json, ./config/arms.json,
 * <exe dir>/../config/arms.json. Returns false if none is readable. */
bool arms_config_default_path(char *out, size_t size);

#endif
