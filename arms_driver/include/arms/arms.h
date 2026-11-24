/**
 * @file arms.h
 * @brief BXI 双臂 EtherCAT 驱动接口（KGU 执行器，MIT 模式）
 *
 * 关节号采用 BXI 31 关节标准顺序：
 *   15..21  左臂 shoulder_y / shoulder_x / shoulder_z / elbow_y / wrist_x / wrist_y / wrist_z
 *   22..28  右臂，顺序同左臂
 * 总线位置（slave）为 EtherLab 链上位置，从 0 开始。
 *
 * 运行模式：
 *   - 独立模式：arms_open() 创建实时线程并完成预热后返回。
 *   - 外部驱动模式：arms_open() 只完成配置和激活；调用方在自己的实时循环中
 *     每周期依次调用 arms_rx() 和 arms_tx()，并通过 arms_ready() 判断预热完成。
 */
#ifndef ARMS_ARMS_H
#define ARMS_ARMS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ARMS_JOINT_FIRST 15
#define ARMS_JOINT_LAST 28
#define ARMS_JOINT_COUNT 14
#define ARMS_ARM_COUNT 2
#define ARMS_ARM_LEFT 0
#define ARMS_ARM_RIGHT 1
#define ARMS_MAX_SLAVES_PER_ARM 16

typedef struct arms arms_t;

/** 返回码 */
typedef enum {
    ARMS_OK = 0,
    ARMS_E_ARG,         /**< 参数无效 */
    ARMS_E_CONFIG,      /**< 配置文件无效 */
    ARMS_E_SYSTEM,      /**< 系统调用失败（权限、内存、线程） */
    ARMS_E_MASTER,      /**< EtherCAT 主站不可用或被占用 */
    ARMS_E_LINK,        /**< 网口无物理链路 */
    ARMS_E_TOPOLOGY,    /**< 总线上的从站与配置不符 */
    ARMS_E_STATE,       /**< 从站未进入要求的状态 */
    ARMS_E_COMM,        /**< 周期通信失败 */
    ARMS_E_TIMEOUT,     /**< 超时 */
    ARMS_E_UNSUPPORTED, /**< 功能不支持 */
} arms_status;

/** @brief 返回码对应的说明文字 */
const char *arms_strerror(arms_status status);

/** @brief 当前线程最近一次失败的详细原因 */
const char *arms_last_error(void);

/** 关节位掩码，第 n 位对应关节 n（仅 15..28 有效） */
typedef uint32_t arms_mask;
#define ARMS_MASK_JOINT(j) ((arms_mask)1u << (j))
#define ARMS_MASK_LEFT 0x003F8000u
#define ARMS_MASK_RIGHT 0x1FC00000u
#define ARMS_MASK_ALL (ARMS_MASK_LEFT | ARMS_MASK_RIGHT)

/** 总线后端 */
typedef enum {
    ARMS_BACKEND_ETHERLAB = 0, /**< EtherLab 主站 */
    ARMS_BACKEND_MOCK = 1,     /**< 模拟后端，不访问硬件 */
} arms_backend;

/** 运行模式 */
typedef enum {
    ARMS_MODE_CONFIG = 0,     /**< 使用配置文件中的 realtime.mode */
    ARMS_MODE_STANDALONE = 1, /**< 驱动自建实时线程 */
    ARMS_MODE_EXTERNAL = 2,   /**< 由调用方的实时循环驱动 */
} arms_run_mode;

/** 日志回调，在非实时上下文中调用 */
typedef void (*arms_log_fn)(void *user, const char *line);

/** arms_open() 选项，使用前先调用 arms_options_init() */
typedef struct {
    const char *config_path;      /**< 配置文件路径，NULL 时按默认路径查找 */
    arms_backend backend;         /**< 总线后端 */
    arms_run_mode mode;           /**< 运行模式 */
    unsigned int cycle_hz;        /**< 周期频率，0 表示使用配置值 */
    int rt_cpu;                   /**< 实时线程 CPU，-2 表示使用配置值，-1 表示不绑定 */
    arms_mask joints;             /**< 参与的关节，0 表示配置中的全部关节 */
    unsigned int scan_timeout_ms; /**< 扫描超时，0 表示 10000 ms */
    arms_log_fn log;              /**< 启动过程日志 */
    void *log_user;
} arms_options;

/** @brief 填充 arms_options 默认值 */
void arms_options_init(arms_options *options);

/** 关节状态 */
typedef enum {
    ARMS_JS_ABSENT = 0, /**< 未选择或未配置 */
    ARMS_JS_OFFLINE,    /**< 已配置但总线上无响应 */
    ARMS_JS_PREOP,
    ARMS_JS_SAFEOP,
    ARMS_JS_LOCKED,     /**< OP，伺服失能 */
    ARMS_JS_RELEASED,   /**< OP，伺服失能，抱闸强制松开 */
    ARMS_JS_ENABLED,    /**< OP，伺服使能 */
    ARMS_JS_FAULT,      /**< 驱动器故障或 AL 错误 */
} arms_joint_mode;

/** @brief 关节状态名称 */
const char *arms_joint_mode_name(arms_joint_mode mode);

/** 单个关节的反馈 */
typedef struct {
    int joint;              /**< 关节号 */
    const char *name;       /**< 关节名称 */
    int arm;                /**< ARMS_ARM_LEFT / ARMS_ARM_RIGHT */
    int slave;              /**< 链上位置，从 0 开始 */
    arms_joint_mode mode;
    uint8_t al_state;       /**< EtherCAT AL 状态 */
    uint16_t status_word;   /**< 0x6041 */
    uint16_t error_code;    /**< 0x603F */
    double position;        /**< 输出端位置 [rad]，已计入方向和零位偏置 */
    double velocity;        /**< 输出端速度 [rad/s] */
    double torque;          /**< 输出端力矩 [N.m] */
    uint64_t sample_ns;     /**< 采样时刻，CLOCK_MONOTONIC */
} arms_joint_state;

/** 周期间隔统计 [us] */
typedef struct {
    double min, mean, max, rms_jitter;
} arms_interval_stats;

/** 单臂通信统计 */
typedef struct {
    bool present;                    /**< 本次是否打开该臂 */
    char interface[32];
    int master_index;
    int slaves_expected;             /**< 配置使用的从站数 */
    int slaves_found;                /**< 总线上发现的从站数 */
    int slaves_op;                   /**< 处于 OP 的从站数 */
    bool link_up;
    uint32_t wkc_expected;
    uint32_t wkc_last;
    uint64_t cycles;
    uint64_t wkc_bad;                /**< WKC 不符的周期数 */
    uint32_t wkc_bad_consecutive_max;
    uint64_t send_errors;
    bool comm_failed;                /**< 连续异常超过 safety.wkc_tolerance */
    double recv_us_max;
    double send_us_max;
} arms_arm_stats;

/** 总线与周期统计 */
typedef struct {
    unsigned int cycle_hz;
    uint64_t cycles;
    uint64_t overruns;               /**< 周期间隔超过 1.5 倍周期的次数 */
    uint64_t skipped;                /**< 独立模式下跳过的周期数 */
    arms_interval_stats period;      /**< 相邻周期起点的间隔 */
    double wake_late_us_max;         /**< 独立模式：唤醒相对计划时刻的最大延迟 */
    double work_us_max;              /**< 单周期最大处理时间 */
    arms_arm_stats arm[ARMS_ARM_COUNT];
} arms_bus_stats;

/**
 * @brief 打开驱动：读取配置、扫描、配置 PDO 与 DC、激活主站
 * @param options 选项
 * @param out 成功时返回句柄
 * @return ARMS_OK 成功；失败时 arms_last_error() 给出原因
 *
 * 独立模式下返回前已完成预热（全部关节进入 OP，伺服失能）。
 * 外部驱动模式下返回时尚未预热，需周期调用 arms_rx()/arms_tx()。
 */
arms_status arms_open(const arms_options *options, arms_t **out);

/** @brief 停止通信、释放主站并释放句柄 */
void arms_close(arms_t *arms);

/**
 * @brief 外部驱动模式：接收两臂过程数据并更新反馈
 * @param now_ns 本周期起点，CLOCK_MONOTONIC
 * @note 仅限外部驱动模式，在调用方实时线程中调用
 */
arms_status arms_rx(arms_t *arms, uint64_t now_ns);

/**
 * @brief 外部驱动模式：写入目标并发送两臂过程数据
 * @param now_ns 当前时刻，CLOCK_MONOTONIC，用作 DC 应用时间
 * @note 必须在同一周期的 arms_rx() 之后调用
 */
arms_status arms_tx(arms_t *arms, uint64_t now_ns);

/** @brief 预热是否完成（全部关节 OP 且 WKC 稳定） */
bool arms_ready(const arms_t *arms);

/**
 * @brief 阻塞等待预热完成
 * @param timeout_ms 超时，0 表示使用配置 safety.op_timeout_ms
 * @note 外部驱动模式下调用方的实时循环必须同时在运行
 */
arms_status arms_wait_ready(arms_t *arms, unsigned int timeout_ms);

/** @brief 周期 [ns] */
uint64_t arms_period_ns(const arms_t *arms);

/** @brief 读取已打开关节的状态，按关节号升序写入 */
arms_status arms_read(arms_t *arms, arms_joint_state *states, size_t count, size_t *written);

/** @brief 读取单个关节状态 */
arms_status arms_read_joint(arms_t *arms, int joint, arms_joint_state *state);

/** @brief 读取通信统计 */
arms_status arms_get_stats(arms_t *arms, arms_bus_stats *stats);

/** @brief 清零通信统计（下一周期生效） */
void arms_reset_stats(arms_t *arms);

unsigned int arms_cycle_hz(const arms_t *arms);
arms_run_mode arms_mode(const arms_t *arms);
int arms_rt_cpu(const arms_t *arms);
const char *arms_backend_name(const arms_t *arms);
const char *arms_config_path(const arms_t *arms);
arms_mask arms_opened_joints(const arms_t *arms);

/** @brief 关节名称；arms 可为 NULL，此时返回标准名称 */
const char *arms_joint_name(const arms_t *arms, int joint);

/** @brief 按名称查找关节号，接受带或不带 "_joint" 后缀的名称；未找到返回 -1 */
int arms_joint_by_name(const arms_t *arms, const char *name);

/** @brief 臂名称 "left" / "right" */
const char *arms_arm_name(int arm);

/** @brief 使能关节（暂不支持，返回 ARMS_E_UNSUPPORTED） */
arms_status arms_enable(arms_t *arms, arms_mask joints);

/** @brief 失能关节 */
arms_status arms_disable(arms_t *arms, arms_mask joints);

/** @brief 版本字符串 */
const char *arms_version(void);

#ifdef __cplusplus
}
#endif

#endif
