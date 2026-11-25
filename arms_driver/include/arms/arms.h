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
    int check_motors;             /**< 1: 未屏蔽关节必须全部在线，否则打开失败；0: 跳过不在线的关节；-1 取配置 */
    int64_t ignore_mask;          /**< 屏蔽掩码，第 n 位对应关节 n，屏蔽的关节不打开、不检查；-1 取配置 */
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
    ARMS_JS_HELD,       /**< OP，伺服失能，抱闸强制锁闸（Pn075=2） */
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
    uint8_t txpdo[16], rxpdo[23]; /**< 最近采样的原始从站输入和主站输出 */
    uint16_t status_word;   /**< 0x6041 */
    uint16_t error_code;    /**< 0x603F */
    double position;        /**< 输出端位置 [rad]，已计入方向和零位偏置 */
    double velocity;        /**< 输出端速度 [rad/s] */
    double torque;          /**< 输出端力矩 [N.m] */
    uint64_t sample_ns;     /**< 采样时刻，CLOCK_MONOTONIC */

    /* 本周期实际下发的指令（关节侧，已限幅） */
    double cmd_position;    /**< [rad] */
    double cmd_velocity;    /**< [rad/s] */
    double cmd_torque;      /**< 前馈力矩 [N.m] */
    double cmd_kp;
    double cmd_kd;
    bool cmd_limited;       /**< 本周期指令被限幅 */
    bool cmd_timeout;       /**< 指令超时，已按 safety.timeout_action 处理 */
    uint32_t fault_count;   /**< 打开以来进入故障的次数 */
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
    bool comm_failed;                /**< 连续异常超过 safety.wkc_tolerance，恢复稳定后自动清除 */
    uint32_t comm_drops;             /**< 通信失败次数 */
    uint32_t comm_recoveries;        /**< 自动恢复次数 */
    double recv_us_max;
    double send_us_max;
} arms_arm_stats;

/** 保护原因 */
typedef enum {
    ARMS_PROTECT_NONE = 0,
    ARMS_PROTECT_OVERSPEED,  /**< 反馈速度超过 limit.vel_fault 累计 safety.overspeed_ms */
    ARMS_PROTECT_OVERTORQUE, /**< 反馈力矩超过 limit.torque_fault 累计 safety.overtorque_ms */
} arms_protect_reason;

/**
 * 超速/超力矩保护：任一使能关节触发后，全部关节失能并锁定，
 * 拒绝使能，直到 arms_reset() 清除。
 */
typedef struct {
    arms_protect_reason reason;
    int joint;        /**< 触发关节，无则 -1 */
    double value;     /**< 触发时的反馈速度 [rad/s] 或力矩 [N.m] */
    double limit;     /**< 对应阈值 */
} arms_protect;

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
    arms_mask offline_joints;        /**< 当前不在线（AL 未响应）的已打开关节 */
    bool tripped;                    /**< 离线保护已触发（safety.offline_action = power_off），需 arms_clear_trip() */
    arms_mask trip_joints;           /**< 触发保护时不在线的关节 */
    arms_protect protect;            /**< 超速/超力矩保护状态 */
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
/** @brief 生效的屏蔽掩码（只含 15..28 位） */
arms_mask arms_ignored_joints(const arms_t *arms);
/** @brief 启动时未屏蔽但不在线、因而未打开的关节（check_motors = 0 时可能非空） */
arms_mask arms_missing_joints(const arms_t *arms);
/** @brief 本次会话的有效启动检查策略 */
bool arms_startup_check_enabled(const arms_t *arms);
/** @brief 是否在关节离线时触发下电保护（safety.offline_action = power_off） */
bool arms_offline_power_off(const arms_t *arms);

/**
 * @brief 检查已打开关节是否全部在线（OP、WKC 正确、通信正常）
 * @param offline 可为 NULL；返回不在线的关节
 * @return ARMS_OK 全部在线；否则 ARMS_E_STATE，arms_last_error() 列出关节
 */
arms_status arms_check_online(arms_t *arms, arms_mask *offline);

/**
 * @brief 清除离线保护：要求全部已打开关节重新在线，清除后关节处于失能
 */
arms_status arms_clear_trip(arms_t *arms);

/** @brief 关节名称；arms 可为 NULL，此时返回标准名称 */
const char *arms_joint_name(const arms_t *arms, int joint);

/** @brief 按名称查找关节号，接受带或不带 "_joint" 后缀的名称；未找到返回 -1 */
int arms_joint_by_name(const arms_t *arms, const char *name);

/** @brief 臂名称 "left" / "right" */
const char *arms_arm_name(int arm);

/** 关节限位与默认增益（关节侧） */
typedef struct {
    double pos_min, pos_max;  /**< [rad] */
    double vel_max;           /**< [rad/s] */
    double tff_max;           /**< [N.m] */
    double kp_max, kd_max;
    double kp_default, kd_default;
    double home;              /**< 回零位置 [rad] */
    double vel_fault;         /**< 反馈速度保护阈值 [rad/s] */
    double torque_fault;      /**< 反馈力矩保护阈值 [N.m] */
} arms_joint_limits;

/** @brief 读取关节限位，关节未打开时返回 ARMS_E_ARG */
arms_status arms_get_limits(const arms_t *arms, int joint, arms_joint_limits *limits);

/**
 * @brief 使能关节（CiA402：0x06 → 0x07 → 0x0F）
 * @param joints 关节掩码，只作用于已打开的关节
 * @param timeout_ms 等待全部关节进入使能的超时，0 表示 1000 ms
 *
 * 使能时目标位置取当前实际位置，增益为 gain_default，因此关节保持原位。
 * 已使能的关节不受影响。处于故障的关节需先 arms_reset()。
 * 超时或失败时，本次请求中未完成的关节回到失能。
 */
arms_status arms_enable(arms_t *arms, arms_mask joints, unsigned int timeout_ms);

/** @brief 失能关节（控制字 0x0000，抱闸由驱动器接管），立即返回 */
arms_status arms_disable(arms_t *arms, arms_mask joints);

/**
 * @brief 故障复位（控制字 0x0080 脉冲），复位后关节处于失能
 * @param timeout_ms 等待故障位清除的超时，0 表示 1000 ms
 *
 * 同时清除所属臂的通信失败标志（当前通信须已恢复）和超速/超力矩保护。
 */
arms_status arms_reset(arms_t *arms, arms_mask joints, unsigned int timeout_ms);

/** MIT 指令（关节侧） */
typedef struct {
    double position; /**< 目标位置 [rad] */
    double velocity; /**< 目标速度 [rad/s] */
    double torque;   /**< 前馈力矩 [N.m] */
    double kp;
    double kd;
} arms_command;

/**
 * @brief 写入单个关节的 MIT 指令，下一周期原样生效
 *
 * 驱动不做轨迹插值，平滑由调用方负责。只对已使能的关节有效。各量在周期内按
 * limit / gain_max 限幅。调用方需在 safety.command_timeout_ms 内持续刷新，否则按
 * safety.timeout_action 处理。可在任意非实时线程调用。
 */
arms_status arms_set_command(arms_t *arms, int joint, const arms_command *command);

/** @brief 关节保持当前指令位置：速度与前馈清零，增益不变 */
arms_status arms_hold(arms_t *arms, arms_mask joints);

/**
 * @brief 零力矩拖动：使能关节（已使能的保持使能），下发 kp = 0、速度 0、前馈 0、阻尼 kd
 * @param kd 阻尼，0 表示完全自由，须不超过 gain_max.kd
 *
 * 关节没有保持力，会受重力下落。先支撑关节再失能；arms_hold() 保留 kp=0，不会恢复位置保持力。
 * 有关节的抱闸不是正常控制模式时拒绝执行。
 */
arms_status arms_free(arms_t *arms, arms_mask joints, double kd);

/*
 * 维护接口：只能在同一个非实时管理线程中调用（内部使用阻塞的 SDO 访问）。
 * 写操作要求该关节所在臂通信正常、confirmed 为 true，其余条件见各接口。
 */

/** 抱闸模式（Pn075） */
typedef enum {
    ARMS_BRAKE_UNKNOWN = -1, /**< 未读取或读取失败 */
    ARMS_BRAKE_NORMAL = 0,   /**< 正常控制：使能时松闸，失能时抱闸 */
    ARMS_BRAKE_RELEASE = 1,  /**< 强制松闸 */
    ARMS_BRAKE_HOLD = 2,     /**< 强制抱闸 */
} arms_brake_mode;

/** @brief 抱闸模式名称 normal / release / hold / unknown */
const char *arms_brake_mode_name(arms_brake_mode mode);

/**
 * @brief 临时设置抱闸模式（写 Pn075 并回读，不保存到驱动器）
 *
 * RELEASE / HOLD 要求该关节失能；NORMAL 不要求失能，但仍要求有效通信和明确确认。
 * 任一已打开关节不是 NORMAL 或无法确认模式时，arms_enable() 和 arms_free() 被拒绝。
 * arms_close() 尝试将本次会话改动过的关节恢复为 NORMAL，不保证机械抱闸已闭合。
 */
arms_status arms_set_brake(arms_t *arms, int joint, arms_brake_mode mode, bool confirmed);

/** @brief 读取单个关节的 Pn075 并更新内部记录 */
arms_status arms_get_brake(arms_t *arms, int joint, arms_brake_mode *mode);

/**
 * @brief 读取全部已打开关节的 Pn075
 * @param released 返回强制松闸的关节，可为 NULL
 * @param held     返回强制抱闸的关节，可为 NULL
 * @param unknown  返回读取失败的关节，可为 NULL
 *
 * 启动后调用一次，用于发现驱动器中保存的或其他工具遗留的非正常抱闸模式。
 */
arms_status arms_brake_scan(arms_t *arms, arms_mask *released, arms_mask *held, arms_mask *unknown);

/** 维护操作 */
typedef enum {
    ARMS_ZERO_SOFT, /**< 以当前位置为零点：写入配置 offset_rad 并立即生效；要求关节失能、静止 */
    ARMS_ZERO_HW,   /**< Pn101 清零脉冲，驱动器重新上电后生效；要求关节失能、静止、抱闸为正常模式 */
    ARMS_SAVE,      /**< Pn097 参数保存脉冲；不检查或更改抱闸模式 */
} arms_service_op;

/** @brief 执行维护操作（单个关节；多个关节由调用方逐个调用）。校零和参数保存均按需失能并等待静止，操作后不重新使能。 */
arms_status arms_service(arms_t *arms, int joint, arms_service_op op, bool confirmed);

/** @brief 读取对象字典 */
arms_status arms_sdo_read(arms_t *arms, int joint, uint16_t index, uint8_t sub, uint8_t *out, size_t cap,
                          size_t *size);

/** @brief 按给定字节下载一个 SDO，不查询手册目录；设备可以返回 SDO abort。
 * 普通写需在线且失能，不抢写运行中的 PDO；成功指从站接受下载，不代表已回读或掉电保存。
 * Pn075/097/101（子索引 0）沿用专用动作及反馈检查。 */
arms_status arms_sdo_write(arms_t *arms, int joint, uint16_t index, uint8_t sub, const uint8_t *data,
                           size_t size, bool confirmed);
/** @brief Pn000..999 的 INT16 简写，与 arms_sdo_write 的语义相同。 */
arms_status arms_pn_write(arms_t *arms, int joint, unsigned number, int value, bool confirmed);

/** @brief Pn 参数号对应的对象字典索引（Pn075 → 0x2075），无效返回 -1 */
int arms_pn_index(unsigned number);

/** 关节的配置映射 */
typedef struct {
    int direction;     /**< +1 / -1 */
    double offset_rad; /**< 零位偏置 */
} arms_joint_info;

/** @brief 读取关节的方向与零位偏置 */
arms_status arms_get_joint_info(const arms_t *arms, int joint, arms_joint_info *info);
/** @brief 版本字符串 */
const char *arms_version(void);

#ifdef __cplusplus
}
#endif

#endif
