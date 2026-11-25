#define _GNU_SOURCE
#include "arms/arms.h"

#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "bus.h"
#include "config.h"
#include "kgu.h"

#ifndef ARMS_VERSION_STRING
#define ARMS_VERSION_STRING "0.0.0"
#endif

#define STATE_POLL_HZ 10u        /* AL 状态与链路刷新频率 */
#define READY_STABLE_MS 200u     /* 全部 OP 且 WKC 正确持续该时长后判定预热完成 */
#define DEFAULT_SCAN_TIMEOUT_MS 10000u
#define DEFAULT_WAIT_MS 1000u          /* 使能、复位的默认等待时间 */
#define RESET_PULSE_MS 20u             /* 故障复位控制字 0x0080 保持时间 */
#define RECOVER_STABLE_MS 500u         /* 通信失败后，恢复稳定该时长才清除失败标志 */

/* 非实时线程向周期上下文发出的关节请求 */
enum { REQ_NONE = 0, REQ_ENABLE, REQ_DISABLE, REQ_RESET, REQ_HOLD };

/* 请求结果，由周期上下文写入 */
enum { RES_PENDING = 0, RES_OK, RES_TIMEOUT, RES_FAULT, RES_STATE, RES_COMM, RES_TRIP, RES_PROTECT };

/* 关节控制阶段 */
enum { CTL_OFF = 0, CTL_ENABLING, CTL_ON, CTL_RESETTING };

static __thread char last_error[512];

const char *arms_last_error(void) { return last_error; }

static void set_error(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(last_error, sizeof(last_error), fmt, ap);
    va_end(ap);
}

const char *arms_version(void) { return ARMS_VERSION_STRING " (" __DATE__ " " __TIME__ ")"; }

const char *arms_strerror(arms_status s)
{
    switch (s) {
    case ARMS_OK: return "成功";
    case ARMS_E_ARG: return "参数无效";
    case ARMS_E_CONFIG: return "配置文件无效";
    case ARMS_E_SYSTEM: return "系统调用失败";
    case ARMS_E_MASTER: return "EtherCAT 主站不可用或被占用";
    case ARMS_E_LINK: return "网口无物理链路";
    case ARMS_E_TOPOLOGY: return "总线上的从站与配置不符";
    case ARMS_E_STATE: return "状态不允许该操作";
    case ARMS_E_COMM: return "周期通信失败";
    case ARMS_E_TIMEOUT: return "超时";
    case ARMS_E_UNSUPPORTED: return "功能不支持";
    }
    return "未知错误";
}

const char *arms_joint_mode_name(arms_joint_mode m)
{
    switch (m) {
    case ARMS_JS_ABSENT: return "absent";
    case ARMS_JS_OFFLINE: return "offline";
    case ARMS_JS_PREOP: return "preop";
    case ARMS_JS_SAFEOP: return "safeop";
    case ARMS_JS_LOCKED: return "locked";
    case ARMS_JS_RELEASED: return "released";
    case ARMS_JS_HELD: return "held";
    case ARMS_JS_ENABLED: return "enabled";
    case ARMS_JS_FAULT: return "FAULT";
    }
    return "?";
}

const char *arms_arm_name(int arm) { return arm == ARMS_ARM_LEFT ? "left" : arm == ARMS_ARM_RIGHT ? "right" : "?"; }

void arms_options_init(arms_options *o)
{
    memset(o, 0, sizeof(*o));
    o->backend = ARMS_BACKEND_ETHERLAB;
    o->mode = ARMS_MODE_CONFIG;
    o->rt_cpu = -2;
    o->check_motors = -1;
    o->ignore_mask = -1;
}

/* ------------------------------------------------------------------ */

typedef struct {
    double sum, sumsq, min, max;
    uint64_t n;
} running_stat;

static void rs_reset(running_stat *r) { memset(r, 0, sizeof(*r)); }

static void rs_add(running_stat *r, double v)
{
    if (r->n == 0 || v < r->min) r->min = v;
    if (r->n == 0 || v > r->max) r->max = v;
    r->sum += v;
    r->sumsq += v * v;
    r->n++;
}

typedef struct {
    bool used;
    const bus_ops *ops;
    bus_t *bus;
    int count;
    int slot_joint[BUS_MAX_SLAVES];
    int slot_slave[BUS_MAX_SLAVES];
    float slot_pos[BUS_MAX_SLAVES];   /* 驱动器原始位置，用于失能时的目标跟随 */
    uint8_t al[BUS_MAX_SLAVES];
    bool link_up;
    int slaves_found;
    uint32_t wkc_expected;
    uint32_t wkc_last;
    uint64_t cycles, wkc_bad, send_errors;
    uint32_t consecutive, consecutive_max;
    bool comm_failed;
    bool wkc_ok;                      /* 本周期 WKC 正确 */
    uint64_t good_cycles;             /* WKC 正确且全部从站 OP 的连续周期数 */
    uint32_t drops, recoveries;
    bool auto_reset;                  /* 恢复后对故障关节执行一次复位 */
    double recv_us_max, send_us_max;
    /* 驱动器监控轮询（周期上下文）：ESC 错误计数与监控量各自轮流处理一个从站 */
    int esc_slot, esc_phase;          /* esc_phase: 0 发起，1 等待结果 */
    bool esc_clearing;
    bool esc_clear[BUS_MAX_SLAVES];
    bool esc_valid[BUS_MAX_SLAVES];
    uint8_t esc[BUS_MAX_SLAVES][ESC_ERR_BYTES];
    int mon_slot, mon_phase;          /* 见 monitor_step() */
    unsigned int mon_wait;
    double temp[BUS_MAX_SLAVES];
    int32_t drive_loss[BUS_MAX_SLAVES];
    bool temp_warned[BUS_MAX_SLAVES];
    /* 处理类错误（只有处理单元/转发错误）按时间窗口汇总：被拒绝的帧数、首个拒绝位置、窗口开始时刻 */
    uint32_t proc_frames;
    int proc_origin;
    uint64_t proc_since;
} arm_rt;

/* ---- 通信事件记录：周期上下文写入，管理线程读出后写日志文件 ---- */
#define TRACE_LEN 64u   /* 每条事件附带的最近周期数 */
#define EMERGENCY_MSG_BYTES 8 /* CoE 紧急报文长度 */
#define EMERGENCY_BATCH 32    /* 每次写日志最多取出的紧急报文数 */
#define EVENT_RING 16u  /* 尚未写出的事件最多保留条数 */
enum {
    EVT_WKC = 0, EVT_FAIL, EVT_RECOVER, EVT_LATE, EVT_TRIP,
    EVT_ESC,        /* ESC 错误计数增加 */
    EVT_DRIVE_LOSS, /* 驱动器自报的 EtherCAT 丢帧次数增加 */
    EVT_TEMP_WARN,  /* 温度达到告警阈值 */
    EVT_OVERTEMP,   /* 温度达到保护阈值，已锁定保护 */
    EVT_ESC_PROC,   /* 处理类错误汇总：从站拒绝的帧，线路类计数未变 */
    EVT_KINDS
};

/* 单个从站相关事件的细节 */
typedef struct {
    int joint, slave;               /* 关节号、链上位置；无则 -1 */
    double value, limit;            /* 温度事件：温度与阈值；丢帧事件：新旧计数 */
    uint8_t before[ESC_ERR_BYTES], after[ESC_ERR_BYTES];
} event_detail;

typedef struct {
    uint64_t cycle;
    uint64_t start_ns;          /* 周期开始时刻（CLOCK_MONOTONIC） */
    uint32_t period_ns;         /* 与上一周期开始的间隔 */
    uint32_t late_ns;           /* 唤醒延迟（独立模式） */
    uint32_t gap_ns;            /* 上一周期发送结束到本周期开始接收的间隔 */
    uint32_t work_ns;           /* 本周期从接收开始到发送结束的耗时 */
    uint16_t wkc[ARMS_ARM_COUNT];
} trace_entry;

typedef struct {
    int kind, which_arm;        /* which_arm 为 -1 表示与具体臂无关 */
    uint32_t suppressed;        /* 限速期间未单独记录的同类事件数 */
    uint64_t cycle, mono_ns;
    int64_t real_ns;
    uint64_t skipped, overruns;
    struct {
        bool used, link_up, comm_failed;
        int count;
        uint32_t wkc, wkc_expected, consecutive;
        uint64_t wkc_bad;
        uint8_t al[BUS_MAX_SLAVES];
        int joint[BUS_MAX_SLAVES];
        bool esc_valid[BUS_MAX_SLAVES];
        uint8_t esc[BUS_MAX_SLAVES][ESC_ERR_BYTES];
        double temp[BUS_MAX_SLAVES];
        int32_t drive_loss[BUS_MAX_SLAVES];
    } arm[ARMS_ARM_COUNT];
    bool has_detail;
    event_detail detail;
    unsigned int n;             /* trace 中的有效条数 */
    trace_entry trace[TRACE_LEN]; /* 旧 → 新 */
} event_record;

#define WORDS_UP(T) ((sizeof(T) + sizeof(uintptr_t) - 1) / sizeof(uintptr_t))
typedef union {
    event_record r;
    uintptr_t w[WORDS_UP(event_record)];
} event_buf;

typedef struct {
    arms_joint_state s[ARMS_JOINT_LAST + 1];
    arms_bus_stats stats;
} shared_snapshot;

/* 序列号只保证读到同一代数据；载荷也必须原子传递，避免 memcpy 与并发写产生数据竞争。 */
#define WORDS_OF(T) (sizeof(T) / sizeof(uintptr_t))
typedef struct {
    arms_command cmd;
} cmd_payload;
_Static_assert(sizeof(shared_snapshot) % sizeof(uintptr_t) == 0, "snapshot word alignment");
_Static_assert(sizeof(cmd_payload) % sizeof(uintptr_t) == 0, "command word alignment");

static void store_words(atomic_uintptr_t *dst, const void *src, size_t count)
{
    const unsigned char *bytes = src;
    for (size_t i = 0; i < count; ++i) {
        uintptr_t word;
        memcpy(&word, bytes + i * sizeof(word), sizeof(word));
        atomic_store_explicit(&dst[i], word, memory_order_relaxed);
    }
}

static void load_words(void *dst, const atomic_uintptr_t *src, size_t count)
{
    unsigned char *bytes = dst;
    for (size_t i = 0; i < count; ++i) {
        uintptr_t word = atomic_load_explicit(&src[i], memory_order_relaxed);
        memcpy(bytes + i * sizeof(word), &word, sizeof(word));
    }
}

/* 指令槽：非实时线程在 cmd_lock 下写入，周期上下文无锁读取（seqlock，读失败沿用上一周期） */
typedef struct {
    atomic_uint seq;
    atomic_uintptr_t data[WORDS_OF(cmd_payload)];
} cmd_slot;

/* 关节控制状态，仅在周期上下文中访问 */
typedef struct {
    int phase;
    uint64_t phase_cycle;
    unsigned int seq_seen;
    uint64_t fresh_cycle;
    arms_command cur;                 /* 当前生效的指令（关节侧，未限幅） */
    bool timed_out;
    bool was_fault;
    uint32_t fault_count;
    uint32_t over_vel, over_tor;      /* 超限累计周期数：超限 +1，未超限 -1 */
} joint_ctl;

struct arms {
    /* 抱闸模式记录（Pn075），由管理线程更新；掩码用原子量供读取接口显示 */
    atomic_uint brake_released;       /* Pn075 = 1 的关节 */
    atomic_uint brake_held;           /* Pn075 = 2 的关节 */
    atomic_uint brake_known;          /* 最近一次 Pn075 回读有效的关节 */
    _Atomic double offset_rad[ARMS_JOINT_LAST + 1]; /* 管理线程更新，周期线程读取 */
    atomic_uint zero_pending;         /* 已硬件校零、等待驱动器重新上电的关节 */
    arms_mask brake_touched;          /* 本次会话改动过抱闸模式的关节 */
    arms_config cfg;
    arms_options opt;
    const bus_ops *ops;
    arms_run_mode mode;
    int rt_cpu;
    unsigned int cycle_hz;
    uint64_t period_ns;
    arms_mask joints;
    arms_mask ignored;                /* 屏蔽掩码 */
    arms_mask missing;                /* 启动时不在线、未打开的关节 */
    bool power_off_action;            /* safety.offline_action = power_off */
    arm_rt arm[ARMS_ARM_COUNT];
    atomic_bool clear_trip;
    atomic_bool clear_protect;

    pthread_t thread;
    bool thread_started;
    atomic_bool stop;
    atomic_bool ready;
    atomic_bool reset_stats;
    atomic_uint seq;
    shared_snapshot snap; /* 写端工作区，不与读取线程直接共享 */
    atomic_uintptr_t published[WORDS_OF(shared_snapshot)];

    pthread_mutex_t cmd_lock;
    cmd_slot slot[ARMS_JOINT_LAST + 1];
    atomic_uint req[ARMS_JOINT_LAST + 1];
    atomic_uint res[ARMS_JOINT_LAST + 1];
    atomic_bool clear_comm[ARMS_ARM_COUNT];
    atomic_uint enable_timeout_cycles;

    /* 以下仅在周期上下文中访问 */
    joint_ctl ctl[ARMS_JOINT_LAST + 1];
    bool tripped;
    arms_protect protect;
    uint64_t overspeed_cycles, overtorque_cycles;
    arms_mask trip_joints, offline_joints;
    uint64_t timeout_cycles;
    uint64_t reset_pulse_cycles;
    uint64_t recover_cycles;
    arms_joint_state js[ARMS_JOINT_LAST + 1];
    running_stat period;
    uint64_t cycles, stats_origin, overruns, skipped; /* cycles 是单调控制时钟，不随统计清零 */
    double wake_late_max, work_max;
    uint64_t last_start_ns;
    uint64_t cycle_start_ns;
    unsigned int poll_div, poll_cnt;
    uint64_t stable_cycles, ready_cycles;
    uint64_t open_ns, activated_ns;   /* arms_open 开始、主站激活的时刻 */
    atomic_ullong ready_ns;           /* 预热完成的时刻 */

    /* 通信事件记录。trace 与限速计数仅在周期上下文访问；事件环按 seqlock 方式发布 */
    trace_entry trace[TRACE_LEN];
    uint64_t trace_count;
    uint64_t wake_late_ns, last_tx_end_ns;
    uint64_t evt_last_ns[EVT_KINDS][ARMS_ARM_COUNT];
    uint32_t evt_suppressed[EVT_KINDS][ARMS_ARM_COUNT];
    event_buf evt_scratch;
    atomic_uintptr_t evt_words[EVENT_RING][WORDS_UP(event_record)];
    atomic_ullong evt_begin;          /* 正在写入的事件序号 + 1 */
    atomic_ullong evt_count;          /* 已完整写入的事件数 */
    uint64_t evt_read;                /* 已写到日志文件的事件数，仅管理线程访问 */
    bool evt_header_done;
    char event_log[PATH_MAX];

    /* 驱动器监控轮询节拍（周期上下文）；清零请求由管理线程发出 */
    unsigned int diag_div, diag_cnt;
    atomic_bool esc_clear_all;
};

static void post(arms_t *a, arms_mask joints, unsigned int req);

static uint64_t mono_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static void logf_(arms_t *a, const char *fmt, ...)
{
    char line[512];
    va_list ap;
    if (!a->opt.log) return;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    a->opt.log(a->opt.log_user, line);
}

static void bus_log_adapter(void *user, const char *fmt, ...)
{
    arms_t *a = user;
    char line[512];
    va_list ap;
    if (!a->opt.log) return;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    a->opt.log(a->opt.log_user, line);
}

static arms_joint_mode mode_from(uint8_t al, uint16_t sw)
{
    if (al == 0) return ARMS_JS_OFFLINE;
    if (al & 0x10) return ARMS_JS_FAULT;
    switch (al & 0x0F) {
    case 0x01: case 0x02: case 0x03: return ARMS_JS_PREOP;
    case 0x04: return ARMS_JS_SAFEOP;
    case 0x08:
        if (sw & KGU_SW_FAULT_BIT) return ARMS_JS_FAULT;
        if ((sw & 0x006F) == 0x0027) return ARMS_JS_ENABLED;
        return ARMS_JS_LOCKED;
    }
    return ARMS_JS_OFFLINE;
}

/* ---- 快照（单写多读，seqlock） ---- */

static void publish(arms_t *a)
{
    int i;
    memcpy(a->snap.s, a->js, sizeof(a->js));
    a->snap.stats.cycle_hz = a->cycle_hz;
    a->snap.stats.cycles = a->cycles - a->stats_origin;
    a->snap.stats.overruns = a->overruns;
    a->snap.stats.skipped = a->skipped;
    if (a->period.n) {
        double mean = a->period.sum / (double)a->period.n;
        double var = a->period.sumsq / (double)a->period.n - mean * mean;
        double dev = (double)a->period_ns - mean;
        a->snap.stats.period.min = a->period.min / 1000.0;
        a->snap.stats.period.max = a->period.max / 1000.0;
        a->snap.stats.period.mean = mean / 1000.0;
        a->snap.stats.period.rms_jitter = sqrt((var > 0 ? var : 0) + dev * dev) / 1000.0;
    } else {
        memset(&a->snap.stats.period, 0, sizeof(a->snap.stats.period));
    }
    a->snap.stats.offline_joints = a->offline_joints;
    a->snap.stats.tripped = a->tripped;
    a->snap.stats.trip_joints = a->trip_joints;
    a->snap.stats.protect = a->protect;
    a->snap.stats.wake_late_us_max = a->wake_late_max / 1000.0;
    a->snap.stats.work_us_max = a->work_max / 1000.0;
    for (i = 0; i < ARMS_ARM_COUNT; ++i) {
        const arm_rt *r = &a->arm[i];
        arms_arm_stats *o = &a->snap.stats.arm[i];
        int k, op = 0;
        if (!r->used) continue;
        for (k = 0; k < r->count; ++k) op += (r->al[k] & 0x0F) == 0x08;
        o->slaves_op = op;
        o->link_up = r->link_up;
        o->wkc_expected = r->wkc_expected;
        o->wkc_last = r->wkc_last;
        o->cycles = r->cycles;
        o->wkc_bad = r->wkc_bad;
        o->wkc_bad_consecutive_max = r->consecutive_max;
        o->send_errors = r->send_errors;
        o->comm_failed = r->comm_failed;
        o->comm_drops = r->drops;
        o->comm_recoveries = r->recoveries;
        o->recv_us_max = r->recv_us_max;
        o->send_us_max = r->send_us_max;
    }
    atomic_fetch_add_explicit(&a->seq, 1, memory_order_acq_rel);
    atomic_thread_fence(memory_order_release);
    store_words(a->published, &a->snap, WORDS_OF(shared_snapshot));
    atomic_fetch_add_explicit(&a->seq, 1, memory_order_acq_rel);
}

static void read_snapshot(arms_t *a, shared_snapshot *out)
{
    unsigned int s1, s2 = 0;
    do {
        s1 = atomic_load_explicit(&a->seq, memory_order_acquire);
        if (s1 & 1u) continue;
        atomic_thread_fence(memory_order_acquire);
        load_words(out, a->published, WORDS_OF(shared_snapshot));
        atomic_thread_fence(memory_order_acquire);
        s2 = atomic_load_explicit(&a->seq, memory_order_acquire);
    } while ((s1 & 1u) || s1 != s2);
}

static void do_reset_stats(arms_t *a)
{
    int i;
    rs_reset(&a->period);
    a->stats_origin = a->cycles;
    a->overruns = a->skipped = 0;
    a->wake_late_max = a->work_max = 0;
    a->last_start_ns = 0;
    for (i = 0; i < ARMS_ARM_COUNT; ++i) {
        arm_rt *r = &a->arm[i];
        r->cycles = r->wkc_bad = r->send_errors = 0;
        r->consecutive_max = 0;
        r->recv_us_max = r->send_us_max = 0;
    }
}

/*
 * 反馈超限保护（仅对使能中的关节）：速度或力矩超过阈值时计数加 1，否则减 1，
 * 计数达到 safety.overspeed_ms / overtorque_ms 对应的周期数时锁定保护，全部关节失能。
 */
static void check_limits(arms_t *a, int j, const arms_joint_state *s)
{
    const arms_joint_config *jc = &a->cfg.joint[j];
    joint_ctl *c = &a->ctl[j];
    bool ov, ot;

    if (c->phase != CTL_ON) {
        c->over_vel = c->over_tor = 0;
        return;
    }
    ov = fabs(s->velocity) > jc->vel_limit;
    ot = fabs(s->torque) > jc->tor_limit;
    c->over_vel = ov ? c->over_vel + 1 : (c->over_vel ? c->over_vel - 1 : 0);
    c->over_tor = ot ? c->over_tor + 1 : (c->over_tor ? c->over_tor - 1 : 0);
    if (a->protect.reason != ARMS_PROTECT_NONE) return;
    if (c->over_vel >= a->overspeed_cycles) {
        a->protect.reason = ARMS_PROTECT_OVERSPEED;
        a->protect.joint = j;
        a->protect.value = s->velocity;
        a->protect.limit = jc->vel_limit;
    } else if (c->over_tor >= a->overtorque_cycles) {
        a->protect.reason = ARMS_PROTECT_OVERTORQUE;
        a->protect.joint = j;
        a->protect.value = s->torque;
        a->protect.limit = jc->tor_limit;
    }
}

/*
 * 记录一条通信事件（周期上下文）：复制最近的周期时序和两臂状态，按 seqlock 方式发布到事件环。
 * WKC 异常、周期延迟、ESC 错误计数和驱动器丢帧每臂每秒最多记录一条，其余计入下一条的 suppressed。
 */
static void record_event(arms_t *a, int kind, int arm, uint64_t now, const event_detail *d)
{
    int ai = arm < 0 ? 0 : arm;
    uint64_t n = atomic_load_explicit(&a->evt_count, memory_order_relaxed);
    event_record *e = &a->evt_scratch.r;
    struct timespec rt;
    unsigned int k;
    bool limited = kind == EVT_WKC || kind == EVT_LATE || kind == EVT_ESC || kind == EVT_DRIVE_LOSS;

    if (limited && a->evt_last_ns[kind][ai] && now - a->evt_last_ns[kind][ai] < 1000000000ull) {
        a->evt_suppressed[kind][ai]++;
        return;
    }
    a->evt_last_ns[kind][ai] = now;
    memset(&a->evt_scratch, 0, sizeof(a->evt_scratch));
    e->kind = kind;
    e->which_arm = arm;
    e->suppressed = a->evt_suppressed[kind][ai];
    a->evt_suppressed[kind][ai] = 0;
    e->cycle = a->cycles;
    e->mono_ns = now;
    clock_gettime(CLOCK_REALTIME, &rt);
    e->real_ns = (int64_t)rt.tv_sec * 1000000000LL + rt.tv_nsec;
    e->skipped = a->skipped;
    e->overruns = a->overruns;
    for (int i = 0; i < ARMS_ARM_COUNT; ++i) {
        const arm_rt *r = &a->arm[i];
        e->arm[i].used = r->used;
        if (!r->used) continue;
        e->arm[i].link_up = r->link_up;
        e->arm[i].comm_failed = r->comm_failed;
        e->arm[i].count = r->count;
        e->arm[i].wkc = r->wkc_last;
        e->arm[i].wkc_expected = r->wkc_expected;
        e->arm[i].consecutive = r->consecutive;
        e->arm[i].wkc_bad = r->wkc_bad;
        memcpy(e->arm[i].al, r->al, sizeof(e->arm[i].al));
        for (int s = 0; s < r->count; ++s) {
            e->arm[i].joint[s] = r->slot_joint[s];
            e->arm[i].esc_valid[s] = r->esc_valid[s];
            memcpy(e->arm[i].esc[s], r->esc[s], ESC_ERR_BYTES);
            e->arm[i].temp[s] = r->temp[s];
            e->arm[i].drive_loss[s] = r->drive_loss[s];
        }
    }
    if (d) {
        e->has_detail = true;
        e->detail = *d;
    }
    e->n = a->trace_count < TRACE_LEN ? (unsigned int)a->trace_count : TRACE_LEN;
    for (k = 0; k < e->n; ++k) e->trace[k] = a->trace[(a->trace_count - e->n + k) % TRACE_LEN];

    atomic_store_explicit(&a->evt_begin, n + 1, memory_order_relaxed);
    atomic_thread_fence(memory_order_release);
    store_words(a->evt_words[n % EVENT_RING], &a->evt_scratch, WORDS_UP(event_record));
    atomic_store_explicit(&a->evt_count, n + 1, memory_order_release);
}

/* 通信事件日志的默认位置：配置文件在 config/ 目录时为同级的 logs/，否则为配置目录下的 logs/ */
static void resolve_event_log(arms_t *a, const char *path)
{
    char dir[PATH_MAX], abs_dir[PATH_MAX];
    const char *slash, *base;
    if (path && *path) {
        snprintf(a->event_log, sizeof(a->event_log), "%s", path);
        return;
    }
    slash = strrchr(a->cfg.path, '/');
    if (slash) snprintf(dir, sizeof(dir), "%.*s", (int)(slash - a->cfg.path), a->cfg.path);
    else snprintf(dir, sizeof(dir), ".");
    if (realpath(dir, abs_dir)) snprintf(dir, sizeof(dir), "%s", abs_dir);
    base = strrchr(dir, '/');
    base = base ? base + 1 : dir;
    snprintf(a->event_log, sizeof(a->event_log), strcmp(base, "config") == 0 ? "%s/../logs/comm_events.log"
                                                                              : "%s/logs/comm_events.log", dir);
}

/* ---- 驱动器监控：ESC 错误计数、温度、驱动器丢帧次数（周期上下文，异步请求，不阻塞） ---- */

/* ESC 错误计数寄存器 0x0300 起各字节的含义；NULL 为错误码或保留字节，不参与比较 */
static const char *const esc_names[ESC_ERR_BYTES] = {
    "端口0无效帧", "端口0物理层错误", "端口1无效帧", "端口1物理层错误",
    "端口2无效帧", "端口2物理层错误", "端口3无效帧", "端口3物理层错误",
    "端口0转发错误", "端口1转发错误", "端口2转发错误", "端口3转发错误",
    "处理单元错误", "PDI错误", NULL, NULL,
    "端口0链路丢失", "端口1链路丢失", "端口2链路丢失", "端口3链路丢失",
};

/*
 * 线路类计数：各端口无效帧、物理层错误、链路丢失，反映网线、接头或从站网口的问题。
 * 其余（转发错误、处理单元错误、PDI 错误）为处理类：帧在电气上完好，但被从站判为无效，
 * 只出现处理类计数时，通常是主机发往 EtherCAT 网口的非 EtherCAT 报文。
 */
static bool esc_line_counter(unsigned int b) { return b < 8 || (b >= 16 && b < 20); }
#define ESC_PU_ERR 12u   /* 处理单元错误 */
#define ESC_FWD_PORT0 8u /* 端口 0 转发错误 */

/*
 * 轮流读取（或按请求清零）每个从站的 ESC 错误计数。线路类计数增加时逐条记录；
 * 只有处理类计数增加时累计被拒绝的帧数，由 diag_tick() 按窗口汇总。
 * 某从站新拒绝的帧数 = 处理单元错误增量 − 端口 0 转发错误增量（后者是上游已标记的帧）。
 */
static void esc_step(arms_t *a, int i, uint64_t now)
{
    arm_rt *r = &a->arm[i];
    int k = r->esc_slot, s;
    uint8_t v[ESC_ERR_BYTES];

    if (r->esc_phase == 0) {
        bool clear = r->esc_clear[k];
        if ((clear ? r->ops->esc_counters_clear(r->bus, k) : r->ops->esc_counters_read(r->bus, k)) == 0) {
            r->esc_clearing = clear;
            r->esc_phase = 1;
        }
        return;
    }
    s = r->ops->esc_counters_state(r->bus, k, v);
    if (s == BUS_REQ_BUSY) return;
    if (s == BUS_REQ_DONE) {
        if (r->esc_clearing) {
            memset(r->esc[k], 0, ESC_ERR_BYTES);
            r->esc_clear[k] = false;
        } else {
            bool line = false, proc = false;
            for (unsigned int b = 0; b < ESC_ERR_BYTES; ++b) {
                if (!esc_names[b] || !r->esc_valid[k] || v[b] <= r->esc[k][b]) continue;
                if (esc_line_counter(b)) line = true;
                else proc = true;
            }
            if (proc && !line) {
                int pu = v[ESC_PU_ERR] > r->esc[k][ESC_PU_ERR] ? v[ESC_PU_ERR] - r->esc[k][ESC_PU_ERR] : 0;
                int fwd = v[ESC_FWD_PORT0] > r->esc[k][ESC_FWD_PORT0] ? v[ESC_FWD_PORT0] - r->esc[k][ESC_FWD_PORT0] : 0;
                if (pu > fwd) {
                    if (!r->proc_frames) {
                        r->proc_since = now;
                        r->proc_origin = k;
                    }
                    r->proc_frames += (uint32_t)(pu - fwd);
                }
            }
            if (line) {
                event_detail d;
                memset(&d, 0, sizeof(d));
                d.joint = r->slot_joint[k];
                d.slave = r->slot_slave[k];
                memcpy(d.before, r->esc[k], ESC_ERR_BYTES);
                memcpy(d.after, v, ESC_ERR_BYTES);
                memcpy(r->esc[k], v, ESC_ERR_BYTES);
                record_event(a, EVT_ESC, i, now, &d);
            } else {
                memcpy(r->esc[k], v, ESC_ERR_BYTES);
            }
        }
        r->esc_valid[k] = true;
    }
    r->esc_phase = 0;
    r->esc_slot = (k + 1) % r->count;
}

static void temperature_sample(arms_t *a, int i, int k, double t, uint64_t now)
{
    arm_rt *r = &a->arm[i];
    event_detail d;
    if (t < -40.0 || t > 200.0) return; /* 不可信的读数不参与保护 */
    r->temp[k] = t;
    memset(&d, 0, sizeof(d));
    d.joint = r->slot_joint[k];
    d.slave = r->slot_slave[k];
    d.value = t;
    if (t >= a->cfg.temp_max_c) {
        if (a->protect.reason == ARMS_PROTECT_NONE) {
            a->protect.reason = ARMS_PROTECT_OVERTEMP;
            a->protect.joint = d.joint;
            a->protect.value = t;
            a->protect.limit = a->cfg.temp_max_c;
            d.limit = a->cfg.temp_max_c;
            record_event(a, EVT_OVERTEMP, i, now, &d);
        }
    } else if (t >= a->cfg.temp_warn_c) {
        if (!r->temp_warned[k]) {
            r->temp_warned[k] = true;
            d.limit = a->cfg.temp_warn_c;
            record_event(a, EVT_TEMP_WARN, i, now, &d);
        }
    } else if (t < a->cfg.temp_warn_c - 5.0) {
        r->temp_warned[k] = false;
    }
}

static void loss_sample(arms_t *a, int i, int k, int32_t v, uint64_t now)
{
    arm_rt *r = &a->arm[i];
    if (r->drive_loss[k] >= 0 && v > r->drive_loss[k]) {
        event_detail d;
        memset(&d, 0, sizeof(d));
        d.joint = r->slot_joint[k];
        d.slave = r->slot_slave[k];
        d.limit = r->drive_loss[k];
        d.value = v;
        r->drive_loss[k] = v;
        record_event(a, EVT_DRIVE_LOSS, i, now, &d);
    } else {
        r->drive_loss[k] = v;
    }
}

/*
 * 监控量轮询：选择温度 → 等 0x3000 刷新 → 读取 → 选择丢帧次数 → 等待 → 读取 → 下一个从站。
 * 驱动器在选择后需要时间刷新 0x3000，等待避免把上一个监控量读成当前量。失败时跳到下一个从站。
 */
enum {
    MON_SEL_TEMP, MON_WAIT_SEL_TEMP, MON_SETTLE_TEMP, MON_READ_TEMP, MON_WAIT_TEMP,
    MON_SEL_LOSS, MON_WAIT_SEL_LOSS, MON_SETTLE_LOSS, MON_READ_LOSS, MON_WAIT_LOSS,
};
#define MON_SETTLE_TICKS 10u /* 选择监控量后等待的轮询节拍数（每节拍 10 ms） */

static void monitor_next(arm_rt *r)
{
    r->mon_phase = MON_SEL_TEMP;
    r->mon_wait = 0;
    r->mon_slot = (r->mon_slot + 1) % r->count;
}

static void monitor_step(arms_t *a, int i, uint64_t now)
{
    arm_rt *r = &a->arm[i];
    int k = r->mon_slot, s;
    int32_t v = 0;
    switch (r->mon_phase) {
    case MON_SEL_TEMP:
    case MON_SEL_LOSS:
        if (r->ops->monitor_select(r->bus, k,
                                   r->mon_phase == MON_SEL_TEMP ? KGU_MON_TEMPERATURE : KGU_MON_ECAT_LOSS) == 0)
            r->mon_phase++;
        return;
    case MON_SETTLE_TEMP:
    case MON_SETTLE_LOSS:
        if (++r->mon_wait >= MON_SETTLE_TICKS) {
            r->mon_wait = 0;
            r->mon_phase++;
        }
        return;
    case MON_READ_TEMP:
    case MON_READ_LOSS:
        if (r->ops->monitor_read(r->bus, k) == 0) r->mon_phase++;
        return;
    default: /* 等待选择或读取的结果 */
        s = r->ops->monitor_state(r->bus, k, &v);
        if (s == BUS_REQ_BUSY) return;
        if (s != BUS_REQ_DONE) {
            monitor_next(r);
            return;
        }
        if (r->mon_phase == MON_WAIT_TEMP) temperature_sample(a, i, k, (double)v, now);
        if (r->mon_phase == MON_WAIT_LOSS) {
            loss_sample(a, i, k, v, now);
            monitor_next(r);
            return;
        }
        r->mon_phase++;
    }
}

/* 每 10 ms 推进一步；通信异常的臂暂停，恢复后继续 */
static void diag_tick(arms_t *a, uint64_t now)
{
    bool clear_all = atomic_exchange_explicit(&a->esc_clear_all, false, memory_order_acq_rel);
    for (int i = 0; i < ARMS_ARM_COUNT; ++i) {
        arm_rt *r = &a->arm[i];
        if (!r->used || r->count <= 0) continue;
        if (clear_all)
            for (int k = 0; k < r->count; ++k) r->esc_clear[k] = true;
        if (r->comm_failed || !r->wkc_ok) continue;
        esc_step(a, i, now);
        monitor_step(a, i, now);
        if (r->proc_frames && now - r->proc_since >= (uint64_t)a->cfg.esc_summary_s * 1000000000ull) {
            event_detail d;
            memset(&d, 0, sizeof(d));
            d.joint = r->slot_joint[r->proc_origin];
            d.slave = r->slot_slave[r->proc_origin];
            d.value = r->proc_frames;
            d.limit = (double)(now - r->proc_since) / 1e9;
            record_event(a, EVT_ESC_PROC, i, now, &d);
            r->proc_frames = 0;
        }
        for (int k = 0; k < r->count; ++k) {
            arms_joint_state *s = &a->js[r->slot_joint[k]];
            s->temperature = r->temp[k];
            s->drive_frame_loss = r->drive_loss[k];
            s->esc_valid = r->esc_valid[k];
            memcpy(s->esc_errors, r->esc[k], ESC_ERR_BYTES);
        }
    }
}

/* ---- 周期处理，独立模式与外部驱动模式共用 ---- */

static void cycle_rx(arms_t *a, uint64_t now)
{
    bool ready = atomic_load_explicit(&a->ready, memory_order_relaxed);
    bool all_good = true, trip_now = false, late_cycle = false;
    bool ev_wkc[ARMS_ARM_COUNT] = {false}, ev_fail[ARMS_ARM_COUNT] = {false}, ev_rec[ARMS_ARM_COUNT] = {false};
    uint64_t dt = 0, gap = 0, rx_ns;
    int i, k;

    if (atomic_exchange_explicit(&a->reset_stats, false, memory_order_acq_rel)) do_reset_stats(a);
    a->cycle_start_ns = now;
    if (a->last_start_ns && now > a->last_start_ns) {
        dt = now - a->last_start_ns;
        rs_add(&a->period, (double)dt);
        if (dt * 2 > a->period_ns * 3) {
            a->overruns++;
            late_cycle = true;
        }
    }
    a->last_start_ns = now;
    rx_ns = mono_ns();
    if (a->last_tx_end_ns && rx_ns > a->last_tx_end_ns) gap = rx_ns - a->last_tx_end_ns;

    for (i = 0; i < ARMS_ARM_COUNT; ++i) {
        arm_rt *r = &a->arm[i];
        uint64_t t0, t1;
        int wkc;
        bool arm_op = true;
        if (!r->used) continue;
        t0 = mono_ns();
        wkc = r->ops->receive(r->bus);
        t1 = mono_ns();
        if ((double)(t1 - t0) / 1000.0 > r->recv_us_max) r->recv_us_max = (double)(t1 - t0) / 1000.0;
        r->cycles++;
        r->wkc_last = wkc < 0 ? 0 : (uint32_t)wkc;
        r->wkc_ok = wkc >= 0 && (uint32_t)wkc == r->wkc_expected;
        if (atomic_exchange_explicit(&a->clear_comm[i], false, memory_order_acq_rel) && r->consecutive == 0)
            r->comm_failed = false;
        if (!r->wkc_ok) {
            all_good = false;
            if (ready) {
                r->wkc_bad++;
                r->consecutive++;
                if (r->consecutive > r->consecutive_max) r->consecutive_max = r->consecutive;
                if (r->consecutive == 1 && !r->comm_failed) ev_wkc[i] = true;
                if (r->consecutive >= a->cfg.wkc_tolerance && !r->comm_failed) {
                    r->comm_failed = true;
                    ev_fail[i] = true;
                    for (int k = 0; k < r->count; ++k) {
                        arms_mask bit = ARMS_MASK_JOINT(r->slot_joint[k]);
                        atomic_fetch_and(&a->brake_known, ~bit);
                        /* 驱动器断电后硬件零点生效，软件偏置同时切换为配置中已写入的 0 */
                        if (atomic_fetch_and(&a->zero_pending, ~bit) & bit)
                            atomic_store(&a->offset_rad[r->slot_joint[k]], 0.0);
                    }
                    r->drops++;
                }
            }
        } else {
            r->consecutive = 0;
        }
        for (k = 0; k < r->count; ++k) {
            int j = r->slot_joint[k];
            const arms_joint_config *jc = &a->cfg.joint[j];
            arms_joint_state *s = &a->js[j];
            if (r->wkc_ok) {
                kgu_feedback f;
                memcpy(s->txpdo, r->ops->inputs(r->bus, k), sizeof(s->txpdo));
                kgu_decode(s->txpdo, &f);
                r->slot_pos[k] = f.pos;
                s->status_word = f.statusword;
                s->error_code = f.error;
                s->position = jc->direction * (double)f.pos - atomic_load_explicit(&a->offset_rad[j], memory_order_relaxed);
                s->velocity = jc->direction * (double)f.vel;
                s->torque = jc->direction * (double)f.torque;
                s->sample_ns = t1;
                check_limits(a, j, s);
            }
            s->al_state = r->al[k];
            s->mode = mode_from(r->al[k], s->status_word);
            if ((r->al[k] & 0x0F) != 0x08) {
                all_good = false;
                arm_op = false;
            }
        }

        /* 掉电或断线后，从站由主站重新配置回 OP；稳定后自动清除通信失败（不自动使能） */
        r->good_cycles = r->wkc_ok && arm_op ? r->good_cycles + 1 : 0;
        if (r->comm_failed && r->good_cycles >= a->recover_cycles) {
            r->comm_failed = false;
            r->consecutive = 0;
            r->recoveries++;
            r->auto_reset = true;
            ev_rec[i] = true;
            /* 驱动器可能已重新上电：错误计数从零重新累计，丢帧次数重新建立基准 */
            for (k = 0; k < r->count; ++k) {
                r->esc_clear[k] = true;
                r->drive_loss[k] = -1;
            }
        }
    }

    {
        trace_entry *t = &a->trace[a->trace_count % TRACE_LEN];
        t->cycle = a->cycles;
        t->start_ns = now;
        t->period_ns = dt > UINT32_MAX ? UINT32_MAX : (uint32_t)dt;
        t->late_ns = a->wake_late_ns > UINT32_MAX ? UINT32_MAX : (uint32_t)a->wake_late_ns;
        t->gap_ns = gap > UINT32_MAX ? UINT32_MAX : (uint32_t)gap;
        t->work_ns = 0;
        for (i = 0; i < ARMS_ARM_COUNT; ++i)
            t->wkc[i] = a->arm[i].used ? (uint16_t)a->arm[i].wkc_last : 0;
        a->trace_count++;
    }

    if (++a->poll_cnt >= a->poll_div) {
        a->poll_cnt = 0;
        for (i = 0; i < ARMS_ARM_COUNT; ++i)
            if (a->arm[i].used) a->arm[i].ops->update_states(a->arm[i].bus, a->arm[i].al, &a->arm[i].link_up);
    }

    /* 离线检测：AL 无响应的从站；通信失败的臂视为全部离线 */
    {
        arms_mask offline = 0, failed = 0;
        for (i = 0; i < ARMS_ARM_COUNT; ++i) {
            arm_rt *r = &a->arm[i];
            if (!r->used) continue;
            for (k = 0; k < r->count; ++k) {
                if (r->al[k] == 0) offline |= ARMS_MASK_JOINT(r->slot_joint[k]);
                if (r->comm_failed) failed |= ARMS_MASK_JOINT(r->slot_joint[k]);
            }
        }
        a->offline_joints = offline;
        if (atomic_exchange_explicit(&a->clear_trip, false, memory_order_acq_rel) && !offline && !failed)
            a->tripped = false;
        if (atomic_exchange_explicit(&a->clear_protect, false, memory_order_acq_rel)) {
            a->protect.reason = ARMS_PROTECT_NONE;
            a->protect.joint = -1;
            a->protect.value = a->protect.limit = 0.0;
        }
        if (ready && a->power_off_action && !a->tripped && (offline || failed)) {
            a->tripped = true;
            a->trip_joints = offline ? offline : failed;
            trip_now = true;
        }
    }

    if (ready) {
        for (i = 0; i < ARMS_ARM_COUNT; ++i) {
            if (ev_fail[i]) record_event(a, EVT_FAIL, i, now, NULL);
            else if (ev_wkc[i]) record_event(a, EVT_WKC, i, now, NULL);
            if (ev_rec[i]) record_event(a, EVT_RECOVER, i, now, NULL);
        }
        if (late_cycle) record_event(a, EVT_LATE, -1, now, NULL);
        if (trip_now) record_event(a, EVT_TRIP, -1, now, NULL);
        if (a->cfg.drive_monitor && ++a->diag_cnt >= a->diag_div) {
            a->diag_cnt = 0;
            diag_tick(a, now);
        }
    }

    if (!ready) {
        a->stable_cycles = all_good ? a->stable_cycles + 1 : 0;
        if (a->stable_cycles >= a->ready_cycles) {
            /* 预热完成：ESC 错误计数从零开始累计，排除上电过程中产生的计数 */
            for (i = 0; i < ARMS_ARM_COUNT; ++i)
                for (k = 0; k < a->arm[i].count; ++k) a->arm[i].esc_clear[k] = true;
            do_reset_stats(a);
            atomic_store_explicit(&a->ready_ns, now, memory_order_relaxed);
            atomic_store_explicit(&a->ready, true, memory_order_release);
        }
    }
}


static void finish(arms_t *a, int j, unsigned int result)
{
    atomic_store_explicit(&a->res[j], result, memory_order_release);
}

/* 尝试读取指令槽；有新指令时返回 true */
static bool fetch_command(arms_t *a, int j, arms_command *cmd)
{
    cmd_slot *sl = &a->slot[j];
    joint_ctl *c = &a->ctl[j];
    unsigned int s1, s2;
    cmd_payload data;

    s1 = atomic_load_explicit(&sl->seq, memory_order_acquire);
    if ((s1 & 1u) || s1 == c->seq_seen) return false;
    load_words(&data, sl->data, WORDS_OF(cmd_payload));
    atomic_thread_fence(memory_order_acquire);
    s2 = atomic_load_explicit(&sl->seq, memory_order_relaxed);
    if (s1 != s2) return false;
    c->seq_seen = s1;
    *cmd = data.cmd;
    return true;
}

/* 单个关节的 CiA402 与 MIT 指令处理，生成本周期的过程数据 */
static void control_joint(arms_t *a, arm_rt *r, int k, bool ready, kgu_command *out)
{
    int j = r->slot_joint[k];
    const arms_joint_config *jc = &a->cfg.joint[j];
    arms_joint_state *s = &a->js[j];
    joint_ctl *c = &a->ctl[j];
    uint16_t sw = s->status_word;
    bool fault = s->mode == ARMS_JS_FAULT;
    bool op = (r->al[k] & 0x0F) == 0x08;
    bool comm_ok = r->wkc_ok && !r->comm_failed;
    unsigned int req = atomic_exchange_explicit(&a->req[j], REQ_NONE, memory_order_acq_rel);
    bool active, blocked = false;

    if (fault && !c->was_fault) c->fault_count++;
    c->was_fault = fault;

    /* 通信恢复后，复位掉电或同步丢失引起的驱动器故障；关节保持失能 */
    if (r->auto_reset && fault && c->phase != CTL_RESETTING) {
        c->phase = CTL_RESETTING;
        c->phase_cycle = a->cycles;
    }

    switch (req) {
    case REQ_DISABLE:
        if (c->phase == CTL_ENABLING) finish(a, j, RES_STATE);
        c->phase = CTL_OFF;
        break;
    case REQ_ENABLE:
        if (a->tripped) {
            finish(a, j, RES_TRIP);
        } else if (a->protect.reason != ARMS_PROTECT_NONE) {
            finish(a, j, RES_PROTECT);
        } else if (c->phase == CTL_ON) {
            finish(a, j, RES_OK);
        } else if (!ready || !comm_ok) {
            finish(a, j, RES_COMM);
        } else if (fault) {
            finish(a, j, RES_FAULT);
        } else if (!op || c->phase == CTL_RESETTING) {
            finish(a, j, RES_STATE);
        } else {
            c->phase = CTL_ENABLING;
            c->phase_cycle = a->cycles;
            /* 与参考 hw_lib 一致：使能后指令为零（零力矩），目标位置取当前位置 */
            c->cur.position = s->position;
            c->cur.velocity = 0.0;
            c->cur.torque = 0.0;
            c->cur.kp = 0.0;
            c->cur.kd = 0.0;
            c->timed_out = false;
            c->seq_seen = atomic_load_explicit(&a->slot[j].seq, memory_order_acquire) & ~1u;
            c->fresh_cycle = a->cycles;
        }
        break;
    case REQ_RESET:
        if (c->phase == CTL_ENABLING) finish(a, j, RES_STATE);
        c->phase = CTL_RESETTING;
        c->phase_cycle = a->cycles;
        break;
    case REQ_HOLD:
        if (c->phase == CTL_ON) {
            c->cur.velocity = 0.0;
            c->cur.torque = 0.0;
            c->fresh_cycle = a->cycles;
        }
        break;
    default:
        break;
    }

    {
        bool protect = a->protect.reason != ARMS_PROTECT_NONE;
        if ((c->phase == CTL_ENABLING || c->phase == CTL_ON) &&
            (fault || !op || r->comm_failed || a->tripped || protect)) {
            if (c->phase == CTL_ENABLING)
                finish(a, j, a->tripped ? RES_TRIP : protect ? RES_PROTECT : fault ? RES_FAULT : RES_COMM);
            c->phase = CTL_OFF;
        }
    }

    out->mode = KGU_MODE_MIT;
    switch (c->phase) {
    case CTL_ENABLING: {
        unsigned int st = sw & 0x006Fu;
        if ((sw & 0x004Fu) == 0x0040u) out->controlword = KGU_CW_SHUTDOWN;
        else if (st == 0x0021u) out->controlword = KGU_CW_SWITCH_ON;
        else if (st == 0x0023u || st == 0x0027u) out->controlword = KGU_CW_ENABLE;
        else out->controlword = KGU_CW_SHUTDOWN;
        if (st == 0x0027u) {
            c->phase = CTL_ON;
            c->fresh_cycle = a->cycles;
            finish(a, j, RES_OK);
        } else if (a->cycles - c->phase_cycle >
                   atomic_load_explicit(&a->enable_timeout_cycles, memory_order_relaxed)) {
            c->phase = CTL_OFF;
            out->controlword = KGU_CW_DISABLE;
            finish(a, j, RES_TIMEOUT);
        }
        break;
    }
    case CTL_ON: {
        arms_command cmd;
        out->controlword = KGU_CW_ENABLE;
        if (fetch_command(a, j, &cmd)) {
            c->fresh_cycle = a->cycles;
            c->timed_out = false;
            c->cur = cmd;
        }
        if (a->cycles - c->fresh_cycle > a->timeout_cycles && !c->timed_out) {
            c->timed_out = true;
            if (strcmp(a->cfg.timeout_action, "disable") == 0) {
                c->phase = CTL_OFF;
                out->controlword = KGU_CW_DISABLE;
            } else {
                c->cur.velocity = 0.0;
                c->cur.torque = 0.0;
            }
        }
        break;
    }
    case CTL_RESETTING: {
        uint64_t el = a->cycles - c->phase_cycle;
        if (el < 2) {
            out->controlword = KGU_CW_DISABLE;
        } else if (el < 2 + a->reset_pulse_cycles) {
            out->controlword = KGU_CW_FAULT_RESET;
        } else {
            out->controlword = KGU_CW_DISABLE;
            if (!fault && el > 2 + 2 * a->reset_pulse_cycles) {
                c->phase = CTL_OFF;
                finish(a, j, RES_OK);
            } else if (el > atomic_load_explicit(&a->enable_timeout_cycles, memory_order_relaxed)) {
                c->phase = CTL_OFF;
                finish(a, j, RES_FAULT);
            }
        }
        break;
    }
    default:
        out->controlword = KGU_CW_DISABLE;
        break;
    }

    active = c->phase == CTL_ENABLING || c->phase == CTL_ON;
    if (active) {
        /* 指令原样下发；kp、kd 为负或 NaN 时按 0（同参考 pack_cmd） */
        double pos = c->cur.position;
        double vel = c->cur.velocity;
        double tff = c->cur.torque;
        double kp = c->cur.kp > 0.0 ? c->cur.kp : 0.0;
        double kd = c->cur.kd > 0.0 ? c->cur.kd : 0.0;
        double d = (double)jc->direction;
        /* 软位置限位（同参考 over_pos）：实际位置越界后清零朝越界方向的速度和前馈，不截断位置 */
        if (s->position > jc->pos_limit_max) {
            if (vel > 0.0 || tff > 0.0) blocked = true;
            if (vel > 0.0) vel = 0.0;
            if (tff > 0.0) tff = 0.0;
        } else if (s->position < jc->pos_limit_min) {
            if (vel < 0.0 || tff < 0.0) blocked = true;
            if (vel < 0.0) vel = 0.0;
            if (tff < 0.0) tff = 0.0;
        }
        out->pos = (float)(d * (pos + atomic_load_explicit(&a->offset_rad[j], memory_order_relaxed)));
        out->vel = (float)(d * vel);
        out->tff = (float)(d * tff);
        out->kp = (float)kp;
        out->kd = (float)kd;
        s->cmd_position = pos;
        s->cmd_velocity = vel;
        s->cmd_torque = tff;
        s->cmd_kp = kp;
        s->cmd_kd = kd;
    } else {
        out->pos = r->slot_pos[k];
        out->vel = out->tff = out->kp = out->kd = 0.0f;
        s->cmd_position = s->position;
        s->cmd_velocity = s->cmd_torque = s->cmd_kp = s->cmd_kd = 0.0;
    }
    s->cmd_pos_blocked = blocked;
    s->cmd_timeout = c->phase == CTL_ON && c->timed_out;
    s->fault_count = c->fault_count;
}

static void cycle_tx(arms_t *a, uint64_t now)
{
    bool ready = atomic_load_explicit(&a->ready, memory_order_relaxed);
    uint64_t end;
    int i, k;

    for (i = 0; i < ARMS_ARM_COUNT; ++i) {
        arm_rt *r = &a->arm[i];
        if (!r->used) continue;
        for (k = 0; k < r->count; ++k) {
            kgu_command c;
            control_joint(a, r, k, ready, &c);
            kgu_encode(r->ops->outputs(r->bus, k), &c);
            memcpy(a->js[r->slot_joint[k]].rxpdo, r->ops->outputs(r->bus, k), KGU_OUT_BYTES);
        }
        r->auto_reset = false;
    }
    for (i = 0; i < ARMS_ARM_COUNT; ++i) {
        arm_rt *r = &a->arm[i];
        uint64_t t0, t1;
        if (!r->used) continue;
        t0 = mono_ns();
        if (r->ops->send(r->bus, now) != 0) r->send_errors++;
        t1 = mono_ns();
        if ((double)(t1 - t0) / 1000.0 > r->send_us_max) r->send_us_max = (double)(t1 - t0) / 1000.0;
    }
    a->cycles++;
    end = mono_ns();
    if (end > a->cycle_start_ns && (double)(end - a->cycle_start_ns) > a->work_max)
        a->work_max = (double)(end - a->cycle_start_ns);
    a->last_tx_end_ns = end;
    if (a->trace_count && end > a->cycle_start_ns) {
        uint64_t w = end - a->cycle_start_ns;
        a->trace[(a->trace_count - 1) % TRACE_LEN].work_ns = w > UINT32_MAX ? UINT32_MAX : (uint32_t)w;
    }
    publish(a);
}

static void *rt_main(void *arg)
{
    arms_t *a = arg;
    uint64_t next_ns = mono_ns() + a->period_ns;

    while (!atomic_load_explicit(&a->stop, memory_order_relaxed)) {
        struct timespec next;
        uint64_t start, late;

        next.tv_sec = (time_t)(next_ns / 1000000000ull);
        next.tv_nsec = (long)(next_ns % 1000000000ull);
        while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL) == EINTR) {}
        start = mono_ns();
        late = start > next_ns ? start - next_ns : 0;

        a->wake_late_ns = late;
        cycle_rx(a, start);
        if ((double)late > a->wake_late_max) a->wake_late_max = (double)late;
        cycle_tx(a, mono_ns());

        /*
         * 下一次接收与本次发送至少间隔半个周期，保证本次发出的帧已绕链返回。
         * 周期迟到后若紧接着接收，帧尚未返回，两臂会同时读到 WKC 为 0。
         */
        next_ns += a->period_ns;
        start = mono_ns();
        while (next_ns < start + a->period_ns / 2) {
            next_ns += a->period_ns;
            a->skipped++;
        }
    }
    return NULL;
}

static arms_status start_rt(arms_t *a)
{
    pthread_attr_t attr;
    struct sched_param sp;
    int ret;
    bool realtime = true;

    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = a->cfg.rt_priority;
    pthread_attr_setschedparam(&attr, &sp);
    if (a->rt_cpu >= 0) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(a->rt_cpu, &set);
        pthread_attr_setaffinity_np(&attr, sizeof(set), &set);
    }
    ret = pthread_create(&a->thread, &attr, rt_main, a);
    if (ret == EPERM && a->opt.backend == ARMS_BACKEND_MOCK) {
        pthread_attr_destroy(&attr);
        realtime = false;
        pthread_attr_init(&attr);
        ret = pthread_create(&a->thread, &attr, rt_main, a);
    }
    pthread_attr_destroy(&attr);
    if (ret == EPERM) {
        set_error("无法创建 SCHED_FIFO 实时线程：权限不足，需要 root");
        return ARMS_E_SYSTEM;
    }
    if (ret != 0) {
        set_error("创建实时线程失败: %s", strerror(ret));
        return ARMS_E_SYSTEM;
    }
    pthread_setname_np(a->thread, "arms_rt");
    a->thread_started = true;
    if (realtime) {
        if (a->rt_cpu >= 0)
            logf_(a, "实时线程: SCHED_FIFO %d, CPU %d, %u Hz", a->cfg.rt_priority, a->rt_cpu, a->cycle_hz);
        else
            logf_(a, "实时线程: SCHED_FIFO %d, 未绑定 CPU, %u Hz", a->cfg.rt_priority, a->cycle_hz);
    } else {
        logf_(a, "警告: 非 root 运行，模拟后端以普通线程运行，时序统计仅供参考");
    }
    return ARMS_OK;
}

static void close_buses(arms_t *a)
{
    int i;
    for (i = 0; i < ARMS_ARM_COUNT; ++i) {
        if (a->arm[i].bus) a->arm[i].ops->close(a->arm[i].bus);
        a->arm[i].bus = NULL;
    }
}

static void stop_and_free(arms_t *a)
{
    if (a->thread_started) {
        /* 先失能并保持若干周期，让驱动器在通信停止前进入失能 */
        post(a, a->joints, REQ_DISABLE);
        usleep(50000);
        atomic_store(&a->stop, true);
        pthread_join(a->thread, NULL);
    }
    close_buses(a);
    pthread_mutex_destroy(&a->cmd_lock);
    free(a);
}

/* ---- 对外接口 ---- */

arms_status arms_open(const arms_options *options, arms_t **out)
{
    arms_t *a;
    char path[ARMS_PATH_LEN], err[512];
    arms_status st = ARMS_OK;
    int i, j;
    bool check;

    last_error[0] = '\0';
    if (!options || !out) {
        set_error("arms_open: 参数为空");
        return ARMS_E_ARG;
    }
    *out = NULL;
    a = calloc(1, sizeof(*a));
    if (!a) {
        set_error("内存不足");
        return ARMS_E_SYSTEM;
    }
    a->open_ns = mono_ns();
    a->opt = *options;

    if (options->config_path) {
        snprintf(path, sizeof(path), "%s", options->config_path);
    } else if (!arms_config_default_path(path, sizeof(path))) {
        set_error("找不到配置文件（依次查找 $ARMS_CONFIG、/etc/bxi/arms.json、./config/arms.json）");
        free(a);
        return ARMS_E_CONFIG;
    }
    if (!arms_config_load(path, &a->cfg, err, sizeof(err))) {
        set_error("%s", err);
        free(a);
        return ARMS_E_CONFIG;
    }
    a->cycle_hz = options->cycle_hz ? options->cycle_hz : a->cfg.cycle_hz;
    if (!arms_cycle_hz_supported(a->cycle_hz)) {
        set_error("周期 %u Hz 不受支持，可选 2000/1000/500/250/200/125", a->cycle_hz);
        free(a);
        return ARMS_E_ARG;
    }
    a->period_ns = 1000000000ull / a->cycle_hz;
    a->poll_div = a->cycle_hz / STATE_POLL_HZ ? a->cycle_hz / STATE_POLL_HZ : 1u;
    a->ready_cycles = (uint64_t)READY_STABLE_MS * a->cycle_hz / 1000u;
    a->timeout_cycles = (uint64_t)a->cfg.command_timeout_ms * a->cycle_hz / 1000u;
    a->reset_pulse_cycles = (uint64_t)RESET_PULSE_MS * a->cycle_hz / 1000u;
    a->recover_cycles = (uint64_t)RECOVER_STABLE_MS * a->cycle_hz / 1000u;
    a->overspeed_cycles = (uint64_t)a->cfg.overspeed_ms * a->cycle_hz / 1000u;
    a->overtorque_cycles = (uint64_t)a->cfg.overtorque_ms * a->cycle_hz / 1000u;
    if (a->overspeed_cycles == 0) a->overspeed_cycles = 1;
    if (a->overtorque_cycles == 0) a->overtorque_cycles = 1;
    a->protect.joint = -1;
    a->diag_div = a->cycle_hz >= 100u ? a->cycle_hz / 100u : 1u; /* 驱动器监控每 10 ms 推进一步 */
    for (int i = 0; i < ARMS_ARM_COUNT; ++i)
        for (int k = 0; k < BUS_MAX_SLAVES; ++k) {
            a->arm[i].temp[k] = NAN;
            a->arm[i].drive_loss[k] = -1;
        }
    atomic_store(&a->enable_timeout_cycles, DEFAULT_WAIT_MS * a->cycle_hz / 1000u);
    a->mode = options->mode != ARMS_MODE_CONFIG ? options->mode : a->cfg.mode;
    a->rt_cpu = options->rt_cpu >= -1 ? options->rt_cpu : a->cfg.rt_cpu;
    if (a->rt_cpu >= sysconf(_SC_NPROCESSORS_CONF)) {
        set_error("CPU %d 不存在", a->rt_cpu);
        free(a);
        return ARMS_E_ARG;
    }
#ifdef ARMS_HAVE_ETHERLAB
    a->ops = options->backend == ARMS_BACKEND_MOCK ? &bus_mock_ops : &bus_etherlab_ops;
#else
    a->ops = options->backend == ARMS_BACKEND_MOCK ? &bus_mock_ops : NULL;
#endif
    if (!a->ops) {
        set_error("未编译 EtherLab 后端，只能使用模拟后端");
        free(a);
        return ARMS_E_UNSUPPORTED;
    }

    if (options->ignore_mask > 0x7FFFFFFFll) {
        set_error("屏蔽掩码 0x%llx 超出 31 位", (unsigned long long)options->ignore_mask);
        free(a);
        return ARMS_E_ARG;
    }
    a->ignored = (options->ignore_mask >= 0 ? (arms_mask)options->ignore_mask : a->cfg.ignore_mask) & ARMS_MASK_ALL;
    check = options->check_motors >= 0 ? options->check_motors != 0 : a->cfg.check_motors;
    a->power_off_action = strcmp(a->cfg.offline_action, "power_off") == 0;
    resolve_event_log(a, options->event_log);
    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
        atomic_init(&a->offset_rad[j], a->cfg.joint[j].offset_rad);
        if (!a->cfg.joint[j].present) continue;
        if (options->joints && !(options->joints & ARMS_MASK_JOINT(j))) continue;
        if (a->ignored & ARMS_MASK_JOINT(j)) continue;
        a->joints |= ARMS_MASK_JOINT(j);
    }
    if (!a->joints) {
        set_error("所选关节在配置中都不存在或全部被屏蔽");
        free(a);
        return ARMS_E_ARG;
    }
    for (j = 0; j <= ARMS_JOINT_LAST; ++j) {
        a->js[j].joint = j;
        a->js[j].arm = -1;
        a->js[j].slave = -1;
        a->js[j].mode = ARMS_JS_ABSENT;
        a->js[j].temperature = NAN;
        a->js[j].drive_frame_loss = -1;
    }
    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
        const arms_joint_config *jc = &a->cfg.joint[j];
        arm_rt *r;
        if (!(a->joints & ARMS_MASK_JOINT(j))) continue;
        r = &a->arm[jc->arm];
        r->used = true;
        r->ops = a->ops;
        r->slot_joint[r->count] = j;
        r->slot_slave[r->count] = jc->slave;
        r->count++;
        a->js[j].name = jc->name;
        a->js[j].arm = jc->arm;
        a->js[j].slave = jc->slave;
        a->js[j].mode = ARMS_JS_OFFLINE;
    }

    if (options->backend != ARMS_BACKEND_MOCK && mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        set_error("mlockall 失败: %s，需要 root", strerror(errno));
        free(a);
        return ARMS_E_SYSTEM;
    }

    if (pthread_mutex_init(&a->cmd_lock, NULL) != 0) {
        set_error("无法初始化指令互斥锁");
        free(a);
        return ARMS_E_SYSTEM;
    }
    logf_(a, "配置: %s", path);
    logf_(a, "启动检查: %s, 屏蔽掩码 0x%08x, 离线处理: %s", check ? "开启" : "关闭", a->ignored,
          a->power_off_action ? "下电保护" : "失能并自动重连");
    for (i = 0; i < ARMS_ARM_COUNT; ++i) {
        arm_rt *r = &a->arm[i];
        const arms_arm_config *ac = &a->cfg.arm[i];
        arms_arm_stats *as = &a->snap.stats.arm[i];
        bus_open_params p;
        bus_slave_info infos[BUS_MAX_SLAVES];
        int found = 0, k;
        if (!r->used) continue;
        memset(&p, 0, sizeof(p));
        p.interface = ac->interface;
        p.mac = ac->mac;
        p.master = ac->master;
        p.cycle_ns = (uint32_t)a->period_ns;
        p.sync0_shift_ns = 0;
        p.vendor_id = a->cfg.vendor_id;
        p.product_code = a->cfg.product_code;
        p.log = bus_log_adapter;
        p.log_user = a;
        if (r->ops->open(&r->bus, &p, err, sizeof(err)) != 0) {
            set_error("%s 臂: %s", arms_arm_name(i), err);
            st = ARMS_E_MASTER;
            goto fail;
        }
        logf_(a, "%s 臂: 网口 %s, 主站 %d, 等待总线稳定", arms_arm_name(i), ac->interface,
              r->ops->master_index(r->bus));
        {
            /* 预期从站数：配置中该臂用到的最大链上位置 + 1 */
            int expected = 0, scan_rc;
            uint64_t t0 = mono_ns();
            for (k = 0; k < r->count; ++k)
                if (r->slot_slave[k] + 1 > expected) expected = r->slot_slave[k] + 1;
            scan_rc = r->ops->scan(r->bus, infos, BUS_MAX_SLAVES, expected, &found,
                                   options->scan_timeout_ms ? options->scan_timeout_ms : DEFAULT_SCAN_TIMEOUT_MS, err,
                                   sizeof(err));
            if (scan_rc == 0) {
                logf_(a, "%s 臂: 扫描完成，发现 %d 个从站（预期 %d），用时 %.2f s", arms_arm_name(i), found, expected,
                      (double)(mono_ns() - t0) / 1e9);
            } else if (scan_rc == BUS_SCAN_MASTER) {
                set_error("%s 臂: %s", arms_arm_name(i), err);
                st = ARMS_E_MASTER;
                goto fail;
            } else if (check || scan_rc != BUS_SCAN_EMPTY) {
                set_error("%s 臂: %s", arms_arm_name(i), err);
                st = scan_rc == BUS_SCAN_LINK ? ARMS_E_LINK : ARMS_E_TOPOLOGY;
                goto fail;
            } else {
                logf_(a, "警告: %s 臂: %s", arms_arm_name(i), err);
                found = 0;
            }
        }
        {
            /* 启动检查：链上位置存在且型号一致才视为在线 */
            arms_mask miss = 0;
            char list[256];
            size_t len = 0;
            int n = 0;
            list[0] = '\0';
            for (k = 0; k < r->count; ++k) {
                int pos = r->slot_slave[k];
                int jn = r->slot_joint[k];
                if (pos < found && infos[pos].vendor_id == a->cfg.vendor_id &&
                    infos[pos].product_code == a->cfg.product_code) {
                    r->slot_joint[n] = jn;
                    r->slot_slave[n] = pos;
                    n++;
                    continue;
                }
                miss |= ARMS_MASK_JOINT(jn);
                if (len < sizeof(list))
                    len += (size_t)snprintf(list + len, sizeof(list) - len, "%s%d(%s, slave %d%s)", len ? " " : "",
                                            jn, a->cfg.joint[jn].name, pos, pos < found ? ", 型号不符" : "");
            }
            if (miss) {
                if (check) {
                    set_error("启动检查失败: %s 臂总线上发现 %d 个从站，以下关节不在线: %s", arms_arm_name(i), found,
                              list);
                    st = ARMS_E_TOPOLOGY;
                    goto fail;
                }
                logf_(a, "警告: %s 臂以下关节不在线，本次不使用: %s", arms_arm_name(i), list);
                r->count = n;
                a->missing |= miss;
                a->joints &= ~miss;
                for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j)
                    if (miss & ARMS_MASK_JOINT(j)) a->js[j].mode = ARMS_JS_ABSENT;
            }
        }
        if (r->count == 0) {
            r->ops->close(r->bus);
            r->bus = NULL;
            r->used = false;
            continue;
        }
        r->slaves_found = found;
        as->present = true;
        as->slaves_found = found;
        as->slaves_expected = r->count;
        as->master_index = r->ops->master_index(r->bus);
        snprintf(as->interface, sizeof(as->interface), "%s", ac->interface);
        logf_(a, "%s 臂: 发现 %d 个从站, 使用 %d 个", arms_arm_name(i), found, r->count);
        if (r->ops->configure(r->bus, r->slot_slave, r->count, err, sizeof(err)) != 0) {
            set_error("%s 臂: %s", arms_arm_name(i), err);
            st = ARMS_E_MASTER;
            goto fail;
        }
        r->wkc_expected = r->ops->expected_wkc(r->bus);
    }
    if (!a->joints) {
        set_error("没有在线的关节");
        st = ARMS_E_TOPOLOGY;
        goto fail;
    }
    for (i = 0; i < ARMS_ARM_COUNT; ++i) {
        arm_rt *r = &a->arm[i];
        if (!r->used) continue;
        if (r->ops->activate(r->bus, err, sizeof(err)) != 0) {
            set_error("%s 臂: %s", arms_arm_name(i), err);
            st = ARMS_E_MASTER;
            goto fail;
        }
    }
    publish(a); /* 外部周期尚未运行时也能读取已配置的关节、主站和初始状态。 */
    a->activated_ns = mono_ns();

    if (a->mode == ARMS_MODE_EXTERNAL) {
        logf_(a, "外部驱动模式: 周期 %u Hz, 等待调用方周期调用 arms_rx/arms_tx", a->cycle_hz);
        *out = a;
        return ARMS_OK;
    }
    st = start_rt(a);
    if (st != ARMS_OK) goto fail;
    st = arms_wait_ready(a, 0);
    if (st != ARMS_OK) goto fail;
    *out = a;
    return ARMS_OK;

fail:
    stop_and_free(a);
    return st;
}

void arms_close(arms_t *a)
{
    if (a) {
        arms_mask abnormal = atomic_load(&a->brake_released) | atomic_load(&a->brake_held) | ~atomic_load(&a->brake_known);
        for (int j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; j++)
            if (a->brake_touched & abnormal & ARMS_MASK_JOINT(j)) {
                if (arms_set_brake(a, j, ARMS_BRAKE_NORMAL, true) != ARMS_OK)
                    logf_(a, "警告：关节 %d 未能恢复正常抱闸控制，请保持机械支撑", j);
            }
        stop_and_free(a);
    }
}

arms_status arms_rx(arms_t *a, uint64_t now_ns)
{
    if (!a) return ARMS_E_ARG;
    if (a->mode != ARMS_MODE_EXTERNAL) return ARMS_E_STATE;
    cycle_rx(a, now_ns);
    return ARMS_OK;
}

arms_status arms_tx(arms_t *a, uint64_t now_ns)
{
    if (!a) return ARMS_E_ARG;
    if (a->mode != ARMS_MODE_EXTERNAL) return ARMS_E_STATE;
    cycle_tx(a, now_ns);
    return ARMS_OK;
}

bool arms_ready(const arms_t *a)
{
    return a && atomic_load_explicit(&((arms_t *)a)->ready, memory_order_acquire);
}

arms_status arms_wait_ready(arms_t *a, unsigned int timeout_ms)
{
    uint64_t deadline, next_log;
    if (!a) return ARMS_E_ARG;
    deadline = mono_ns() + (uint64_t)(timeout_ms ? timeout_ms : a->cfg.op_timeout_ms) * 1000000ull;
    next_log = mono_ns() + 1000000000ull;
    while (!arms_ready(a)) {
        shared_snapshot s;
        uint64_t now = mono_ns();
        if (now >= deadline || now >= next_log) read_snapshot(a, &s);
        if (now >= deadline) {
            set_error("预热超时 (%u ms): OP 从站 左 %d/%d, 右 %d/%d",
                      timeout_ms ? timeout_ms : a->cfg.op_timeout_ms, s.stats.arm[0].slaves_op, a->arm[0].count,
                      s.stats.arm[1].slaves_op, a->arm[1].count);
            return ARMS_E_TIMEOUT;
        }
        if (now >= next_log) {
            logf_(a, "预热中: OP 从站 左 %d/%d, 右 %d/%d", s.stats.arm[0].slaves_op, a->arm[0].count,
                  s.stats.arm[1].slaves_op, a->arm[1].count);
            next_log = now + 1000000000ull;
        }
        usleep(10000);
    }
    {
        uint64_t ready = atomic_load(&a->ready_ns);
        double warm = ready > a->activated_ns ? (double)(ready - a->activated_ns) / 1e9 : 0.0;
        double total = ready > a->open_ns ? (double)(ready - a->open_ns) / 1e9 : 0.0;
        logf_(a, "预热完成: %d 个关节处于 OP, 伺服失能；预热用时 %.2f s（主站激活到稳定 OP），启动总用时 %.2f s",
              a->arm[0].count + a->arm[1].count, warm, total);
    }
    return ARMS_OK;
}

uint64_t arms_period_ns(const arms_t *a) { return a ? a->period_ns : 0; }

/* 失能关节按抱闸模式显示为 released / held */
static void apply_brake_mode(arms_t *a, arms_joint_state *s)
{
    arms_mask bit = ARMS_MASK_JOINT(s->joint);
    if (s->mode != ARMS_JS_LOCKED || !(atomic_load(&a->brake_known) & bit)) return;
    if (atomic_load(&a->brake_released) & bit) s->mode = ARMS_JS_RELEASED;
    else if (atomic_load(&a->brake_held) & bit) s->mode = ARMS_JS_HELD;
}

arms_status arms_read(arms_t *a, arms_joint_state *states, size_t count, size_t *written)
{
    shared_snapshot s;
    size_t n = 0;
    int j;
    if (!a || !states) return ARMS_E_ARG;
    read_snapshot(a, &s);
    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST && n < count; ++j)
        if (a->joints & ARMS_MASK_JOINT(j)) {
            states[n] = s.s[j];
            apply_brake_mode(a, &states[n]);
            n++;
        }
    if (written) *written = n;
    return ARMS_OK;
}

arms_status arms_read_joint(arms_t *a, int joint, arms_joint_state *state)
{
    shared_snapshot s;
    if (!a || !state || joint < ARMS_JOINT_FIRST || joint > ARMS_JOINT_LAST) return ARMS_E_ARG;
    read_snapshot(a, &s);
    *state = s.s[joint];
    apply_brake_mode(a, state);
    return ARMS_OK;
}

arms_status arms_get_stats(arms_t *a, arms_bus_stats *stats)
{
    shared_snapshot s;
    if (!a || !stats) return ARMS_E_ARG;
    read_snapshot(a, &s);
    *stats = s.stats;
    stats->cycle_hz = a->cycle_hz;
    return ARMS_OK;
}

void arms_reset_stats(arms_t *a)
{
    if (a) atomic_store(&a->reset_stats, true);
}

unsigned int arms_cycle_hz(const arms_t *a) { return a ? a->cycle_hz : 0; }
arms_run_mode arms_mode(const arms_t *a) { return a ? a->mode : ARMS_MODE_CONFIG; }
int arms_rt_cpu(const arms_t *a) { return a ? a->rt_cpu : -1; }
const char *arms_backend_name(const arms_t *a) { return a && a->ops ? a->ops->name : "?"; }
const char *arms_config_path(const arms_t *a) { return a ? a->cfg.path : ""; }
const char *arms_event_log_path(const arms_t *a) { return a ? a->event_log : ""; }

static const char *event_name(int kind)
{
    switch (kind) {
    case EVT_WKC: return "WKC 异常";
    case EVT_FAIL: return "通信失败";
    case EVT_RECOVER: return "通信恢复";
    case EVT_LATE: return "周期延迟";
    case EVT_TRIP: return "离线保护触发";
    case EVT_ESC: return "线路错误计数增加";
    case EVT_ESC_PROC: return "从站拒绝的帧（汇总）";
    case EVT_DRIVE_LOSS: return "驱动器丢帧";
    case EVT_TEMP_WARN: return "温度告警";
    case EVT_OVERTEMP: return "过温保护";
    default: return "?";
    }
}

/* 输出各从站的温度、驱动器丢帧次数和非零的 ESC 错误计数 */
static void write_slave_table(FILE *fp, const event_record *e)
{
    for (int i = 0; i < ARMS_ARM_COUNT; ++i) {
        if (!e->arm[i].used) continue;
        fprintf(fp, "  %-5s 温度℃/驱动器丢帧:", arms_arm_name(i));
        for (int s = 0; s < e->arm[i].count; ++s) {
            if (isnan(e->arm[i].temp[s])) fprintf(fp, "  -");
            else fprintf(fp, "  %.0f", e->arm[i].temp[s]);
            if (e->arm[i].drive_loss[s] < 0) fprintf(fp, "/-");
            else fprintf(fp, "/%d", e->arm[i].drive_loss[s]);
        }
        fputc('\n', fp);
        for (int s = 0; s < e->arm[i].count; ++s) {
            bool any = false;
            if (!e->arm[i].esc_valid[s]) continue;
            for (unsigned int b = 0; b < ESC_ERR_BYTES; ++b) {
                if (!esc_names[b] || !e->arm[i].esc[s][b]) continue;
                if (!any) fprintf(fp, "    关节 %d（链上位置 %d）ESC 错误计数:", e->arm[i].joint[s], s);
                fprintf(fp, " %s %u", esc_names[b], e->arm[i].esc[s][b]);
                any = true;
            }
            if (any) fputc('\n', fp);
        }
    }
}

static void write_detail(FILE *fp, const event_record *e)
{
    const event_detail *d = &e->detail;
    if (!e->has_detail) return;
    switch (e->kind) {
    case EVT_ESC:
        fprintf(fp, "  关节 %d（链上位置 %d）计数变化:", d->joint, d->slave);
        for (unsigned int b = 0; b < ESC_ERR_BYTES; ++b)
            if (esc_names[b] && d->after[b] != d->before[b])
                fprintf(fp, " %s %u→%u", esc_names[b], d->before[b], d->after[b]);
        fprintf(fp, "\n  线路类计数（无效帧、物理层错误、链路丢失）增加，指向网线、接头或从站网口。\n"
                    "  端口 0 接主站一侧，端口 1 接下一个从站；计数最先增加的位置通常最靠近故障点\n");
        break;
    case EVT_ESC_PROC:
        fprintf(fp,
                "  过去 %.0f 秒内有 %.0f 个帧被从站处理单元拒绝，最先拒绝的是关节 %d（链上位置 %d）；线路类计数未变。\n"
                "  帧在电气上完好，不是网线问题。通常是主机发往 EtherCAT 网口的 DHCP、mDNS、IPv6 等普通报文，\n"
                "  rt_setup.sh 在程序运行期间会阻止这类报文；用 rt_setup.sh status 查看网口是否仍有 IP 或由 "
                "NetworkManager 管理。\n",
                d->limit, d->value, d->joint, d->slave);
        break;
    case EVT_DRIVE_LOSS:
        fprintf(fp, "  关节 %d（链上位置 %d）驱动器自报 EtherCAT 丢帧次数 %.0f → %.0f\n", d->joint, d->slave, d->limit,
                d->value);
        break;
    case EVT_TEMP_WARN:
    case EVT_OVERTEMP:
        fprintf(fp, "  关节 %d（链上位置 %d）执行器温度 %.0f ℃，阈值 %.0f ℃%s\n", d->joint, d->slave, d->value, d->limit,
                e->kind == EVT_OVERTEMP ? "；全部关节已失能并锁定，降温后 reset" : "");
        break;
    default:
        break;
    }
}

/* 本地时间 YYYY-mm-dd HH:MM:SS，micro 为 true 时附加微秒 */
static void format_real(int64_t ns, bool micro, char *buf, size_t size)
{
    time_t sec = (time_t)(ns / 1000000000LL);
    struct tm tm;
    char head[32];
    localtime_r(&sec, &tm);
    strftime(head, sizeof(head), "%Y-%m-%d %H:%M:%S", &tm);
    if (micro) snprintf(buf, size, "%s.%06ld", head, (long)(ns % 1000000000LL / 1000));
    else snprintf(buf, size, "%s", head);
}

static FILE *open_event_log(arms_t *a)
{
    char dir[PATH_MAX];
    const char *slash;
    FILE *fp;
    for (int attempt = 0; attempt < 2; ++attempt) {
        slash = strrchr(a->event_log, '/');
        if (slash) {
            snprintf(dir, sizeof(dir), "%.*s", (int)(slash - a->event_log), a->event_log);
            (void)mkdir(dir, 0755); /* 已存在或无法创建时由 fopen 报告 */
        }
        fp = fopen(a->event_log, "a");
        if (fp) return fp;
        if (strcmp(a->event_log, "/tmp/arms_comm_events.log") == 0) break;
        logf_(a, "通信事件日志无法写入 %s（%s），改写到 /tmp/arms_comm_events.log", a->event_log, strerror(errno));
        snprintf(a->event_log, sizeof(a->event_log), "/tmp/arms_comm_events.log");
    }
    set_error("无法写入通信事件日志 %s: %s", a->event_log, strerror(errno));
    return NULL;
}

static void write_event(arms_t *a, FILE *fp, const event_record *e)
{
    char when[48], since[48];
    bool timing = e->kind != EVT_TEMP_WARN && e->kind != EVT_OVERTEMP && e->kind != EVT_ESC_PROC;
    unsigned int shown = !timing ? 0
                         : (e->kind == EVT_FAIL || e->kind == EVT_TRIP) ? e->n
                                                                         : (e->n < 16 ? e->n : 16);
    format_real(e->real_ns, true, when, sizeof(when));
    format_real(e->real_ns - 2000000000LL, false, since, sizeof(since));
    fprintf(fp, "==== %s  %s", when, event_name(e->kind));
    if (e->which_arm >= 0) fprintf(fp, "  %s 臂", arms_arm_name(e->which_arm));
    fprintf(fp, "  周期 %llu\n", (unsigned long long)e->cycle);
    if (e->suppressed) fprintf(fp, "  此前 1 秒内另有 %u 次同类事件未单独记录\n", e->suppressed);
    fprintf(fp, "  跳过周期 %llu  迟到周期(>1.5T) %llu  周期 %u Hz\n", (unsigned long long)e->skipped,
            (unsigned long long)e->overruns, a->cycle_hz);
    for (int i = 0; i < ARMS_ARM_COUNT; ++i) {
        if (!e->arm[i].used) continue;
        fprintf(fp, "  %-5s 链路 %s  通信%s  WKC %u/%u  连续异常 %u  累计异常 %llu  AL", arms_arm_name(i),
                e->arm[i].link_up ? "UP" : "DOWN", e->arm[i].comm_failed ? "失败" : "正常", e->arm[i].wkc,
                e->arm[i].wkc_expected, e->arm[i].consecutive, (unsigned long long)e->arm[i].wkc_bad);
        for (int k = 0; k < e->arm[i].count; ++k) fprintf(fp, " %02x", e->arm[i].al[k]);
        fputc('\n', fp);
    }
    write_detail(fp, e);
    write_slave_table(fp, e);
    if (shown) {
        fprintf(fp, "  最近 %u 个周期（旧 → 新；时刻相对本事件）:\n", shown);
        fprintf(fp, "     相对ms        周期    间隔us  唤醒延迟us  发送→接收us   处理us  WKC左 WKC右\n");
        for (unsigned int k = e->n - shown; k < e->n; ++k) {
            const trace_entry *t = &e->trace[k];
            fprintf(fp, "  %9.3f  %10llu  %8.1f  %10.1f  %11.1f  %7.1f  %5u %5u\n",
                    ((double)t->start_ns - (double)e->mono_ns) / 1e6, (unsigned long long)t->cycle,
                    t->period_ns / 1e3, t->late_ns / 1e3, t->gap_ns / 1e3, t->work_ns / 1e3, t->wkc[0], t->wkc[1]);
        }
    }
    if (timing) fprintf(fp, "  对照内核日志: journalctl -k -o short-precise --since \"%s\"\n", since);
    fputc('\n', fp);
}

typedef struct {
    int arm, slave, joint;
    uint8_t msg[EMERGENCY_MSG_BYTES];
} emergency_msg;

/* CoE 紧急报文：错误码（2 字节小端）、错误寄存器（1 字节）、厂家数据（5 字节） */
static void write_emergency(arms_t *a, FILE *fp, const emergency_msg *m)
{
    struct timespec rt;
    char when[48];
    arms_joint_state s;
    clock_gettime(CLOCK_REALTIME, &rt);
    format_real((int64_t)rt.tv_sec * 1000000000LL + rt.tv_nsec, true, when, sizeof(when));
    fprintf(fp, "==== %s  驱动器紧急报文  %s 臂\n", when, arms_arm_name(m->arm));
    fprintf(fp, "  关节 %d（链上位置 %d）错误码 0x%04x  错误寄存器 0x%02x  厂家数据 %02x %02x %02x %02x %02x\n",
            m->joint, m->slave, (unsigned)(m->msg[0] | (m->msg[1] << 8)), m->msg[2], m->msg[3], m->msg[4], m->msg[5],
            m->msg[6], m->msg[7]);
    if (arms_read_joint(a, m->joint, &s) == ARMS_OK)
        fprintf(fp, "  取出时状态 %s  状态字 0x%04x  错误码(0x603F) 0x%04x\n", arms_joint_mode_name(s.mode),
                s.status_word, s.error_code);
    fputc('\n', fp);
}

int arms_log_events(arms_t *a)
{
    uint64_t total, i;
    int written = 0, ne = 0;
    emergency_msg emcy[EMERGENCY_BATCH];
    FILE *fp;
    if (!a) return -1;
    for (int arm = 0; arm < ARMS_ARM_COUNT; ++arm) {
        arm_rt *r = &a->arm[arm];
        if (!r->used || !r->bus) continue;
        for (int k = 0; k < r->count && ne < EMERGENCY_BATCH; ++k) {
            while (ne < EMERGENCY_BATCH && r->ops->emerg_pop(r->bus, k, emcy[ne].msg) == 1) {
                emcy[ne].arm = arm;
                emcy[ne].slave = r->slot_slave[k];
                emcy[ne].joint = r->slot_joint[k];
                ne++;
            }
        }
    }
    total = atomic_load_explicit(&a->evt_count, memory_order_acquire);
    if (total == a->evt_read && ne == 0) return 0;
    fp = open_event_log(a);
    if (!fp) return -1;
    if (!a->evt_header_done) {
        struct timespec rt;
        char when[48];
        clock_gettime(CLOCK_REALTIME, &rt);
        format_real((int64_t)rt.tv_sec * 1000000000LL + rt.tv_nsec, false, when, sizeof(when));
        fprintf(fp, "#### 会话 %s  arms_driver %s  后端 %s  配置 %s  WKC 容忍 %u 次  离线处理 %s\n\n", when,
                arms_version(), arms_backend_name(a), a->cfg.path, a->cfg.wkc_tolerance, a->cfg.offline_action);
        a->evt_header_done = true;
    }
    if (total - a->evt_read > EVENT_RING) {
        fprintf(fp, "==== %llu 条通信事件因产生过快被覆盖，未写入\n\n",
                (unsigned long long)(total - a->evt_read - EVENT_RING));
        a->evt_read = total - EVENT_RING;
    }
    for (i = a->evt_read; i < total; ++i) {
        event_buf b;
        load_words(&b, a->evt_words[i % EVENT_RING], WORDS_UP(event_record));
        atomic_thread_fence(memory_order_acquire);
        if (atomic_load_explicit(&a->evt_begin, memory_order_relaxed) > i + EVENT_RING) {
            fprintf(fp, "==== 1 条通信事件在读取时被新事件覆盖，未写入\n\n");
            continue;
        }
        write_event(a, fp, &b.r);
        written++;
    }
    a->evt_read = total;
    for (int m = 0; m < ne; ++m) write_emergency(a, fp, &emcy[m]);
    written += ne;
    fclose(fp);
    return written;
}

void arms_clear_esc_counters(arms_t *a)
{
    if (a) atomic_store_explicit(&a->esc_clear_all, true, memory_order_release);
}
arms_mask arms_opened_joints(const arms_t *a) { return a ? a->joints : 0; }
arms_mask arms_ignored_joints(const arms_t *a) { return a ? a->ignored : 0; }
arms_mask arms_missing_joints(const arms_t *a) { return a ? a->missing : 0; }
bool arms_startup_check_enabled(const arms_t *a)
{
    return a && (a->opt.check_motors >= 0 ? a->opt.check_motors != 0 : a->cfg.check_motors);
}
bool arms_offline_power_off(const arms_t *a) { return a && a->power_off_action; }

static size_t list_joints(const arms_t *a, arms_mask m, char *buf, size_t size)
{
    size_t len = 0;
    int j;
    buf[0] = '\0';
    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST && len < size; ++j)
        if (m & ARMS_MASK_JOINT(j))
            len += (size_t)snprintf(buf + len, size - len, "%s%d(%s)", len ? " " : "", j, a->cfg.joint[j].name);
    return len;
}

arms_status arms_check_online(arms_t *a, arms_mask *offline_out)
{
    shared_snapshot s;
    arms_mask off = 0;
    char list[512];
    int j;
    if (!a) return ARMS_E_ARG;
    read_snapshot(a, &s);
    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
        const arms_arm_stats *as;
        if (!(a->joints & ARMS_MASK_JOINT(j))) continue;
        as = &s.stats.arm[a->cfg.joint[j].arm];
        if ((s.s[j].al_state & 0x0F) != 0x08 || as->comm_failed || as->wkc_last != as->wkc_expected)
            off |= ARMS_MASK_JOINT(j);
    }
    if (offline_out) *offline_out = off;
    if (!off) return ARMS_OK;
    list_joints(a, off, list, sizeof(list));
    set_error("以下关节不在线: %s", list);
    return ARMS_E_STATE;
}

arms_status arms_clear_trip(arms_t *a)
{
    shared_snapshot s;
    uint64_t until;
    arms_status st;
    if (!a) return ARMS_E_ARG;
    st = arms_check_online(a, NULL);
    if (st != ARMS_OK) return st;
    atomic_store(&a->clear_trip, true);
    until = mono_ns() + 200000000ull;
    do {
        usleep(2000);
        read_snapshot(a, &s);
    } while (s.stats.tripped && mono_ns() < until);
    if (s.stats.tripped) {
        set_error("离线保护未能清除：通信仍不稳定");
        return ARMS_E_STATE;
    }
    return ARMS_OK;
}

const char *arms_joint_name(const arms_t *a, int joint)
{
    if (joint < 0 || joint > 30) return NULL;
    if (a && joint >= ARMS_JOINT_FIRST && joint <= ARMS_JOINT_LAST && a->cfg.joint[joint].present)
        return a->cfg.joint[joint].name;
    return arms_standard_joint_names[joint];
}

int arms_joint_by_name(const arms_t *a, const char *name)
{
    int j;
    size_t n;
    (void)a;
    if (!name) return -1;
    n = strlen(name);
    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
        const char *s = arms_standard_joint_names[j];
        if (strcmp(s, name) == 0) return j;
        if (strncmp(s, name, n) == 0 && strcmp(s + n, "_joint") == 0) return j;
    }
    return -1;
}

arms_status arms_get_limits(const arms_t *a, int joint, arms_joint_limits *l)
{
    const arms_joint_config *jc;
    if (!a || !l || joint < ARMS_JOINT_FIRST || joint > ARMS_JOINT_LAST || !(a->joints & ARMS_MASK_JOINT(joint)))
        return ARMS_E_ARG;
    jc = &a->cfg.joint[joint];
    l->pos_limit_min = jc->pos_limit_min;
    l->pos_limit_max = jc->pos_limit_max;
    l->vel_limit = jc->vel_limit;
    l->tor_limit = jc->tor_limit;
    return ARMS_OK;
}

static void post(arms_t *a, arms_mask joints, unsigned int req)
{
    int j;
    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j)
        if (joints & a->joints & ARMS_MASK_JOINT(j)) atomic_store(&a->req[j], req);
}

static const char *result_text(unsigned int r)
{
    switch (r) {
    case RES_TIMEOUT: return "超时";
    case RES_FAULT: return "驱动器故障";
    case RES_STATE: return "从站不在 OP 或请求被取消";
    case RES_COMM: return "通信未就绪或已失败";
    case RES_TRIP: return "离线保护已触发，需重新上电并通过在线检查";
    case RES_PROTECT: return "超速/超力矩/过温保护已触发，需先 reset";
    case RES_PENDING: return "未响应";
    }
    return "?";
}

/* 向 mask 中已打开的关节发出请求并等待结果；失败时返回首个失败关节的说明 */
static arms_status request_and_wait(arms_t *a, arms_mask mask, unsigned int req, unsigned int timeout_ms,
                                    const char *what)
{
    uint64_t deadline;
    int j, bad = -1;
    unsigned int bad_res = RES_PENDING;
    bool pending = true;

    mask &= a->joints;
    if (!mask) {
        set_error("%s: 没有选中已打开的关节", what);
        return ARMS_E_ARG;
    }
    if (!arms_ready(a)) {
        set_error("%s: 预热未完成", what);
        return ARMS_E_STATE;
    }
    if (!timeout_ms) timeout_ms = DEFAULT_WAIT_MS;
    atomic_store(&a->enable_timeout_cycles, (unsigned int)((uint64_t)timeout_ms * a->cycle_hz / 1000u));
    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
        if (!(mask & ARMS_MASK_JOINT(j))) continue;
        atomic_store(&a->res[j], RES_PENDING);
        atomic_store(&a->req[j], req);
    }
    /* 周期上下文自身也按 timeout_ms 结束请求，这里多留余量 */
    deadline = mono_ns() + ((uint64_t)timeout_ms + 200u) * 1000000ull;
    while (pending && mono_ns() < deadline) {
        pending = false;
        for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j)
            if ((mask & ARMS_MASK_JOINT(j)) && atomic_load(&a->res[j]) == RES_PENDING) pending = true;
        if (pending) usleep(2000);
    }
    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
        unsigned int r;
        if (!(mask & ARMS_MASK_JOINT(j))) continue;
        r = atomic_load(&a->res[j]);
        if (r != RES_OK && bad < 0) {
            bad = j;
            bad_res = r;
        }
    }
    if (bad < 0) {
        /* 等快照包含本次结果后再返回，保证随后的 arms_read()/arms_set_command() 看到新状态 */
        unsigned int s0 = atomic_load(&a->seq);
        uint64_t until = mono_ns() + 100000000ull;
        while (atomic_load(&a->seq) - s0 < 4u && mono_ns() < until) usleep(500);
        return ARMS_OK;
    }
    if (req == REQ_ENABLE) {
        arms_mask undone = 0;
        for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j)
            if ((mask & ARMS_MASK_JOINT(j)) && atomic_load(&a->res[j]) != RES_OK) undone |= ARMS_MASK_JOINT(j);
        post(a, undone, REQ_DISABLE);
    }
    {
        arms_joint_state s;
        arms_read_joint(a, bad, &s);
        set_error("%s失败: 关节 %d (%s) %s, 状态字 0x%04x, 错误码 0x%04x", what, bad, a->cfg.joint[bad].name,
                  result_text(bad_res), s.status_word, s.error_code);
    }
    if (bad_res == RES_TIMEOUT || bad_res == RES_PENDING) return ARMS_E_TIMEOUT;
    if (bad_res == RES_COMM) return ARMS_E_COMM;
    return ARMS_E_STATE;
}

/* 等待周期上下文发布两次快照，使随后的读取接口看到刚生效的指令或偏置（最长 100 ms） */
static void wait_snapshot(arms_t *a)
{
    unsigned int s0 = atomic_load(&a->seq);
    uint64_t until = mono_ns() + 100000000ull;
    while (atomic_load(&a->seq) - s0 < 4u && mono_ns() < until) usleep(500);
}

/*
 * 检查关节抱闸是否为正常控制模式（Pn075=0）。强制松闸（1）时失能后抱闸不闭合，
 * 关节失去保持；强制抱闸（2）时电机会顶着抱闸出力。用于必须在正常模式下执行的操作。
 */
static bool brake_abnormal(arms_t *a, arms_mask mask, const char *what)
{
    /* 库调用方不必先运行 CLI 自检。只在非实时管理上下文补读未知模式。 */
    arms_mask unknown = mask & ~atomic_load(&a->brake_known);
    for (int j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
        arms_brake_mode mode;
        if ((unknown & ARMS_MASK_JOINT(j)) && arms_get_brake(a, j, &mode) != ARMS_OK) return true;
    }
    arms_mask rel = atomic_load(&a->brake_released) & mask;
    arms_mask held = atomic_load(&a->brake_held) & mask;
    if (!rel && !held) return false;
    if (rel)
        set_error("%s被拒绝: 关节 %d 处于强制松闸，先执行 brake %d normal", what, __builtin_ctz(rel),
                  __builtin_ctz(rel));
    else
        set_error("%s被拒绝: 关节 %d 处于强制抱闸，先执行 brake %d normal", what, __builtin_ctz(held),
                  __builtin_ctz(held));
    return true;
}

/*
 * 使能前将所选关节的抱闸切回正常控制（Pn075=0）：强制松闸（1）或强制抱闸（2）
 * 写 0 并回读确认，不保存；模式未知时同样写 0。任一关节失败则拒绝本次操作。
 */
static arms_status brake_to_normal(arms_t *a, arms_mask mask, const char *what)
{
    mask &= a->joints;
    for (int j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
        arms_mask bit = ARMS_MASK_JOINT(j);
        arms_brake_mode mode = ARMS_BRAKE_UNKNOWN;
        arms_status st;
        if (!(mask & bit)) continue;
        if (atomic_load(&a->brake_known) & bit) {
            if (atomic_load(&a->brake_released) & bit) mode = ARMS_BRAKE_RELEASE;
            else if (atomic_load(&a->brake_held) & bit) mode = ARMS_BRAKE_HOLD;
            else mode = ARMS_BRAKE_NORMAL;
        } else if (arms_get_brake(a, j, &mode) != ARMS_OK) {
            mode = ARMS_BRAKE_UNKNOWN;
        }
        if (mode == ARMS_BRAKE_NORMAL) continue;
        st = arms_set_brake(a, j, ARMS_BRAKE_NORMAL, true);
        if (st != ARMS_OK) {
            char reason[sizeof(last_error)];
            snprintf(reason, sizeof(reason), "%s", last_error);
            set_error("%s被拒绝: 关节 %d 的抱闸无法切回正常控制（Pn075=0）: %s", what, j, reason);
            return st;
        }
        if (mode == ARMS_BRAKE_UNKNOWN)
            logf_(a, "关节 %d 抱闸模式未知，已写入正常控制抱闸（Pn075=0）并回读确认，未保存", j);
        else
            logf_(a, "关节 %d 抱闸由%s（Pn075=%d）自动切为正常控制（Pn075=0），未保存", j,
                  mode == ARMS_BRAKE_RELEASE ? "强制松闸" : "强制抱闸", (int)mode);
    }
    return ARMS_OK;
}

/* 硬件校零后、驱动器重新上电前，拒绝对该关节执行 what */
static bool zero_pending(arms_t *a, arms_mask mask, const char *what)
{
    arms_mask p = atomic_load(&a->zero_pending) & mask & a->joints;
    if (!p) return false;
    set_error("%s被拒绝: 关节 %d 已硬件校零（Pn101），需给该臂驱动器断电并重新上电后零点才生效", what,
              __builtin_ctz(p));
    return true;
}

arms_mask arms_zero_pending(const arms_t *a)
{
    return a ? atomic_load(&((arms_t *)a)->zero_pending) : 0;
}

arms_status arms_enable(arms_t *a, arms_mask joints, unsigned int timeout_ms)
{
    arms_status st;
    if (!a) return ARMS_E_ARG;
    if (zero_pending(a, joints, "使能")) return ARMS_E_STATE;
    if ((st = brake_to_normal(a, joints, "使能")) != ARMS_OK) return st;
    return request_and_wait(a, joints, REQ_ENABLE, timeout_ms, "使能");
}

arms_status arms_free(arms_t *a, arms_mask joints, double kd)
{
    arms_mask todo = 0;
    arms_status st;
    int j;
    if (!a) return ARMS_E_ARG;
    joints &= a->joints;
    if (!joints) {
        set_error("零力矩拖动: 没有选中已打开的关节");
        return ARMS_E_ARG;
    }
    if (!isfinite(kd) || kd < 0.0) {
        set_error("零力矩拖动: kd 必须是非负数");
        return ARMS_E_ARG;
    }
    if (zero_pending(a, joints, "零力矩拖动")) return ARMS_E_STATE;
    if ((st = brake_to_normal(a, joints, "零力矩拖动")) != ARMS_OK) return st;
    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
        arms_joint_state s;
        if (!(joints & ARMS_MASK_JOINT(j))) continue;
        arms_read_joint(a, j, &s);
        if (s.mode != ARMS_JS_ENABLED) todo |= ARMS_MASK_JOINT(j);
    }
    if (todo && (st = request_and_wait(a, todo, REQ_ENABLE, 0, "使能")) != ARMS_OK) return st;
    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
        arms_joint_state s;
        arms_command c;
        if (!(joints & ARMS_MASK_JOINT(j))) continue;
        arms_read_joint(a, j, &s);
        c.position = s.cmd_position; /* kp = 0 时位置不起作用 */
        c.velocity = 0.0;
        c.torque = 0.0;
        c.kp = 0.0;
        c.kd = kd;
        if ((st = arms_set_command(a, j, &c)) != ARMS_OK) return st;
    }
    wait_snapshot(a);
    return ARMS_OK;
}

arms_status arms_reset(arms_t *a, arms_mask joints, unsigned int timeout_ms)
{
    int j;
    if (!a) return ARMS_E_ARG;
    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j)
        if ((joints & a->joints & ARMS_MASK_JOINT(j))) atomic_store(&a->clear_comm[a->cfg.joint[j].arm], true);
    atomic_store(&a->clear_protect, true);
    return request_and_wait(a, joints, REQ_RESET, timeout_ms, "复位");
}

arms_status arms_disable(arms_t *a, arms_mask joints)
{
    if (!a) return ARMS_E_ARG;
    post(a, joints, REQ_DISABLE);
    return ARMS_OK;
}

arms_status arms_hold(arms_t *a, arms_mask joints)
{
    if (!a) return ARMS_E_ARG;
    post(a, joints, REQ_HOLD);
    return ARMS_OK;
}

arms_status arms_set_command(arms_t *a, int joint, const arms_command *cmd)
{
    arms_joint_state s;
    cmd_slot *sl;
    if (!a || !cmd || joint < ARMS_JOINT_FIRST || joint > ARMS_JOINT_LAST || !(a->joints & ARMS_MASK_JOINT(joint))) {
        set_error("关节 %d 未打开", joint);
        return ARMS_E_ARG;
    }
    /* kp、kd 不检查：为负或 NaN 时周期内按 0 下发（同参考 pack_cmd） */
    if (!isfinite(cmd->position) || !isfinite(cmd->velocity) || !isfinite(cmd->torque)) {
        set_error("关节 %d 的指令无效（位置、速度或前馈不是有限值）", joint);
        return ARMS_E_ARG;
    }
    arms_read_joint(a, joint, &s);
    if (s.mode != ARMS_JS_ENABLED) {
        set_error("关节 %d (%s) 未使能，当前 %s", joint, a->cfg.joint[joint].name, arms_joint_mode_name(s.mode));
        return ARMS_E_STATE;
    }
    sl = &a->slot[joint];
    pthread_mutex_lock(&a->cmd_lock);
    atomic_fetch_add_explicit(&sl->seq, 1, memory_order_relaxed);
    atomic_thread_fence(memory_order_release);
    cmd_payload data = {*cmd};
    store_words(sl->data, &data, WORDS_OF(cmd_payload));
    atomic_fetch_add_explicit(&sl->seq, 1, memory_order_release);
    pthread_mutex_unlock(&a->cmd_lock);
    return ARMS_OK;
}

/* Blocking mailbox operations belong to the non-real-time management thread. */
arms_status arms_get_joint_info(const arms_t *a, int j, arms_joint_info *out)
{
    if (!a || !out || j < 15 || j > 28 || !a->cfg.joint[j].present)
        return ARMS_E_ARG;
    const arms_joint_config *c = &a->cfg.joint[j];
    out->direction = c->direction;
    out->offset_rad = atomic_load(&a->offset_rad[j]);
    return ARMS_OK;
}
int arms_pn_index(unsigned n)
{
    return n > 999 ? -1 : (int)(0x2000u + ((n / 100) << 8) + (((n / 10) % 10) << 4) + n % 10);
}
static arms_status mailbox(arms_t *a, int j, uint16_t idx, uint8_t sub, uint8_t *p, size_t *n, bool write)
{
    if (!a || j < ARMS_JOINT_FIRST || j > ARMS_JOINT_LAST || !(a->joints & ARMS_MASK_JOINT(j)) || !p || !n ||
        !*n) {
        set_error("未打开的关节或无效缓冲区");
        return ARMS_E_ARG;
    }
    const arms_joint_config *jc = &a->cfg.joint[j];
    arm_rt *r = &a->arm[jc->arm];
    uint32_t abort = 0;
    if (r->ops->sdo(r->bus, jc->slave, idx, sub, p, n, write, &abort)) {
        set_error("关节 %d SDO %04x:%02x %s失败 (abort=0x%08x)", j, idx, sub, write ? "写" : "读", abort);
        return ARMS_E_COMM;
    }
    return ARMS_OK;
}
arms_status arms_sdo_read(arms_t *a, int j, uint16_t idx, uint8_t sub, uint8_t *out, size_t cap, size_t *n)
{
    if (!n)
        return ARMS_E_ARG;
    *n = cap;
    return mailbox(a, j, idx, sub, out, n, false);
}
static arms_status verified(arms_t *a, int j, unsigned n, uint16_t value)
{
    uint8_t data[2];
    size_t len = 2;
    uint16_t idx = (uint16_t)arms_pn_index(n);
    kgu_put_u16(data, 0, value);
    arms_status st = mailbox(a, j, idx, 0, data, &len, true);
    if (st != ARMS_OK)
        return st;
    st = arms_sdo_read(a, j, idx, 0, data, sizeof(data), &len);
    if (st != ARMS_OK)
        return st;
    if (len != 2 || kgu_get_u16(data, 0) != value) {
        set_error("关节 %d Pn%03u 回读不一致", j, n);
        return ARMS_E_COMM;
    }
    return ARMS_OK;
}
/* 从同一周期的快照取得状态与通信统计，不把失能请求当作驱动器反馈。 */
static arms_status maintenance_read(arms_t *a, int j, bool confirmed, arms_joint_state *s, arms_bus_stats *stats)
{
    if (!a || !confirmed) {
        set_error("维护接口需要调用方明确授权");
        return ARMS_E_STATE;
    }
    if (j < ARMS_JOINT_FIRST || j > ARMS_JOINT_LAST || !(a->joints & ARMS_MASK_JOINT(j))) {
        set_error("关节 %d 未打开", j);
        return ARMS_E_ARG;
    }
    shared_snapshot snap;
    read_snapshot(a, &snap);
    *s = snap.s[j];
    apply_brake_mode(a, s);
    *stats = snap.stats;
    const arms_arm_stats *as = &stats->arm[a->cfg.joint[j].arm];
    if (s->al_state != 0x08 || !as->link_up || as->comm_failed || as->wkc_last != as->wkc_expected ||
        !s->sample_ns || mono_ns() - s->sample_ns > 50000000ull) {
        set_error("关节 %d 不在线或反馈过期", j);
        return ARMS_E_STATE;
    }
    return ARMS_OK;
}

static bool drive_disabled(const arms_joint_state *s)
{
    return (s->mode == ARMS_JS_LOCKED || s->mode == ARMS_JS_RELEASED || s->mode == ARMS_JS_HELD) &&
           (s->status_word & 0x006f) == 0x0040;
}

static arms_status maintenance_ready_ex(arms_t *a, int j, bool confirmed, arms_joint_state *s, bool need_disabled)
{
    arms_bus_stats stats;
    arms_status st = maintenance_read(a, j, confirmed, s, &stats);
    if (st != ARMS_OK) return st;
    if (need_disabled && !drive_disabled(s)) {
        set_error("关节 %d 未失能（当前 %s），先执行 disable", j, arms_joint_mode_name(s->mode));
        return ARMS_E_STATE;
    }
    return ARMS_OK;
}

/* 运行中的 MIT 主站拥有这些周期输出及其映射，不允许两个生产者同时写同一状态。
 * 与 Pn 手册目录无关；其余 SDO 由从站按自己的对象字典和访问属性应答。 */
static bool owned_by_cycle(uint16_t index, uint8_t sub)
{
    if (index == 0x1601 || index == 0x1a01 || index == 0x1c12 || index == 0x1c13) return true;
    return !sub && (index == 0x6040 || index == 0x6060 || (index >= 0x4010 && index <= 0x4014));
}

arms_status arms_sdo_write(arms_t *a, int j, uint16_t idx, uint8_t sub, const uint8_t *data,
                           size_t len, bool confirmed)
{
    if (!data || !len) { set_error("SDO 写入需要非空数据"); return ARMS_E_ARG; }
    if (!sub && (idx == 0x2075 || idx == 0x2097 || idx == 0x2101)) {
        if (len != 2) { set_error("抱闸/保存/清零动作需要 2 字节"); return ARMS_E_ARG; }
        unsigned v = kgu_get_u16(data, 0);
        if (idx == 0x2075) return arms_set_brake(a, j, (arms_brake_mode)v, confirmed);
        if (v != 1) { set_error("保存/清零只接受写 1 触发，复位由程序完成"); return ARMS_E_ARG; }
        return arms_service(a, j, idx == 0x2097 ? ARMS_SAVE : ARMS_ZERO_HW, confirmed);
    }
    arms_joint_state s;
    arms_bus_stats stats;
    arms_status st = maintenance_read(a, j, confirmed, &s, &stats);
    if (st != ARMS_OK) return st;
    if (stats.tripped || stats.protect.reason != ARMS_PROTECT_NONE || s.error_code || !drive_disabled(&s)) {
        set_error("关节 %d 未确认失能或有故障/保护，未写 %04x:%02x", j, idx, sub);
        return ARMS_E_STATE;
    }
    if (owned_by_cycle(idx, sub)) {
        set_error("%04x:%02x 由正在运行的 PDO 循环接管，不并行写入", idx, sub);
        return ARMS_E_STATE;
    }
    /* bus_ops 的同一回调用于上传和下载；下载不会修改调用方的缓冲区。 */
    return mailbox(a, j, idx, sub, (uint8_t *)data, &len, true);
}

arms_status arms_pn_write(arms_t *a, int j, unsigned n, int v, bool confirmed)
{
    int index = arms_pn_index(n);
    if (index < 0 || v < INT16_MIN || v > INT16_MAX) {
        set_error("需要 Pn000..999 和 INT16 原始整数"); return ARMS_E_ARG;
    }
    uint8_t data[2];
    kgu_put_u16(data, 0, (uint16_t)v);
    return arms_sdo_write(a, j, (uint16_t)index, 0, data, sizeof(data), confirmed);
}

const char *arms_brake_mode_name(arms_brake_mode m)
{
    switch (m) {
    case ARMS_BRAKE_NORMAL: return "normal";
    case ARMS_BRAKE_RELEASE: return "release";
    case ARMS_BRAKE_HOLD: return "hold";
    default: return "unknown";
    }
}

static void record_brake(arms_t *a, int j, arms_brake_mode m)
{
    arms_mask bit = ARMS_MASK_JOINT(j);
    if (m == ARMS_BRAKE_RELEASE) atomic_fetch_or(&a->brake_released, bit);
    else atomic_fetch_and(&a->brake_released, ~bit);
    if (m == ARMS_BRAKE_HOLD) atomic_fetch_or(&a->brake_held, bit);
    else atomic_fetch_and(&a->brake_held, ~bit);
    atomic_fetch_or(&a->brake_known, bit);
}

arms_status arms_get_brake(arms_t *a, int j, arms_brake_mode *mode)
{
    uint8_t data[4];
    size_t n = 0;
    unsigned v;
    arms_status st;
    if (!a || !mode || j < ARMS_JOINT_FIRST || j > ARMS_JOINT_LAST) return ARMS_E_ARG;
    atomic_fetch_and(&a->brake_known, ~ARMS_MASK_JOINT(j));
    *mode = ARMS_BRAKE_UNKNOWN;
    st = arms_sdo_read(a, j, (uint16_t)arms_pn_index(75), 0, data, sizeof(data), &n);
    if (st != ARMS_OK) return st;
    if (n != 2) {
        set_error("关节 %d 的 Pn075 读回 %zu 字节", j, n);
        return ARMS_E_COMM;
    }
    v = kgu_get_u16(data, 0);
    if (v > 2) {
        set_error("关节 %d 的 Pn075 = %u，不是有效的抱闸模式", j, v);
        return ARMS_E_STATE;
    }
    *mode = (arms_brake_mode)v;
    record_brake(a, j, *mode);
    return ARMS_OK;
}

arms_status arms_brake_scan(arms_t *a, arms_mask *released, arms_mask *held, arms_mask *unknown)
{
    arms_mask rel = 0, hld = 0, unk = 0;
    int j;
    if (!a) return ARMS_E_ARG;
    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
        arms_brake_mode m;
        if (!(a->joints & ARMS_MASK_JOINT(j))) continue;
        if (arms_get_brake(a, j, &m) != ARMS_OK) unk |= ARMS_MASK_JOINT(j);
        else if (m == ARMS_BRAKE_RELEASE) rel |= ARMS_MASK_JOINT(j);
        else if (m == ARMS_BRAKE_HOLD) hld |= ARMS_MASK_JOINT(j);
    }
    if (released) *released = rel;
    if (held) *held = hld;
    if (unknown) *unknown = unk;
    return ARMS_OK;
}

arms_status arms_set_brake(arms_t *a, int j, arms_brake_mode mode, bool confirmed)
{
    arms_joint_state s;
    arms_status st;
    if (!a) return ARMS_E_ARG;
    if (mode != ARMS_BRAKE_NORMAL && mode != ARMS_BRAKE_RELEASE && mode != ARMS_BRAKE_HOLD) {
        set_error("无效的抱闸模式");
        return ARMS_E_ARG;
    }
    /* 恢复正常控制不要求失能；强制松闸/抱闸要求失能 */
    st = maintenance_ready_ex(a, j, confirmed, &s, mode != ARMS_BRAKE_NORMAL);
    if (st != ARMS_OK) return st;
    a->brake_touched |= ARMS_MASK_JOINT(j);
    st = verified(a, j, 75, (uint16_t)mode);
    if (st == ARMS_OK) {
        record_brake(a, j, mode);
    } else {
        /* 写入结果不确定时重新读取，以驱动器实际值为准；读取失败则按未知模式联锁 */
        arms_brake_mode now;
        char saved[sizeof(last_error)];
        snprintf(saved, sizeof(saved), "%s", last_error);
        arms_get_brake(a, j, &now);
        snprintf(last_error, sizeof(last_error), "%s", saved);
    }
    return st;
}

static double zero_value(const arms_t *a, int j, const arms_joint_state *s)
{
    return a->cfg.joint[j].direction * (double)kgu_get_f32(s->txpdo, KGU_IN_POS);
}

/* 校零与保存共用：仅在需要时请求目标关节失能，以新 PDO 反馈验证静止，不重新使能。 */
static arms_status prepare_stationary(arms_t *a, int j, bool confirmed, arms_joint_state *s, const char *operation)
{
    const uint64_t deadline = mono_ns() + 1000000000ull;
    uint64_t quiet_since = 0;
    bool requested = false;
    do {
        arms_bus_stats stats;
        arms_status st = maintenance_read(a, j, confirmed, s, &stats);
        if (st != ARMS_OK) return st;
        if (stats.tripped || stats.protect.reason != ARMS_PROTECT_NONE || s->error_code ||
            s->mode == ARMS_JS_FAULT || !isfinite(s->position) || !isfinite(s->velocity)) {
            set_error("关节 %d 故障、保护锁定或反馈无效，未%s", j, operation);
            return ARMS_E_STATE;
        }
        if (drive_disabled(s)) {
            if (fabs(s->velocity) <= 0.01) {
                if (!quiet_since) quiet_since = s->sample_ns;
                if (s->sample_ns - quiet_since >= 50000000ull) return ARMS_OK;
            } else quiet_since = 0;
        } else {
            quiet_since = 0;
            if (!requested) {
                if (s->mode != ARMS_JS_ENABLED) {
                    set_error("关节 %d 状态字 0x%04x 不允许%s", j, s->status_word, operation);
                    return ARMS_E_STATE;
                }
                st = arms_disable(a, ARMS_MASK_JOINT(j));
                if (st != ARMS_OK) return st;
                requested = true;
            }
        }
        usleep(2000);
    } while (mono_ns() < deadline);
    set_error("关节 %d 在 1 秒内未确认失能并静止，未%s（状态字 0x%04x，速度 %.4f rad/s）",
              j, operation, s->status_word, s->velocity);
    return ARMS_E_TIMEOUT;
}

arms_status arms_service(arms_t *a, int j, arms_service_op op, bool confirmed)
{
    arms_joint_state s;
    if (op != ARMS_ZERO_SOFT && op != ARMS_ZERO_HW && op != ARMS_SAVE) return ARMS_E_ARG;
    if (j < ARMS_JOINT_FIRST || j > ARMS_JOINT_LAST) return ARMS_E_ARG;
    if (op == ARMS_ZERO_SOFT && zero_pending(a, ARMS_MASK_JOINT(j), "软件校零")) return ARMS_E_STATE;
    arms_status st = prepare_stationary(a, j, confirmed, &s, op == ARMS_SAVE ? "保存参数" : "校零");
    if (st != ARMS_OK)
        return st;
    uint8_t data[2];
    size_t n = 0;
    switch (op) {
    case ARMS_ZERO_SOFT: {
        double off = zero_value(a, j, &s);
        if (!arms_store_offset(a->cfg.path, j, off)) {
            set_error("无法把关节 %d 的 offset_rad 写入 %s", j, a->cfg.path);
            return ARMS_E_SYSTEM;
        }
        /* 立即生效：周期上下文下一周期按新偏置换算位置（关节已失能，不影响指令） */
        atomic_store(&a->offset_rad[j], off);
        wait_snapshot(a);
        return ARMS_OK;
    }
    case ARMS_ZERO_HW:
    case ARMS_SAVE: {
        unsigned pn = op == ARMS_SAVE ? 97 : 101;
        if (op == ARMS_ZERO_HW && brake_abnormal(a, ARMS_MASK_JOINT(j), "硬件校零")) return ARMS_E_STATE;
        st = arms_sdo_read(a, j, arms_pn_index(pn), 0, data, 2, &n);
        if (st != ARMS_OK)
            return st;
        if (n != 2 || kgu_get_u16(data, 0) != 0) {
            set_error("关节 %d 的 Pn%03u 未处于初始值 0", j, pn);
            return ARMS_E_STATE;
        }
        st = verified(a, j, pn, 1);
        usleep(100000);
        arms_status reset = verified(a, j, pn, 0);
        if (reset != ARMS_OK) return reset;
        if (st != ARMS_OK || op != ARMS_ZERO_HW) return st;
        /* 硬件零点取代软件偏置：配置立即写 0，运行中的偏置在驱动器断电时切换 */
        atomic_fetch_or(&a->zero_pending, ARMS_MASK_JOINT(j));
        if (!arms_store_offset(a->cfg.path, j, 0.0)) {
            set_error("关节 %d 已发出硬件清零脉冲，但无法把 %s 中的 offset_rad 写为 0，请手动修改", j, a->cfg.path);
            return ARMS_E_SYSTEM;
        }
        return ARMS_OK;
    }
    default:
        return ARMS_E_ARG;
    }
}
