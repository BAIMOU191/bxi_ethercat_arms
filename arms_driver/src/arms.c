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
#include <sys/mman.h>
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
#define MOVE_MAX_S 60.0

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
} arm_rt;

typedef struct {
    arms_joint_state s[ARMS_JOINT_LAST + 1];
    arms_bus_stats stats;
} shared_snapshot;

/* 指令槽：非实时线程在 cmd_lock 下写入，周期上下文无锁读取（seqlock，读失败沿用上一周期） */
typedef struct {
    atomic_uint seq;
    arms_command cmd;
    double move_s;
} cmd_slot;

/* 关节控制状态，仅在周期上下文中访问 */
typedef struct {
    int phase;
    uint64_t phase_cycle;
    unsigned int seq_seen;
    uint64_t fresh_cycle;
    arms_command cur;                 /* 当前生效的指令（关节侧，未限幅） */
    double lim_min, lim_max;          /* 生效的位置限位，使能时若实际位置在限位外则放宽到该位置 */
    bool moving;
    double mv_from, mv_to;
    uint64_t mv_start, mv_len;
    bool timed_out;
    bool was_fault;
    uint32_t fault_count;
    uint32_t over_vel, over_tor;      /* 超限累计周期数：超限 +1，未超限 -1 */
} joint_ctl;

struct arms {
    atomic_uint brake_confirmed;
    arms_mask released; /* 管理线程独占；含尚未确认锁回的关节 */
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
    shared_snapshot snap;

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
    uint64_t cycles, overruns, skipped;
    double wake_late_max, work_max;
    uint64_t last_start_ns;
    uint64_t cycle_start_ns;
    unsigned int poll_div, poll_cnt;
    uint64_t stable_cycles, ready_cycles;
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
    atomic_fetch_add_explicit(&a->seq, 1, memory_order_acq_rel);
    atomic_thread_fence(memory_order_release);
    memcpy(a->snap.s, a->js, sizeof(a->js));
    a->snap.stats.cycle_hz = a->cycle_hz;
    a->snap.stats.cycles = a->cycles;
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
    atomic_thread_fence(memory_order_release);
    atomic_fetch_add_explicit(&a->seq, 1, memory_order_acq_rel);
}

static void read_snapshot(arms_t *a, shared_snapshot *out)
{
    unsigned int s1, s2 = 0;
    do {
        s1 = atomic_load_explicit(&a->seq, memory_order_acquire);
        if (s1 & 1u) continue;
        atomic_thread_fence(memory_order_acquire);
        memcpy(out, &a->snap, sizeof(*out));
        atomic_thread_fence(memory_order_acquire);
        s2 = atomic_load_explicit(&a->seq, memory_order_acquire);
    } while ((s1 & 1u) || s1 != s2);
}

static void do_reset_stats(arms_t *a)
{
    int i;
    rs_reset(&a->period);
    a->cycles = a->overruns = a->skipped = 0;
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
    ov = fabs(s->velocity) > jc->vel_fault;
    ot = fabs(s->torque) > jc->tor_fault;
    c->over_vel = ov ? c->over_vel + 1 : (c->over_vel ? c->over_vel - 1 : 0);
    c->over_tor = ot ? c->over_tor + 1 : (c->over_tor ? c->over_tor - 1 : 0);
    if (a->protect.reason != ARMS_PROTECT_NONE) return;
    if (c->over_vel >= a->overspeed_cycles) {
        a->protect.reason = ARMS_PROTECT_OVERSPEED;
        a->protect.joint = j;
        a->protect.value = s->velocity;
        a->protect.limit = jc->vel_fault;
    } else if (c->over_tor >= a->overtorque_cycles) {
        a->protect.reason = ARMS_PROTECT_OVERTORQUE;
        a->protect.joint = j;
        a->protect.value = s->torque;
        a->protect.limit = jc->tor_fault;
    }
}

/* ---- 周期处理，独立模式与外部驱动模式共用 ---- */

static void cycle_rx(arms_t *a, uint64_t now)
{
    bool ready = atomic_load_explicit(&a->ready, memory_order_relaxed);
    bool all_good = true;
    int i, k;

    if (atomic_exchange_explicit(&a->reset_stats, false, memory_order_acq_rel)) do_reset_stats(a);
    a->cycle_start_ns = now;
    if (a->last_start_ns && now > a->last_start_ns) {
        uint64_t dt = now - a->last_start_ns;
        rs_add(&a->period, (double)dt);
        if (dt * 2 > a->period_ns * 3) a->overruns++;
    }
    a->last_start_ns = now;

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
                if (r->consecutive >= a->cfg.wkc_tolerance && !r->comm_failed) {
                    r->comm_failed = true;
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
            if (wkc > 0) {
                kgu_feedback f;
                memcpy(s->txpdo, r->ops->inputs(r->bus, k), sizeof(s->txpdo));
                kgu_decode(s->txpdo, &f);
                r->slot_pos[k] = f.pos;
                s->status_word = f.statusword;
                s->error_code = f.error;
                s->position = jc->direction * (double)f.pos - jc->offset_rad;
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
        }
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
        }
    }

    if (!ready) {
        a->stable_cycles = all_good ? a->stable_cycles + 1 : 0;
        if (a->stable_cycles >= a->ready_cycles) {
            do_reset_stats(a);
            atomic_store_explicit(&a->ready, true, memory_order_release);
        }
    }
}

static double clampd(double v, double lo, double hi, bool *limited)
{
    if (v < lo) { *limited = true; return lo; }
    if (v > hi) { *limited = true; return hi; }
    return v;
}

static void finish(arms_t *a, int j, unsigned int result)
{
    atomic_store_explicit(&a->res[j], result, memory_order_release);
}

/* 尝试读取指令槽；有新指令时返回 true */
static bool fetch_command(arms_t *a, int j, arms_command *cmd, double *move_s)
{
    cmd_slot *sl = &a->slot[j];
    joint_ctl *c = &a->ctl[j];
    unsigned int s1, s2;
    arms_command tmp;
    double mv;

    s1 = atomic_load_explicit(&sl->seq, memory_order_acquire);
    if ((s1 & 1u) || s1 == c->seq_seen) return false;
    tmp = sl->cmd;
    mv = sl->move_s;
    atomic_thread_fence(memory_order_acquire);
    s2 = atomic_load_explicit(&sl->seq, memory_order_relaxed);
    if (s1 != s2) return false;
    c->seq_seen = s1;
    *cmd = tmp;
    *move_s = mv;
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
    bool active, limited = false;

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
            c->cur.position = s->position;
            c->cur.velocity = 0.0;
            c->cur.torque = 0.0;
            c->cur.kp = jc->kp_default;
            c->cur.kd = jc->kd_default;
            c->lim_min = s->position < jc->pos_min ? s->position : jc->pos_min;
            c->lim_max = s->position > jc->pos_max ? s->position : jc->pos_max;
            c->moving = false;
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
            c->moving = false;
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
        double mv;
        out->controlword = KGU_CW_ENABLE;
        if (fetch_command(a, j, &cmd, &mv)) {
            c->fresh_cycle = a->cycles;
            c->timed_out = false;
            if (mv > 0.0) {
                c->moving = true;
                c->mv_from = c->cur.position;
                c->mv_to = cmd.position;
                c->mv_start = a->cycles;
                c->mv_len = (uint64_t)(mv * (double)a->cycle_hz);
                if (c->mv_len == 0) c->mv_len = 1;
                c->cur.velocity = 0.0;
                c->cur.torque = 0.0;
                c->cur.kp = cmd.kp;
                c->cur.kd = cmd.kd;
            } else {
                c->moving = false;
                c->cur = cmd;
            }
        }
        if (c->moving) {
            double t = (double)(a->cycles - c->mv_start) / (double)c->mv_len;
            if (t >= 1.0) {
                c->cur.position = c->mv_to;
                c->moving = false;
                c->fresh_cycle = a->cycles;
            } else {
                c->cur.position = c->mv_from + (c->mv_to - c->mv_from) * 0.5 * (1.0 - cos(M_PI * t));
            }
        } else if (a->cycles - c->fresh_cycle > a->timeout_cycles && !c->timed_out) {
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
        double pos = clampd(c->cur.position, c->lim_min, c->lim_max, &limited);
        double vel = clampd(c->cur.velocity, -jc->vel_max, jc->vel_max, &limited);
        double tff = clampd(c->cur.torque, -jc->tff_max, jc->tff_max, &limited);
        double kp = clampd(c->cur.kp, 0.0, jc->kp_max, &limited);
        double kd = clampd(c->cur.kd, 0.0, jc->kd_max, &limited);
        double d = (double)jc->direction;
        out->pos = (float)(d * (pos + jc->offset_rad));
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
    s->cmd_moving = active && c->moving;
    s->cmd_limited = limited;
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

        cycle_rx(a, start);
        if ((double)late > a->wake_late_max) a->wake_late_max = (double)late;
        cycle_tx(a, mono_ns());

        next_ns += a->period_ns;
        start = mono_ns();
        while (next_ns <= start) {
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
    atomic_store(&a->enable_timeout_cycles, DEFAULT_WAIT_MS * a->cycle_hz / 1000u);
    pthread_mutex_init(&a->cmd_lock, NULL);
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
    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
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

    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0 && options->backend != ARMS_BACKEND_MOCK) {
        set_error("mlockall 失败: %s，需要 root", strerror(errno));
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
        logf_(a, "%s 臂: 网口 %s, 主站 %d, 扫描中", arms_arm_name(i), ac->interface, r->ops->master_index(r->bus));
        if (r->ops->scan(r->bus, infos, BUS_MAX_SLAVES, &found,
                         options->scan_timeout_ms ? options->scan_timeout_ms : DEFAULT_SCAN_TIMEOUT_MS, err,
                         sizeof(err)) != 0) {
            if (check || strstr(err, "DOWN")) {
                set_error("%s 臂: %s", arms_arm_name(i), err);
                st = strstr(err, "DOWN") ? ARMS_E_LINK : ARMS_E_TOPOLOGY;
                goto fail;
            }
            logf_(a, "警告: %s 臂: %s", arms_arm_name(i), err);
            found = 0;
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
        for (int j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; j++)
            if (a->released & ARMS_MASK_JOINT(j)) {
                if (arms_service(a, j, ARMS_LOCK, true) != ARMS_OK)
                    logf_(a, "警告：关节 %d 未能确认锁闸，请保持机械支撑", j);
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
    logf_(a, "预热完成: %d 个关节处于 OP, 伺服失能", a->arm[0].count + a->arm[1].count);
    return ARMS_OK;
}

uint64_t arms_period_ns(const arms_t *a) { return a ? a->period_ns : 0; }

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
            if (states[n].mode == ARMS_JS_LOCKED && (atomic_load(&a->brake_confirmed) & ARMS_MASK_JOINT(j)))
                states[n].mode = ARMS_JS_RELEASED;
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
    if (state->mode == ARMS_JS_LOCKED && (atomic_load(&a->brake_confirmed) & ARMS_MASK_JOINT(joint)))
        state->mode = ARMS_JS_RELEASED;
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
    l->pos_min = jc->pos_min;
    l->pos_max = jc->pos_max;
    l->vel_max = jc->vel_max;
    l->tff_max = jc->tff_max;
    l->kp_max = jc->kp_max;
    l->kd_max = jc->kd_max;
    l->kp_default = jc->kp_default;
    l->kd_default = jc->kd_default;
    l->home = jc->home;
    l->vel_fault = jc->vel_fault;
    l->torque_fault = jc->tor_fault;
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
    case RES_PROTECT: return "超速/超力矩保护已触发，需先 reset";
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

arms_status arms_enable(arms_t *a, arms_mask joints, unsigned int timeout_ms)
{
    if (!a) return ARMS_E_ARG;
    if (a->released) {
        /* 松闸状态下使能后再失能，抱闸不会闭合，关节会失去保持 */
        set_error("关节 %d 处于松闸状态，先执行 lock 再使能", __builtin_ctz(a->released));
        return ARMS_E_STATE;
    }
    return request_and_wait(a, joints, REQ_ENABLE, timeout_ms, "使能");
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

arms_status arms_set_command(arms_t *a, int joint, const arms_command *cmd, double move_s)
{
    arms_joint_state s;
    cmd_slot *sl;
    if (!a || !cmd || joint < ARMS_JOINT_FIRST || joint > ARMS_JOINT_LAST || !(a->joints & ARMS_MASK_JOINT(joint))) {
        set_error("关节 %d 未打开", joint);
        return ARMS_E_ARG;
    }
    if (!isfinite(cmd->position) || !isfinite(cmd->velocity) || !isfinite(cmd->torque) || !isfinite(cmd->kp) ||
        !isfinite(cmd->kd) || cmd->kp < 0.0 || cmd->kd < 0.0 || !isfinite(move_s) || move_s < 0.0 ||
        move_s > MOVE_MAX_S) {
        set_error("关节 %d 的指令无效（数值非有限、增益为负或过渡时间超出 0..%.0f s）", joint, MOVE_MAX_S);
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
    sl->cmd = *cmd;
    sl->move_s = move_s;
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
    out->offset_rad = c->offset_rad;
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
static arms_status maintenance_ready(arms_t *a, int j, bool confirmed, arms_joint_state *s)
{
    if (!a || !confirmed) {
        set_error("维护操作需要明确确认（--confirm）");
        return ARMS_E_STATE;
    }
    if (arms_read_joint(a, j, s) != ARMS_OK)
        return ARMS_E_ARG;
    arms_bus_stats stats;
    arms_get_stats(a, &stats);
    if (stats.tripped || arms_check_online(a, NULL) != ARMS_OK ||
        (s->mode != ARMS_JS_LOCKED && s->mode != ARMS_JS_RELEASED) || (s->status_word & 0x006f) != 0x0040 ||
        !s->sample_ns || mono_ns() - s->sample_ns > 50000000ull) {
        set_error("要求反馈新鲜、通信正常、未触发保护、驱动器已失能");
        return ARMS_E_STATE;
    }
    return ARMS_OK;
}
arms_status arms_pn_write(arms_t *a, int j, unsigned n, unsigned v, bool confirmed)
{
    arms_joint_state s;
    arms_status st = maintenance_ready(a, j, confirmed, &s);
    if (st != ARMS_OK)
        return st;
    if (a->released) {
        set_error("请先锁回抱闸，再修改参数");
        return ARMS_E_STATE;
    }
    const unsigned allowed[][2] = {{28, 5000}, {44, 9}, {82, 32767}, {106, 300}, {107, 1}};
    for (size_t i = 0; i < sizeof(allowed) / sizeof(*allowed); i++)
        if (n == allowed[i][0] && v <= allowed[i][1])
            return verified(a, j, n, v);
    set_error("Pn 不在可写范围内；抱闸、校零、参数保存请使用 release/lock、zero、pn_save 命令");
    return ARMS_E_ARG;
}
arms_status arms_service(arms_t *a, int j, arms_service_op op, bool confirmed)
{
    arms_joint_state s;
    arms_status st = maintenance_ready(a, j, confirmed, &s);
    if (st != ARMS_OK)
        return st;
    arms_mask bit = ARMS_MASK_JOINT(j);
    uint8_t data[2];
    size_t n = 0;
    switch (op) {
    case ARMS_RELEASE:
        if (a->released & ~bit) {
            set_error("请先锁回已释放关节");
            return ARMS_E_STATE;
        }
        for (int other = ARMS_JOINT_FIRST; other <= ARMS_JOINT_LAST; other++)
            if (other != j && (a->joints & ARMS_MASK_JOINT(other))) {
                st = arms_sdo_read(a, other, arms_pn_index(75), 0, data, 2, &n);
                if (st != ARMS_OK)
                    return st;
                if (n != 2 || kgu_get_u16(data, 0) != 0) {
                    set_error("关节 %d 抱闸参数非默认或未知；拒绝释放第二个关节", other);
                    return ARMS_E_STATE;
                }
            }
        a->released |= bit; /* a failed acknowledgement does not prove the brake is engaged */
        st = verified(a, j, 75, 1);
        if (st == ARMS_OK)
            atomic_fetch_or(&a->brake_confirmed, bit);
        return st;
    case ARMS_LOCK:
        st = verified(a, j, 75, 0);
        if (st == ARMS_OK) {
            a->released &= ~bit;
            atomic_fetch_and(&a->brake_confirmed, ~bit);
        }
        return st;
    case ARMS_ZERO_SOFT:
        if (fabs(s.velocity) > 0.01 ||
            !arms_store_offset(a->cfg.path, j,
                               a->cfg.joint[j].direction * (double)kgu_get_f32(s.txpdo, KGU_IN_POS))) {
            set_error("关节未静止或无法保存偏置");
            return ARMS_E_STATE;
        }
        return ARMS_OK; /* next session reloads the calibrated frame */
    case ARMS_ZERO_HW:
    case ARMS_SAVE: {
        if (a->released || fabs(s.velocity) > 0.01) {
            set_error("请先锁回抱闸并保持静止");
            return ARMS_E_STATE;
        }
        unsigned pn = op == ARMS_SAVE ? 97 : 101;
        st = arms_sdo_read(a, j, arms_pn_index(pn), 0, data, 2, &n);
        if (st != ARMS_OK)
            return st;
        if (n != 2 || kgu_get_u16(data, 0) != 0) {
            set_error("触发参数未处于初始零状态");
            return ARMS_E_STATE;
        }
        st = verified(a, j, pn, 1);
        usleep(100000);
        arms_status reset = verified(a, j, pn, 0);
        return reset != ARMS_OK ? reset : st;
    }
    default:
        return ARMS_E_ARG;
    }
}
