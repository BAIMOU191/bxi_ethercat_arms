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
    double recv_us_max, send_us_max;
} arm_rt;

typedef struct {
    arms_joint_state s[ARMS_JOINT_LAST + 1];
    arms_bus_stats stats;
} shared_snapshot;

struct arms {
    arms_config cfg;
    arms_options opt;
    const bus_ops *ops;
    arms_run_mode mode;
    int rt_cpu;
    unsigned int cycle_hz;
    uint64_t period_ns;
    arms_mask joints;
    arm_rt arm[ARMS_ARM_COUNT];

    pthread_t thread;
    bool thread_started;
    atomic_bool stop;
    atomic_bool ready;
    atomic_bool reset_stats;
    atomic_uint seq;
    shared_snapshot snap;

    /* 以下仅在周期上下文中访问 */
    arms_joint_state js[ARMS_JOINT_LAST + 1];
    running_stat period;
    uint64_t cycles, overruns, skipped;
    double wake_late_max, work_max;
    uint64_t last_start_ns;
    uint64_t cycle_start_ns;
    unsigned int poll_div, poll_cnt;
    uint64_t stable_cycles, ready_cycles;
};

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
        if (!r->used) continue;
        t0 = mono_ns();
        wkc = r->ops->receive(r->bus);
        t1 = mono_ns();
        if ((double)(t1 - t0) / 1000.0 > r->recv_us_max) r->recv_us_max = (double)(t1 - t0) / 1000.0;
        r->cycles++;
        r->wkc_last = wkc < 0 ? 0 : (uint32_t)wkc;
        if (wkc < 0 || (uint32_t)wkc != r->wkc_expected) {
            all_good = false;
            if (ready) {
                r->wkc_bad++;
                r->consecutive++;
                if (r->consecutive > r->consecutive_max) r->consecutive_max = r->consecutive;
                if (r->consecutive >= a->cfg.wkc_tolerance) r->comm_failed = true;
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
                kgu_decode(r->ops->inputs(r->bus, k), &f);
                r->slot_pos[k] = f.pos;
                s->status_word = f.statusword;
                s->error_code = f.error;
                s->position = jc->direction * (double)f.pos - jc->offset_rad;
                s->velocity = jc->direction * (double)f.vel;
                s->torque = jc->direction * (double)f.torque;
                s->sample_ns = t1;
            }
            s->al_state = r->al[k];
            s->mode = mode_from(r->al[k], s->status_word);
            if ((r->al[k] & 0x0F) != 0x08) all_good = false;
        }
    }

    if (++a->poll_cnt >= a->poll_div) {
        a->poll_cnt = 0;
        for (i = 0; i < ARMS_ARM_COUNT; ++i)
            if (a->arm[i].used) a->arm[i].ops->update_states(a->arm[i].bus, a->arm[i].al, &a->arm[i].link_up);
    }

    if (!ready) {
        a->stable_cycles = all_good ? a->stable_cycles + 1 : 0;
        if (a->stable_cycles >= a->ready_cycles) {
            do_reset_stats(a);
            atomic_store_explicit(&a->ready, true, memory_order_release);
        }
    }
}

static void cycle_tx(arms_t *a, uint64_t now)
{
    uint64_t end;
    int i, k;

    for (i = 0; i < ARMS_ARM_COUNT; ++i) {
        arm_rt *r = &a->arm[i];
        if (!r->used) continue;
        for (k = 0; k < r->count; ++k) {
            kgu_command c;
            c.controlword = KGU_CW_DISABLE;
            c.pos = r->slot_pos[k];
            c.vel = 0.0f;
            c.tff = 0.0f;
            c.kp = 0.0f;
            c.kd = 0.0f;
            c.mode = KGU_MODE_MIT;
            kgu_encode(r->ops->outputs(r->bus, k), &c);
        }
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

    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
        if (!a->cfg.joint[j].present) continue;
        if (options->joints && !(options->joints & ARMS_MASK_JOINT(j))) continue;
        a->joints |= ARMS_MASK_JOINT(j);
    }
    if (!a->joints) {
        set_error("所选关节在配置中都不存在");
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
            set_error("%s 臂: %s", arms_arm_name(i), err);
            st = strstr(err, "DOWN") ? ARMS_E_LINK : ARMS_E_TOPOLOGY;
            goto fail;
        }
        r->slaves_found = found;
        as->present = true;
        as->slaves_found = found;
        as->slaves_expected = r->count;
        as->master_index = r->ops->master_index(r->bus);
        snprintf(as->interface, sizeof(as->interface), "%s", ac->interface);
        logf_(a, "%s 臂: 发现 %d 个从站, 使用 %d 个", arms_arm_name(i), found, r->count);
        for (k = 0; k < r->count; ++k) {
            int pos = r->slot_slave[k];
            int jn = r->slot_joint[k];
            if (pos >= found) {
                set_error("%s 臂: 关节 %d (%s) 配置为 slave %d, 总线上只有 %d 个从站", arms_arm_name(i), jn,
                          a->cfg.joint[jn].name, pos, found);
                st = ARMS_E_TOPOLOGY;
                goto fail;
            }
            if (infos[pos].vendor_id != a->cfg.vendor_id || infos[pos].product_code != a->cfg.product_code) {
                set_error("%s 臂: slave %d 的厂商/产品号为 0x%08x/0x%08x, 配置要求 0x%08x/0x%08x",
                          arms_arm_name(i), pos, infos[pos].vendor_id, infos[pos].product_code, a->cfg.vendor_id,
                          a->cfg.product_code);
                st = ARMS_E_TOPOLOGY;
                goto fail;
            }
        }
        if (r->ops->configure(r->bus, r->slot_slave, r->count, err, sizeof(err)) != 0) {
            set_error("%s 臂: %s", arms_arm_name(i), err);
            st = ARMS_E_MASTER;
            goto fail;
        }
        r->wkc_expected = r->ops->expected_wkc(r->bus);
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
    if (a) stop_and_free(a);
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
        if (a->joints & ARMS_MASK_JOINT(j)) states[n++] = s.s[j];
    if (written) *written = n;
    return ARMS_OK;
}

arms_status arms_read_joint(arms_t *a, int joint, arms_joint_state *state)
{
    shared_snapshot s;
    if (!a || !state || joint < ARMS_JOINT_FIRST || joint > ARMS_JOINT_LAST) return ARMS_E_ARG;
    read_snapshot(a, &s);
    *state = s.s[joint];
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

arms_status arms_enable(arms_t *a, arms_mask joints)
{
    (void)a;
    (void)joints;
    set_error("使能功能暂不支持");
    return ARMS_E_UNSUPPORTED;
}

arms_status arms_disable(arms_t *a, arms_mask joints)
{
    (void)a;
    (void)joints;
    return ARMS_OK;
}
