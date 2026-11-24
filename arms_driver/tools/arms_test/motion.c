/** @file motion.c  @brief arms_test 轨迹控制、指令刷新与故障监视 */
#define _POSIX_C_SOURCE 200809L
#include "motion.h"

#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>
#include <sys/select.h>
#include <time.h>
#include <unistd.h>

#include "tlog.h"

#define NS_PER_S UINT64_C(1000000000)
#define REFRESH_NS UINT64_C(10000000)
#define SWING_NS UINT64_C(5000000)
#define TAU 6.28318530717958647693
#define MOVE_MAX_S 60.0

static arms_t *bus;
static volatile sig_atomic_t *interrupted;
static bool poll_input;
static pthread_t worker;
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static bool stopping;
static arms_mask refreshing;
static arms_command latest[ARMS_JOINT_LAST + 1];

typedef struct {
    arms_joint_state joints[ARMS_JOINT_COUNT];
    size_t count;
    arms_bus_stats stats;
} snapshot;

static snapshot previous;
static volatile bool power_off;

void motion_set_power_off(bool off) { power_off = off; }

static void (*trip_handler)(void);
static bool trip_seen;

void motion_set_trip_handler(void (*fn)(void)) { trip_handler = fn; }

static bool protect_seen;
static int take_snapshot(snapshot *s);

/* 超速/超力矩保护：驱动已失能全部关节，这里停止指令刷新并打印诊断快照 */
static void check_protect(const arms_bus_stats *st)
{
    size_t i;
    snapshot s;
    if (st->protect.reason == ARMS_PROTECT_NONE) {
        protect_seen = false;
        return;
    }
    if (protect_seen) return;
    protect_seen = true;
    pthread_mutex_lock(&lock);
    refreshing = 0;
    pthread_mutex_unlock(&lock);
    tlog_err("超限保护触发: 关节 %d(%s) %s %.4f 超过阈值 %.4f；全部关节已失能，执行 reset 清除",
             st->protect.joint, arms_joint_name(bus, st->protect.joint),
             st->protect.reason == ARMS_PROTECT_OVERSPEED ? "速度(rad/s)" : "力矩(N.m)", st->protect.value,
             st->protect.limit);
    if (!take_snapshot(&s)) return;
    for (i = 0; i < s.count; ++i) {
        const arms_joint_state *j = &s.joints[i];
        tlog_err("  %d %-8s pos=%.4f vel=%.4f tor=%.4f cmd=%.4f kp=%.2f kd=%.2f", j->joint,
                 arms_joint_mode_name(j->mode), j->position, j->velocity, j->torque, j->cmd_position, j->cmd_kp,
                 j->cmd_kd);
    }
}

/* 离线保护：驱动在周期内已失能全部关节，这里打印触发关节并调用下电处理 */
static void check_trip(void)
{
    arms_bus_stats st;
    char list[512];
    size_t len = 0;
    int j;
    if (arms_get_stats(bus, &st) != ARMS_OK) return;
    check_protect(&st);
    if (!st.tripped) {
        trip_seen = false;
        return;
    }
    if (trip_seen) return;
    trip_seen = true;
    pthread_mutex_lock(&lock);
    refreshing = 0;
    pthread_mutex_unlock(&lock);
    list[0] = '\0';
    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST && len < sizeof(list); ++j)
        if (st.trip_joints & ARMS_MASK_JOINT(j))
            len += (size_t)snprintf(list + len, sizeof(list) - len, " %d(%s)", j, arms_joint_name(bus, j));
    if (power_off) {
        tlog("电机已断电，以下关节离线:%s", list);
        return;
    }
    tlog_err("离线保护触发: 以下关节不在线:%s；全部关节已失能", list);
    if (trip_handler) trip_handler();
}

static uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * NS_PER_S + (uint64_t)ts.tv_nsec;
}

static struct timespec as_timespec(uint64_t ns)
{
    struct timespec ts = {(time_t)(ns / NS_PER_S), (long)(ns % NS_PER_S)};
    return ts;
}

static void sleep_until(uint64_t deadline)
{
    struct timespec ts = as_timespec(deadline);
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) == EINTR) {}
}

/* 保持绝对周期。延迟时跳过过期采样点，不补发旧目标。 */
static uint64_t next_tick(uint64_t last, uint64_t step)
{
    return last + ((now_ns() - last) / step + 1) * step;
}

static int wait_until(uint64_t deadline)
{
    while (!*interrupted) {
        uint64_t now = now_ns();
        if (now >= deadline) break;
        struct timespec left = as_timespec(deadline - now);
        fd_set fds;
        FD_ZERO(&fds);
        if (poll_input) FD_SET(STDIN_FILENO, &fds);
        int rc = pselect(poll_input ? STDIN_FILENO + 1 : 0, &fds, NULL, NULL, &left, NULL);
        if (rc > 0) {
            char key;
            ssize_t n = read(STDIN_FILENO, &key, 1);
            if (n == 1 && key == '\n') return 1;
            if (n == 0 || (n < 0 && errno != EINTR)) poll_input = false;
        } else if (rc < 0 && errno != EINTR) {
            tlog_err("等待中止输入失败: %s", strerror(errno));
            return 1;
        }
    }
    return *interrupted != 0;
}

int motion_wait_or_enter(unsigned int ms)
{
    return wait_until(now_ns() + (uint64_t)ms * 1000000);
}

arms_mask motion_enabled(void)
{
    arms_joint_state states[ARMS_JOINT_COUNT];
    arms_mask mask = 0;
    size_t n = 0;
    arms_read(bus, states, ARMS_JOINT_COUNT, &n);
    for (size_t i = 0; i < n; ++i)
        if (states[i].mode == ARMS_JS_ENABLED) mask |= ARMS_MASK_JOINT(states[i].joint);
    return mask;
}

static int take_snapshot(snapshot *s)
{
    return arms_read(bus, s->joints, ARMS_JOINT_COUNT, &s->count) == ARMS_OK &&
           arms_get_stats(bus, &s->stats) == ARMS_OK;
}

static void monitor_faults(void)
{
    snapshot s;
    bool report[ARMS_ARM_COUNT] = {false};
    if (!take_snapshot(&s)) return;
    /* 同一会话中，arms_read 始终按相同顺序返回全部已打开关节。 */
    for (size_t i = 0; i < s.count; ++i) {
        const arms_joint_state *j = &s.joints[i], *old = &previous.joints[i];
        /* 通信失败期间上电产生的同步丢失故障在恢复时自动复位，不单独报告 */
        if (j->fault_count > old->fault_count && !s.stats.arm[j->arm].comm_failed) {
            tlog_err("关节 %d 故障计数 %u -> %u", j->joint, old->fault_count, j->fault_count);
            report[j->arm] = true;
        }
        /* 主动失能后为 locked，不属于异常 */
        if (old->mode == ARMS_JS_ENABLED && j->mode != ARMS_JS_ENABLED && j->mode != ARMS_JS_LOCKED) {
            tlog_err("关节 %d 退出使能: %s", j->joint, arms_joint_mode_name(j->mode));
            report[j->arm] = true;
        }
    }
    for (int arm = 0; arm < ARMS_ARM_COUNT; ++arm) {
        const arms_arm_stats *a = &s.stats.arm[arm];
        if (!previous.stats.arm[arm].comm_failed && a->comm_failed) {
            if (power_off) {
                tlog("%s 臂通信中断（电机已断电）", arms_arm_name(arm));
                continue;
            }
            tlog_err("%s 臂通信失败，该臂关节已失能%s", arms_arm_name(arm),
                     arms_offline_power_off(bus) ? "" : "；电源或网线恢复后自动重连");
            report[arm] = true;
        }
        if (previous.stats.arm[arm].comm_failed && !a->comm_failed)
            tlog("%s 臂通信已恢复（累计中断 %u 次），关节保持失能，需重新 enable", arms_arm_name(arm),
                 a->comm_drops);
        if (!report[arm]) continue;
        for (size_t i = 0; i < s.count; ++i) {
            const arms_joint_state *j = &s.joints[i];
            if (j->arm == arm)
                tlog_err("  %d %-8s SW=0x%04x ERR=0x%04x pos=%.4f cmd=%.4f rad", j->joint,
                         arms_joint_mode_name(j->mode), j->status_word, j->error_code, j->position, j->cmd_position);
        }
        tlog_err("  %s WKC=%u/%u 异常=%llu 最长连续=%u comm_failed=%d", arms_arm_name(arm), a->wkc_last,
                 a->wkc_expected, (unsigned long long)a->wkc_bad, a->wkc_bad_consecutive_max, a->comm_failed);
    }
    previous = s;
}

static void *refresh_main(void *arg)
{
    uint64_t tick = now_ns(), diagnostic = tick;
    (void)arg;
    for (;;) {
        pthread_mutex_lock(&lock);
        if (stopping) {
            pthread_mutex_unlock(&lock);
            break;
        }
        for (int j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
            arms_mask bit = ARMS_MASK_JOINT(j);
            /* 驱动在每次写入时检查使能状态。检查和发送均在取消刷新的互斥区内。 */
            if ((refreshing & bit) && arms_set_command(bus, j, &latest[j], 0.0) != ARMS_OK) {
                refreshing &= ~bit;
                tlog_err("关节 %d 停止刷新: %s", j, arms_last_error());
            }
        }
        pthread_mutex_unlock(&lock);
        check_trip();
        if (tick >= diagnostic) {
            monitor_faults();
            diagnostic = next_tick(diagnostic, 10 * REFRESH_NS);
        }
        tick = next_tick(tick, REFRESH_NS);
        sleep_until(tick);
    }
    return NULL;
}

int motion_start(arms_t *arms, volatile sig_atomic_t *flag, bool batch)
{
    pthread_attr_t attr;
    struct sched_param normal = {0};
    int rc;
    stopping = false;
    bus = arms;
    interrupted = flag;
    /* 交互终端与 -e 会话接受回车。重定向的命令脚本不能被运动命令提前读取。 */
    poll_input = batch || isatty(STDIN_FILENO);
    if (!take_snapshot(&previous)) {
        tlog_err("无法读取故障监视的初始总线快照");
        return -1;
    }
    rc = pthread_attr_init(&attr);
    if (!rc) {
        rc = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
        if (!rc) rc = pthread_attr_setschedpolicy(&attr, SCHED_OTHER);
        if (!rc) rc = pthread_attr_setschedparam(&attr, &normal);
        if (!rc) rc = pthread_create(&worker, &attr, refresh_main, NULL);
        pthread_attr_destroy(&attr);
    }
    if (rc) tlog_err("无法启动指令刷新与监视线程: %s", strerror(rc));
    return rc ? -1 : 0;
}

void motion_stop(void)
{
    pthread_mutex_lock(&lock);
    stopping = true;
    refreshing = 0;
    pthread_mutex_unlock(&lock);
    pthread_join(worker, NULL);
}

void motion_cancel_refresh(arms_mask mask)
{
    pthread_mutex_lock(&lock);
    refreshing &= ~mask;
    pthread_mutex_unlock(&lock);
}

int motion_hold(arms_mask mask)
{
    motion_cancel_refresh(mask);
    uint64_t sent = now_ns(), deadline = sent + NS_PER_S;
    for (;;) {
        arms_joint_state states[ARMS_JOINT_COUNT];
        size_t n = 0;
        bool held = true;
        if (arms_hold(bus, mask) != ARMS_OK) {
            tlog_err("保持失败: %s", arms_last_error());
            return -1;
        }
        /* 请求与最后一条刷新可能在同一总线周期被取走；确认新反馈后才允许下一条控制命令。 */
        sleep_until(now_ns() + arms_period_ns(bus));
        if (arms_read(bus, states, ARMS_JOINT_COUNT, &n) != ARMS_OK) {
            tlog_err("无法读取保持反馈");
            return -1;
        }
        for (size_t i = 0; i < n; ++i) {
            const arms_joint_state *s = &states[i];
            if ((mask & ARMS_MASK_JOINT(s->joint)) && s->mode == ARMS_JS_ENABLED &&
                (s->sample_ns <= sent || s->cmd_moving || s->cmd_velocity != 0.0 || s->cmd_torque != 0.0)) held = false;
        }
        if (held) break;
        if (now_ns() >= deadline) {
            tlog_err("等待关节保持确认超时");
            return -1;
        }
    }
    return 0;
}

typedef struct {
    int joint;
    double center, move;
    arms_joint_limits limits;
    arms_command cmd;
} target;

typedef struct {
    target items[ARMS_JOINT_COUNT];
    size_t count;
    arms_mask mask;
} trajectory;

static int prepare(trajectory *p, arms_mask mask, arms_command cmd)
{
    arms_joint_state states[ARMS_JOINT_COUNT];
    size_t n = 0;
    p->mask = mask;
    p->count = 0;
    if (arms_read(bus, states, ARMS_JOINT_COUNT, &n) != ARMS_OK) return -1;
    for (size_t i = 0; i < n; ++i) {
        const arms_joint_state *s = &states[i];
        if (!(mask & ARMS_MASK_JOINT(s->joint))) continue;
        if (s->mode != ARMS_JS_ENABLED) {
            tlog_err("关节 %d 未使能，当前模式 %s；未下发指令", s->joint, arms_joint_mode_name(s->mode));
            return -1;
        }
        target *t = &p->items[p->count++];
        t->joint = s->joint;
        t->center = s->cmd_position;
        if (arms_get_limits(bus, t->joint, &t->limits) != ARMS_OK) {
            tlog_err("无法读取关节 %d 的限位", t->joint);
            return -1;
        }
        t->cmd = cmd;
        if (isnan(cmd.kp)) t->cmd.kp = t->limits.kp_default;
        if (isnan(cmd.kd)) t->cmd.kd = t->limits.kd_default;
    }
    if (!p->count) tlog_err("没有可执行运动的已使能关节");
    return p->count ? 0 : -1;
}

static int in_range(int joint, const char *name, double value, double min, double max)
{
    if (isfinite(value) && value >= min && value <= max) return 1;
    tlog_err("关节 %d: %s %.4f 超出范围 [%.4f, %.4f]；未下发指令", joint, name, value, min, max);
    return 0;
}

static int validate(const target *t, double amplitude, double speed)
{
    const arms_joint_limits *l = &t->limits;
    const arms_command *c = &t->cmd;
    return in_range(t->joint, "位置(rad)", c->position - amplitude, l->pos_min, l->pos_max) &&
           in_range(t->joint, "位置(rad)", c->position + amplitude, l->pos_min, l->pos_max) &&
           in_range(t->joint, "kp", c->kp, 0.0, l->kp_max) && in_range(t->joint, "kd", c->kd, 0.0, l->kd_max) &&
           in_range(t->joint, "速度(rad/s)", speed, -l->vel_max, l->vel_max) &&
           in_range(t->joint, "前馈(N.m)", c->torque, -l->tff_max, l->tff_max) &&
           in_range(t->joint, "过渡时间(s)", t->move, 0.0, MOVE_MAX_S);
}

static int send_target(const target *t, double move)
{
    if (arms_set_command(bus, t->joint, &t->cmd, move) == ARMS_OK) return 0;
    tlog_err("关节 %d 指令失败: %s", t->joint, arms_last_error());
    return -1;
}

static int submit(trajectory *p, bool refresh)
{
    motion_cancel_refresh(p->mask);
    for (size_t i = 0; i < p->count; ++i) {
        const target *t = &p->items[i];
        if (send_target(t, t->move)) {
            motion_hold(p->mask);
            return -1;
        }
    }
    if (refresh) {
        pthread_mutex_lock(&lock);
        for (size_t i = 0; i < p->count; ++i) latest[p->items[i].joint] = p->items[i].cmd;
        refreshing |= p->mask;
        pthread_mutex_unlock(&lock);
    }
    return 0;
}

static double transition(const target *t, double minimum)
{
    double distance = fabs(t->cmd.position - t->center);
    if (distance == 0.0) return minimum;
    if (t->limits.vel_max <= 0.0) return INFINITY;
    return fmax(minimum, distance / (0.5 * t->limits.vel_max));
}

int motion_mit(arms_mask mask, arms_command cmd, double move_s)
{
    trajectory p;
    bool dynamic = cmd.velocity != 0.0 || cmd.torque != 0.0;
    if (prepare(&p, mask, cmd)) return -1;
    for (size_t i = 0; i < p.count; ++i) {
        target *t = &p.items[i];
        t->move = dynamic ? 0.0 : (move_s < 0.0 ? transition(t, 0.5) : move_s);
        if (!validate(t, 0.0, cmd.velocity)) return -1;
    }
    if (submit(&p, dynamic)) return -1;
    /* 等待两个周期，使随后的 status 显示新指令 */
    sleep_until(now_ns() + 2 * arms_period_ns(bus) + 1000000);
    return 0;
}

/* 等到下发后的反馈确实到达目标，避免将上一周期的 cmd_moving=false 当作结束。 */
static int await_targets(const trajectory *p, double move, bool allow_abort, bool report)
{
    uint64_t sent = now_ns(), deadline = sent + (uint64_t)((move + 2.0) * NS_PER_S);
    arms_joint_state state[ARMS_JOINT_COUNT];
    for (;;) {
        bool done = true;
        for (size_t i = 0; i < p->count; ++i) {
            const target *t = &p->items[i];
            arms_joint_state *s = &state[i];
            if (arms_read_joint(bus, t->joint, s) != ARMS_OK || s->mode != ARMS_JS_ENABLED) {
                tlog_err("关节 %d 反馈不可用或已退出使能，停止运动", t->joint);
                motion_hold(p->mask);
                return -1;
            }
            if (s->sample_ns <= sent || s->cmd_moving || s->cmd_position != t->cmd.position) done = false;
        }
        if (done) break;
        if (now_ns() >= deadline) {
            tlog_err("等待轨迹结束超时，保持所选关节");
            motion_hold(p->mask);
            return -1;
        }
        if (!allow_abort) sleep_until(now_ns() + REFRESH_NS);
        else if (motion_wait_or_enter(10)) {
            tlog("运动已中止，保持当前指令位置");
            return motion_hold(p->mask);
        }
    }
    if (report) {
        for (size_t i = 0; i < p->count; ++i) {
            double goal = p->items[i].cmd.position, actual = state[i].position;
            tlog("关节 %d 目标 %.4f 实际 %.4f 误差(目标-实际) %+.4f rad", p->items[i].joint, goal, actual, goal - actual);
        }
    }
    return 0;
}

int motion_home(arms_mask mask, double move_s)
{
    arms_command cmd = {.kp = NAN, .kd = NAN};
    trajectory p;
    double common = move_s < 0.0 ? 1.0 : move_s;
    if (prepare(&p, mask, cmd)) return -1;
    for (size_t i = 0; i < p.count; ++i) {
        target *t = &p.items[i];
        t->cmd.position = t->limits.home;
        if (move_s < 0.0) common = fmax(common, transition(t, 1.0));
    }
    for (size_t i = 0; i < p.count; ++i) {
        p.items[i].move = common;
        if (!validate(&p.items[i], 0.0, 0.0)) return -1;
    }
    tlog("回零: 过渡 %.4f s；回车或 Ctrl+C 中止", common);
    if (submit(&p, false)) return -1;
    return await_targets(&p, common, true, true);
}

int motion_swing(arms_mask mask, double amplitude, double period, long count, double kp, double kd)
{
    arms_command cmd = {.kp = kp, .kd = kd};
    trajectory p;
    double omega = TAU / period, peak = amplitude * omega, duration = period * count;
    double max_error = 0.0, next_report = 1.0;
    if (!isfinite(duration) || !isfinite(omega)) {
        tlog_err("摆动周期或总时长超出范围");
        return -1;
    }
    if (prepare(&p, mask, cmd)) return -1;
    for (size_t i = 0; i < p.count; ++i) {
        target *t = &p.items[i];
        t->cmd.position = t->center;
        t->move = 0.0;
        if (!validate(t, amplitude, peak)) return -1;
    }
    motion_cancel_refresh(mask);
    tlog("正弦摆动: 幅值 %.4f rad 周期 %.4f s，共 %ld 次；回车或 Ctrl+C 中止", amplitude, period, count);
    uint64_t start = now_ns(), tick = start;
    bool aborted = false;
    for (;;) {
        double elapsed = (double)(now_ns() - start) / NS_PER_S;
        if (elapsed >= next_report) {
            tlog("摆动 %.4f s  最大跟踪误差 %.4f rad", elapsed, max_error);
            next_report = floor(elapsed) + 1.0;
            max_error = 0.0;
        }
        if (elapsed >= duration) break;
        if (*interrupted) {
            aborted = true;
            break;
        }
        double phase = TAU * (fmod(elapsed, period) / period);
        double offset = amplitude * sin(phase), velocity = peak * cos(phase);
        for (size_t i = 0; i < p.count; ++i) {
            target *t = &p.items[i];
            arms_joint_state s;
            t->cmd.position = t->center + offset;
            t->cmd.velocity = velocity;
            if (send_target(t, 0.0) || arms_read_joint(bus, t->joint, &s) != ARMS_OK) {
                motion_hold(mask);
                return -1;
            }
            max_error = fmax(max_error, fabs(t->cmd.position - s.position));
        }
        tick = next_tick(tick, SWING_NS);
        if (wait_until(tick)) {
            aborted = true;
            break;
        }
    }
    double move = aborted ? 1.0 : 0.0;
    if (aborted) tlog("摆动已中止，以 1.0000 s 过渡返回中心");
    for (size_t i = 0; i < p.count; ++i) {
        p.items[i].cmd.position = p.items[i].center;
        p.items[i].cmd.velocity = 0.0;
        p.items[i].move = move;
    }
    if (submit(&p, false)) return -1;
    return await_targets(&p, move, false, false);
}
