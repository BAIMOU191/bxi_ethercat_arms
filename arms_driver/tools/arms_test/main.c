/**
 * @file main.c
 * @brief arms_test：双臂 EtherCAT 驱动测试工具
 *
 * 命令统一登记在 commands[] 中，help、"命令 -h" 和参数个数检查均由该表生成。
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <limits.h>
#include <math.h>
#include <signal.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/select.h>
#include <sys/wait.h>
#include <spawn.h>
#include <time.h>
#include <unistd.h>

#include "arms/arms.h"
#include "arms/pn.h"
#include "arms/lease.h"
#include "joint_select.h"
#include "lineedit.h"
#include "motion.h"
#include "power.h"
#include "runtime_env.h"
#include "tlog.h"

static volatile sig_atomic_t g_interrupted, g_terminate;
static int stop_pipe[2];
static arms_t *g_arms;
static atomic_bool powered, power_known;
static bool mock_backend;
static pthread_mutex_t power_lock = PTHREAD_MUTEX_INITIALIZER;
static bool g_stamp_all; /* -T：每行加时间戳，输出经过管道 */
static unsigned int power_wait = 15000; /* 上电后等待总线稳定的最长时间 [ms] */

static void on_sigint(int sig)
{
    int saved_errno = errno;
    if (sig == SIGTERM) {
        g_terminate = 1;
        ssize_t n = write(stop_pipe[1], "!", 1);
        (void)n;
    }
    g_interrupted = 1;
    errno = saved_errno;
}

static void log_line(void *user, const char *line)
{
    (void)user;
    tlog("  %s", line);
}

typedef int (*cmd_fn)(int argc, char **argv);

typedef struct {
    const char *name;
    const char *group;
    const char *args;     /* 命令名之后的参数说明 */
    int min_args, max_args;
    const char *brief;
    const char *detail;   /* 详细说明，可多行 */
    const char *example;
    cmd_fn fn;
} command;

static const command *find_command(const char *name);
static int ok(void) { printf("OK\n"); return 0; }
static int failed(const char *fmt, ...)
{
    va_list ap;
    printf("FAILED: ");
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
    return -1;
}

static void print_usage(const command *c)
{
    printf("用法: %s%s%s\n", c->name, c->args[0] ? " " : "", c->args);
}

static void print_detail(const command *c)
{
    print_usage(c);
    printf("  %s\n", c->brief);
    if (c->detail && c->detail[0]) printf("%s\n", c->detail);
    if (c->example && c->example[0]) printf("示例:\n%s\n", c->example);
}

static int need_session(void)
{
    if (!g_arms) {
        failed("总线未打开");
        return -1;
    }
    return 0;
}

static unsigned int hz_short(unsigned int hz, char *buf, size_t n)
{
    if (hz % 1000 == 0) snprintf(buf, n, "%ukHz", hz / 1000);
    else snprintf(buf, n, "%uHz", hz);
    return hz;
}

/* ---- status ---- */
static int cmd_status(int argc, char **argv)
{
    arms_mask known, sel;
    char err[256];
    int j;
    if (need_session()) return -1;
    known = arms_opened_joints(g_arms) | arms_ignored_joints(g_arms) | arms_missing_joints(g_arms);
    sel = known;
    bool raw = false;
    const char *selector = NULL;
    for (int k = 1; k < argc; k++) {
        if (!strcmp(argv[k], "--raw") && !raw)
            raw = true;
        else if (!selector)
            selector = argv[k];
        else
            return -2;
    }
    if (selector && arms_parse_joints(selector, known, &sel, err, sizeof(err)))
        return failed("%s", err);
    printf("%-3s %-20s %-11s %-8s %-6s %-6s %-6s %10s %10s %9s %10s %8s %8s %4s %5s %5s\n", "关节", "名称", "臂/位置", "状态",
           "AL", "SW", "ERR", "pos[rad]", "vel[rad/s]", "tor[N.m]", "cmd", "kp", "kd", "flag", "flt", "温度℃");
    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
        arms_joint_state s;
        char where[16], flags[4];
        int n = 0;
        if (!(sel & ARMS_MASK_JOINT(j))) continue;
        if (!(arms_opened_joints(g_arms) & ARMS_MASK_JOINT(j))) {
            printf("%-3d %-20s %-11s %-8s\n", j, arms_joint_name(g_arms, j), "-",
                   (arms_ignored_joints(g_arms) & ARMS_MASK_JOINT(j)) ? "ignored" : "missing");
            continue;
        }
        arms_read_joint(g_arms, j, &s);
        if (raw) {
            printf("%d TxPDO:", j);
            for (size_t k = 0; k < sizeof(s.txpdo); k++)
                printf(" %02x", s.txpdo[k]);
            printf("  RxPDO:");
            for (size_t k = 0; k < sizeof(s.rxpdo); k++)
                printf(" %02x", s.rxpdo[k]);
            puts("");
            continue;
        }
        snprintf(where, sizeof(where), "%s/%d", s.arm == 0 ? "L" : "R", s.slave);
        if (s.cmd_pos_blocked) flags[n++] = 'P';
        if (s.cmd_timeout) flags[n++] = 'T';
        if (!n) flags[n++] = '-';
        flags[n] = '\0';
        {
            char temp[16];
            if (isnan(s.temperature)) snprintf(temp, sizeof(temp), "-");
            else snprintf(temp, sizeof(temp), "%.0f", s.temperature);
            printf("%-3d %-20s %-11s %-8s 0x%02x   0x%04x 0x%04x %10.4f %10.4f %9.4f %10.4f %8.4f %8.4f %4s %5u %5s\n", j,
                   s.name ? s.name : "-", where, arms_joint_mode_name(s.mode), s.al_state, s.status_word, s.error_code,
                   s.position, s.velocity, s.torque, s.cmd_position, s.cmd_kp, s.cmd_kd, flags, s.fault_count, temp);
        }
    }
    return ok();
}

/* ---- bus ---- */
static void print_stats(const arms_bus_stats *st)
{
    int i;
    printf("周期 %u Hz  已运行 %llu 周期  平均 %.4f us  最短 %.4f  最长 %.4f  抖动RMS %.4f us\n", st->cycle_hz,
           (unsigned long long)st->cycles, st->period.mean, st->period.min, st->period.max, st->period.rms_jitter);
    printf("唤醒最大延迟 %.4f us  单周期最大耗时 %.4f us  超时(>1.5T) %llu  跳过周期 %llu\n", st->wake_late_us_max,
           st->work_us_max, (unsigned long long)st->overruns, (unsigned long long)st->skipped);
    for (i = 0; i < ARMS_ARM_COUNT; ++i) {
        const arms_arm_stats *a = &st->arm[i];
        if (!a->present) {
            printf("%-5s 未打开\n", arms_arm_name(i));
            continue;
        }
        printf("%-5s %s 主站%d 链路%s  从站 OP %d/%d(总线上 %d)  WKC %u/%u  异常 %llu (最长连续 %u)  发送错误 %llu"
               "  收 %.4f us 发 %.4f us%s\n",
               arms_arm_name(i), a->interface, a->master_index, a->link_up ? "UP" : "DOWN", a->slaves_op,
               a->slaves_expected, a->slaves_found, a->wkc_last, a->wkc_expected, (unsigned long long)a->wkc_bad,
               a->wkc_bad_consecutive_max, (unsigned long long)a->send_errors, a->recv_us_max, a->send_us_max,
               a->comm_failed ? "  [通信失败]" : "");
        if (a->comm_drops || a->comm_recoveries)
            printf("      通信中断 %u 次，自动恢复 %u 次\n", a->comm_drops, a->comm_recoveries);
    }
}

static void print_mask_line(const char *label, arms_mask m)
{
    int j;
    printf("%s 0x%08x", label, m);
    if (m) {
        printf("  [");
        for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j)
            if (m & ARMS_MASK_JOINT(j)) printf(" %d", j);
        printf(" ]");
    }
    printf("\n");
}

static void print_protection(const arms_bus_stats *st)
{
    printf("启动检查: %s\n", arms_startup_check_enabled(g_arms) ? "开启" : "关闭");
    print_mask_line("屏蔽掩码", arms_ignored_joints(g_arms));
    if (arms_missing_joints(g_arms)) print_mask_line("启动时不在线（未使用）", arms_missing_joints(g_arms));
    printf("离线处理: %s", arms_offline_power_off(g_arms) ? "下电保护" : "失能并自动重连");
    if (st->tripped) print_mask_line("  [保护已触发] 触发关节", st->trip_joints);
    else printf("\n");
    if (st->offline_joints) print_mask_line("当前不在线", st->offline_joints);
    if (st->protect.reason != ARMS_PROTECT_NONE)
        printf("超限保护: [已触发] 关节 %d %s %.4f 超过阈值 %.4f，执行 reset 清除\n", st->protect.joint,
               st->protect.reason == ARMS_PROTECT_OVERSPEED    ? "速度(rad/s)"
               : st->protect.reason == ARMS_PROTECT_OVERTORQUE ? "力矩(N.m)"
                                                                : "温度(℃)",
               st->protect.value, st->protect.limit);
    else
        printf("超限保护: 正常\n");
}

/* ---- check ---- */
static int cmd_check(int argc, char **argv)
{
    arms_bus_stats st;
    arms_mask off = 0;
    (void)argc;
    (void)argv;
    if (need_session()) return -1;
    arms_get_stats(g_arms, &st);
    print_protection(&st);
    if (arms_check_online(g_arms, &off) != ARMS_OK) return failed("%s", arms_last_error());
    printf("已打开的 %d 个关节全部在线\n", __builtin_popcount(arms_opened_joints(g_arms)));
    return ok();
}

static int cmd_bus(int argc, char **argv)
{
    arms_bus_stats st;
    if (need_session()) return -1;
    if (argc > 1) {
        if (strcmp(argv[1], "reset") != 0) return failed("未知参数 %s（只支持 reset）", argv[1]);
        arms_reset_stats(g_arms);
        printf("统计已清零\n");
        return ok();
    }
    arms_get_stats(g_arms, &st);
    print_stats(&st);
    print_protection(&st);
    return ok();
}

/* ---- 控制命令的公共参数 ---- */
static int bad_args(const char *reason)
{
    failed("%s", reason);
    return -2; /* 由命令分派器打印用法 */
}

static int selection(const char *text, arms_mask *mask)
{
    char err[256];
    *mask = arms_opened_joints(g_arms);
    if (text && arms_parse_joints(text, *mask, mask, err, sizeof(err)) != 0) return bad_args(err);
    if (!*mask) return bad_args("未选中已打开的关节");
    return 0;
}

static bool number(const char *s, double *v)
{
    char *end;
    errno = 0;
    *v = strtod(s, &end);
    return end != s && !*end && !errno && isfinite(*v);
}

static bool integer(const char *s, long lo, long hi, long *v)
{
    char *end;
    errno = 0;
    *v = strtol(s, &end, 10);
    return end != s && !*end && !errno && *v >= lo && *v <= hi;
}

static void delay_ms(unsigned int ms)
{
    struct timespec ts = {ms / 1000, (long)(ms % 1000) * 1000000L};
    while (nanosleep(&ts, &ts) != 0 && errno == EINTR) {}
}

static void show_modes(arms_mask mask, bool with_position)
{
    for (int j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
        arms_joint_state s;
        if (!(mask & ARMS_MASK_JOINT(j))) continue;
        arms_read_joint(g_arms, j, &s);
        printf("%d %-8s", j, arms_joint_mode_name(s.mode));
        if (with_position) printf(" 位置 %.4f rad", s.position);
        putchar('\n');
    }
}

static int disable_joints(arms_mask mask, unsigned int wait_ms)
{
    motion_cancel_refresh(mask);
    if (arms_disable(g_arms, mask) != ARMS_OK) return failed("失能失败: %s", arms_last_error());
    delay_ms(wait_ms);
    return 0;
}

static int cmd_enable(int argc, char **argv)
{
    arms_mask mask;
    long ms = 0;
    if (selection(argv[1], &mask)) return -2;
    if (argc != 2 && (argc != 4 || strcmp(argv[2], "-t") || !integer(argv[3], 0, INT_MAX, &ms)))
        return bad_args("-t 需要非负整数毫秒数");
    if (arms_enable(g_arms, mask, (unsigned int)ms) != ARMS_OK) return failed("使能失败: %s", arms_last_error());
    show_modes(mask, true);
    tlog("使能后指令为零（kp=0、kd=0），关节无保持力矩");
    return ok();
}

static int cmd_disable(int argc, char **argv)
{
    arms_mask mask;
    if (selection(argc > 1 ? argv[1] : NULL, &mask)) return -2;
    if (disable_joints(mask, 20)) return -1;
    show_modes(mask, false);
    return ok();
}

static int cmd_reset(int argc, char **argv)
{
    arms_mask mask;
    (void)argc;
    if (selection(argv[1], &mask)) return -2;
    motion_cancel_refresh(mask);
    if (arms_reset(g_arms, mask, 0) != ARMS_OK) return failed("复位失败: %s", arms_last_error());
    show_modes(mask, false);
    return ok();
}

static int cmd_hold(int argc, char **argv)
{
    arms_mask mask;
    if (selection(argc > 1 ? argv[1] : NULL, &mask)) return -2;
    return motion_hold(mask) ? -1 : ok();
}

static int cmd_mit(int argc, char **argv)
{
    arms_mask mask;
    arms_command c = {0};
    double *fields[] = {&c.position, &c.kp, &c.kd, &c.velocity, &c.torque};
    if (argc < 5 || argc > 7) return bad_args("参数: <关节> <位置> <kp> <kd> [速度] [前馈]");
    if (selection(argv[1], &mask)) return -2;
    for (int i = 2; i < argc; ++i)
        if (!number(argv[i], fields[i - 2])) return bad_args("运动参数必须是有限数值");
    return motion_mit(mask, c) ? -2 : ok();
}

static int cmd_swing(int argc, char **argv)
{
    arms_mask mask;
    double amplitude, period, kp, kd;
    long count;
    if (selection(argv[1], &mask)) return -2;
    if (!number(argv[2], &amplitude) || amplitude < 0.0) return bad_args("幅值必须为非负有限数值");
    if (!number(argv[3], &period) || period <= 0.0) return bad_args("周期必须为正数");
    if (!integer(argv[4], 1, LONG_MAX, &count)) return bad_args("次数必须为正整数");
    if (!number(argv[5], &kp)) return bad_args("kp 必须是有限数值");
    if (!number(argv[6], &kd)) return bad_args("kd 必须是有限数值");
    return motion_swing(mask, amplitude, period, count, kp, kd) ? -2 : ok();
}

/* 等待已打开的臂全部恢复通信且从站进入 OP；回车或 Ctrl+C 中止 */
static int wait_bus_online(unsigned int timeout_ms)
{
    unsigned int waited;
    for (waited = 0; waited < timeout_ms; waited += 100) {
        arms_bus_stats st;
        bool online = true;
        int i;
        arms_get_stats(g_arms, &st);
        for (i = 0; i < ARMS_ARM_COUNT; ++i) {
            const arms_arm_stats *a = &st.arm[i];
            if (a->present && (a->comm_failed || a->slaves_op != a->slaves_expected || a->wkc_last != a->wkc_expected))
                online = false;
        }
        if (online) return 0;
        if (motion_wait_or_enter(100)) return -1;
    }
    return -1;
}

/* 命令线程与掉线监视线程共用 PCI 句柄；锁只保护 SDK 调用，不包含总线恢复等待 */
static int request_power(bool on)
{
    int rc;
    pthread_mutex_lock(&power_lock);
    if (on && powered && power_known) rc = 0;
    else rc = on ? arms_power_on(0) : arms_power_off();
    if (rc == 0) {
        powered = on;
        motion_set_power_off(!on);
    }
    power_known = rc == 0;
    pthread_mutex_unlock(&power_lock);
    return rc;
}

static int cmd_power(int argc, char **argv)
{
    const char *op = argv[1];
    (void)argc;
    if (strcmp(op, "on") && strcmp(op, "off") && strcmp(op, "status")) return bad_args("未知电源操作");
    if (!arms_power_available()) return failed("本程序不支持 PCI 电源控制");
    if (mock_backend) return failed("模拟后端不访问 PCI 电源");
    if (!strcmp(op, "status")) {
        printf("电源: %s（本程序的请求状态，非硬件回读）\n", !power_known ? "状态未知" : powered ? "已请求上电" : "已请求断电");
    } else if (!strcmp(op, "on")) {
        if (request_power(true) != 0) {
            return failed("上电失败");
        }
        if (wait_bus_online(30000) != 0 || arms_check_online(g_arms, NULL) != ARMS_OK) {
            /* 在线检查未通过：重新下电保护 */
            char why[512];
            arms_mask off = 0;
            arms_check_online(g_arms, &off);
            snprintf(why, sizeof(why), "%s", off ? arms_last_error() : "通信在 30 秒内未恢复");
            if (request_power(false) == 0) {
                return failed("上电后在线检查未通过，已重新下电。%s", why);
            }
            return failed("上电后在线检查未通过，下电失败，需手动断电。%s", why);
        }
        if (arms_clear_trip(g_arms) != ARMS_OK) return failed("离线保护未能清除: %s", arms_last_error());
        tlog("在线检查通过，关节保持失能");
    } else {
        if (!powered && !power_known) return failed("未接管电源，无法确认或切断外部供电");
        if (motion_enabled() && disable_joints(arms_opened_joints(g_arms), 50)) return -1;
        if (request_power(false) != 0) {
            return failed("断电失败，电源状态未知");
        }
    }
    return ok();
}

/* ---- commtest ---- */
/* print_diag() 的返回位：出现线路类计数、只有处理类计数、驱动器丢帧增加 */
enum { DIAG_LINE = 1, DIAG_PROC = 2, DIAG_LOSS = 4 };

/*
 * 打印驱动器丢帧次数和 ESC 错误计数，线路类（无效帧、物理层错误、链路丢失）与处理类
 * （转发错误、处理单元错误、PDI 错误）分列。only_nonzero 为真时只列出存在计数或驱动器丢帧次数
 * 相对 loss_base（按关节号索引，可为 NULL）增加的关节。返回 DIAG_* 位的组合。
 */
static int print_diag(arms_mask mask, bool only_nonzero, const int32_t *loss_base)
{
    int found = 0;
    bool header = false;
    for (int j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
        arms_joint_state s;
        char where[16], loss[16];
        const uint8_t *e;
        int flags = 0;
        if (!(mask & ARMS_MASK_JOINT(j)) || arms_read_joint(g_arms, j, &s) != ARMS_OK) continue;
        e = s.esc_errors;
        for (int b = 0; b < 20; ++b) {
            if (b == 14 || b == 15 || !e[b]) continue;
            flags |= (b < 8 || b >= 16) ? DIAG_LINE : DIAG_PROC;
        }
        if (s.drive_frame_loss > (loss_base && loss_base[j] >= 0 ? loss_base[j] : 0)) flags |= DIAG_LOSS;
        if (only_nonzero && !flags) continue;
        found |= flags;
        if (!header) {
            printf("%-3s %-20s %-7s %9s  %-24s  %-24s  %s\n", "关节", "名称", "臂/位置", "驱动器丢帧",
                   "线路类 端口0 无效/物理/丢链", "线路类 端口1 无效/物理/丢链", "处理类 转发0/转发1/处理单元/PDI");
            header = true;
        }
        snprintf(where, sizeof(where), "%s/%d", s.arm == 0 ? "L" : "R", s.slave);
        if (s.drive_frame_loss < 0) snprintf(loss, sizeof(loss), "-");
        else snprintf(loss, sizeof(loss), "%d", s.drive_frame_loss);
        if (!s.esc_valid) {
            printf("%-3d %-20s %-7s %9s  %s\n", j, s.name ? s.name : "-", where, loss, "ESC 计数尚未读到");
        } else {
            printf("%-3d %-20s %-7s %9s  %4u %4u %4u               %4u %4u %4u               %4u %4u %4u %4u\n", j,
                   s.name ? s.name : "-", where, loss, e[0], e[1], e[16], e[2], e[3], e[17], e[8], e[9], e[12],
                   e[13]);
        }
    }
    return found;
}

/* diag：驱动器丢帧次数和各从站 ESC 错误计数 */
static int cmd_diag(int argc, char **argv)
{
    arms_mask m = arms_opened_joints(g_arms);
    const char *selector = NULL;
    bool reset = false;
    for (int k = 1; k < argc; ++k) {
        if (!strcmp(argv[k], "reset") && !reset) reset = true;
        else if (!selector) selector = argv[k];
        else return bad_args("参数为可选的关节和 reset");
    }
    if (selector && selection(selector, &m)) return -2;
    if (reset) {
        arms_clear_esc_counters(g_arms);
        tlog("已请求清零全部从站的 ESC 错误计数，约 1 秒内生效");
        return ok();
    }
    print_diag(m, false, NULL);
    return ok();
}

static int cmd_commtest(int argc, char **argv)
{
    long seconds;
    arms_bus_stats st;
    arms_mask all = arms_opened_joints(g_arms);
    bool enabled = argc == 5, pass = true;
    const char *label = enabled ? "伺服使能，" : "";
    int32_t loss_base[ARMS_JOINT_LAST + 1];
    double hold_kp = 0.0, hold_kd = 0.0;
    long t;
    int i;
    if (!integer(argv[1], 1, 86400, &seconds)) return bad_args("秒数必须是 1..86400 的整数");
    if (argc != 2 && (argc != 5 || strcmp(argv[2], "--enabled") || !number(argv[3], &hold_kp) ||
                      !number(argv[4], &hold_kd)))
        return bad_args("可选参数: --enabled <kp> <kd>");
    if (!enabled && motion_enabled()) return failed("关节须全部失能；使能状态下测试使用 --enabled <kp> <kd>");
    if (enabled) {
        if (motion_hold(all)) goto enable_error;
        if (arms_enable(g_arms, all, 0) != ARMS_OK) {
            failed("伺服使能失败: %s", arms_last_error());
            goto enable_error;
        }
        /* 使能后指令为零，立即以给定增益保持当前位置 */
        for (int j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
            arms_joint_state s;
            if (!(all & ARMS_MASK_JOINT(j))) continue;
            if (arms_read_joint(g_arms, j, &s) != ARMS_OK) {
                failed("无法读取关节 %d 的当前位置", j);
                goto enable_error;
            }
            arms_command cmd = {.position = s.position, .kp = hold_kp, .kd = hold_kd};
            if (arms_set_command(g_arms, j, &cmd) != ARMS_OK) {
                failed("关节 %d 无法保持原位: %s", j, arms_last_error());
                goto enable_error;
            }
        }
    }
    arms_reset_stats(g_arms);
    arms_clear_esc_counters(g_arms);
    for (int j = 0; j <= ARMS_JOINT_LAST; ++j) {
        arms_joint_state s;
        loss_base[j] = arms_read_joint(g_arms, j, &s) == ARMS_OK ? s.drive_frame_loss : -1;
    }
    delay_ms(300); /* 等待各从站完成 ESC 错误计数清零 */
    tlog("通信测试开始: %s%ld 秒，回车或 Ctrl+C 提前结束（ESC 错误计数已清零）", label, seconds);
    for (t = 1; t <= seconds; ++t) {
        if (motion_wait_or_enter(1000)) {
            tlog("通信测试提前结束: 已运行 %ld 秒", t - 1);
            break;
        }
        if (enabled && motion_enabled() != all) {
            tlog_err("伺服使能状态丢失，终止通信测试");
            pass = false;
            break;
        }
        if (t % 10 == 0 || t == seconds) {
            arms_get_stats(g_arms, &st);
            tlog("  %3ld s  周期 %llu  WKC异常次数 左 %llu 右 %llu  周期最长 %.4f us", t, (unsigned long long)st.cycles,
                 (unsigned long long)st.arm[0].wkc_bad, (unsigned long long)st.arm[1].wkc_bad, st.period.max);
        }
    }
    tlog("通信测试结束");
    arms_get_stats(g_arms, &st);
    if (enabled) {
        if (motion_enabled() != all) pass = false;
        if (disable_joints(all, 20)) return -1;
    }
    print_stats(&st);
    {
        int found = print_diag(all, true, loss_base);
        if (found & (DIAG_LINE | DIAG_LOSS)) {
            printf("以上关节在测试期间出现线路类 ESC 错误计数或驱动器丢帧，明细见 diag 和通信事件日志\n");
            pass = false;
        } else if (found & DIAG_PROC) {
            printf("提示: 存在处理类 ESC 错误计数，线路类计数为 0，不计入测试结果\n");
        }
    }
    for (i = 0; i < ARMS_ARM_COUNT; ++i)
        if (st.arm[i].present && (st.arm[i].wkc_bad || st.arm[i].send_errors || st.arm[i].comm_failed)) pass = false;
    if (st.overruns || st.skipped) pass = false;
    if (pass) {
        printf("结果: 通过（%s无 WKC 异常、无发送错误、无超时周期）\n", label);
        return ok();
    }
    return failed("通信测试未通过（%s见上方统计）", label);

enable_error:
    disable_joints(all, 20);
    return -1;
}

/* ---- watch ---- */
static int wait_or_enter(unsigned int ms)
{
    fd_set rfds;
    struct timeval tv;
    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);
    tv.tv_sec = ms / 1000;
    tv.tv_usec = (ms % 1000) * 1000;
    if (select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv) > 0) {
        char line[64];
        if (fgets(line, sizeof(line), stdin) == NULL) return 1;
        return 1;
    }
    return 0;
}

static int cmd_watch(int argc, char **argv)
{
    arms_mask sel = arms_opened_joints(g_arms);
    char err[256];
    long hz = 5;
    if (need_session()) return -1;
    if (argc > 1 && arms_parse_joints(argv[1], arms_opened_joints(g_arms), &sel, err, sizeof(err)) != 0)
        return failed("%s", err);
    if (argc > 2) {
        char *end;
        hz = strtol(argv[2], &end, 10);
        if (*end || hz < 1 || hz > 50) return failed("刷新频率必须是 1..50 Hz");
    }
    printf("持续刷新 %ld Hz，回车或 Ctrl+C 结束\n", hz);
    g_interrupted = 0;
    {
        /* 每个关节一行，原地刷新：第二次起先把光标移回表格首行。-T 模式输出经过时间戳管道，改为逐次打印 */
        bool in_place = !g_stamp_all && isatty(STDOUT_FILENO);
        const char *clear = in_place ? "\r\x1b[2K" : "";
        int rows = 0;
        for (;;) {
            arms_joint_state states[ARMS_JOINT_COUNT];
            size_t count = 0;
            if (arms_read(g_arms, states, ARMS_JOINT_COUNT, &count) != ARMS_OK)
                return failed("读取关节快照失败");
            if (in_place && rows > 0) printf("\x1b[%dA", rows);
            rows = 0;
            printf("%s%-4s %-20s %-8s %10s %10s %9s %10s\n", clear, "关节", "名称", "状态", "pos[rad]", "vel[rad/s]",
                   "tor[N.m]", "cmd");
            rows++;
            for (size_t i = 0; i < count; ++i) {
                arms_joint_state s = states[i];
                if (!(sel & ARMS_MASK_JOINT(s.joint))) continue;
                printf("%s%-4d %-20s %-8s %10.4f %10.4f %9.4f %10.4f\n", clear, s.joint, s.name ? s.name : "-",
                       arms_joint_mode_name(s.mode), s.position, s.velocity, s.torque, s.cmd_position);
                rows++;
            }
            fflush(stdout);
            if (g_terminate || wait_or_enter((unsigned int)(1000 / hz)) || g_interrupted) break;
            if (!in_place) printf("\n");
        }
    }
    return ok();
}

/* ---- info ---- */
static int cmd_info(int argc, char **argv)
{
    char hz[16];
    int j;
    (void)argc;
    (void)argv;
    if (need_session()) return -1;
    hz_short(arms_cycle_hz(g_arms), hz, sizeof(hz));
    printf("版本     %s\n配置     %s\n后端     %s\n周期     %s\n", arms_version(), arms_config_path(g_arms),
           arms_backend_name(g_arms), hz);
    if (arms_rt_cpu(g_arms) >= 0) printf("实时 CPU %d\n", arms_rt_cpu(g_arms));
    else printf("实时 CPU 未绑定\n");
    arms_mask sel;
    if (selection(argc > 1 ? argv[1] : NULL, &sel))
        return -2;
    printf("关节映射:\n");
    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
        arms_joint_state s;
        if (!(sel & ARMS_MASK_JOINT(j)))
            continue;
        arms_read_joint(g_arms, j, &s);
        printf("  %d %-20s %s 臂 slave %d\n", j, s.name, arms_arm_name(s.arm), s.slave);
        if (argc > 1) {
            arms_joint_info q;
            arms_joint_limits l;
            arms_get_joint_info(g_arms, j, &q);
            arms_get_limits(g_arms, j, &l);
            printf("    方向 %+d 偏置 %.6f rad\n", q.direction, q.offset_rad);
            printf("    软位置限位 pos_limit [%.3f, %.3f] rad\n",
                   l.pos_limit_min, l.pos_limit_max);
            printf("    反馈保护: vel_limit %.3f rad/s  tor_limit %.3f N.m\n", l.vel_limit, l.tor_limit);
        }
    }
    return ok();
}

static int cmd_version(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    printf("arms_test %s\n", arms_version());
    return ok();
}

static int g_quit;
static int cmd_quit(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    g_quit = 1;
    return 0;
}

static int cmd_selftest(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (arms_check_online(g_arms, NULL) != ARMS_OK)
        return failed("%s", arms_last_error());
    arms_bus_stats st;
    arms_get_stats(g_arms, &st);
    if (st.tripped)
        return failed("离线保护已锁定");
    if (st.protect.reason != ARMS_PROTECT_NONE)
        return failed("超速/超力矩/过温保护已锁定（关节 %d），执行 reset 清除", st.protect.joint);
    for (int j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; j++)
        if (arms_opened_joints(g_arms) & ARMS_MASK_JOINT(j)) {
            arms_joint_state s;
            arms_read_joint(g_arms, j, &s);
            if (s.error_code || s.mode == ARMS_JS_FAULT)
                return failed("关节 %d 故障 0x%04x", j, s.error_code);
        }
    return ok();
}

/* 关节掩码转为 " 15 16 21" 形式 */
static const char *mask_list(arms_mask m, char *buf, size_t size)
{
    size_t len = 0;
    buf[0] = '\0';
    for (int j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST && len < size; j++)
        if (m & ARMS_MASK_JOINT(j)) len += (size_t)snprintf(buf + len, size - len, " %d", j);
    return buf;
}

/* 启动自检：在线检查已在打开时完成；驱动器残留故障先自动复位一次，仍有故障只告警 */
static void startup_selftest(void)
{
    arms_mask faulted = 0;
    for (int j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; j++)
        if (arms_opened_joints(g_arms) & ARMS_MASK_JOINT(j)) {
            arms_joint_state s;
            arms_read_joint(g_arms, j, &s);
            if (s.mode == ARMS_JS_FAULT || s.error_code) faulted |= ARMS_MASK_JOINT(j);
        }
    if (faulted) {
        tlog("自检: 发现驱动器故障，自动复位一次");
        if (arms_reset(g_arms, faulted, 0) != ARMS_OK)
            tlog_err("自检: 复位未完成: %s", arms_last_error());
    }
    {
        arms_mask released = 0, held = 0, unknown = 0;
        char list[256];
        arms_brake_scan(g_arms, &released, &held, &unknown);
        if (released)
            tlog_err("自检: 关节%s 为强制松闸（Pn075=1），enable / free 时改为 0",
                     mask_list(released, list, sizeof(list)));
        if (held)
            tlog_err("自检: 关节%s 为强制抱闸（Pn075=2），enable / free 时改为 0",
                     mask_list(held, list, sizeof(list)));
        if (unknown)
            tlog_err("自检: 关节%s 的 Pn075 读取失败", mask_list(unknown, list, sizeof(list)));
    }
}
static bool uint_arg(const char *s, unsigned max, unsigned *out)
{
    char *end;
    errno = 0;
    unsigned long v = strtoul(s, &end, 0);
    if (!*s || *s == '-' || *end || errno || v > max)
        return false;
    *out = (unsigned)v;
    return true;
}
static const char *brake_text(arms_brake_mode m)
{
    switch (m) {
    case ARMS_BRAKE_NORMAL: return "normal  (Pn075=0，正常控制抱闸)";
    case ARMS_BRAKE_RELEASE: return "release (Pn075=1，强制松闸)";
    case ARMS_BRAKE_HOLD: return "hold    (Pn075=2，强制抱闸)";
    default: return "unknown";
    }
}

static bool parse_brake_mode(const char *s, arms_brake_mode *m)
{
    if (!strcmp(s, "normal") || !strcmp(s, "0")) *m = ARMS_BRAKE_NORMAL;
    else if (!strcmp(s, "release") || !strcmp(s, "1")) *m = ARMS_BRAKE_RELEASE;
    else if (!strcmp(s, "hold") || !strcmp(s, "2")) *m = ARMS_BRAKE_HOLD;
    else return false;
    return true;
}

/* brake：查看或临时设置抱闸模式（Pn075），可选择任意关节组合 */
static int cmd_brake(int argc, char **argv)
{
    arms_mask m;
    arms_brake_mode mode;
    int j, failures = 0;
    if (need_session() || selection(argv[1], &m)) return -2;
    if (argc == 2) {
        for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
            arms_brake_mode cur;
            if (!(m & ARMS_MASK_JOINT(j))) continue;
            if (arms_get_brake(g_arms, j, &cur) == ARMS_OK)
                printf("%d %-20s %s\n", j, arms_joint_name(g_arms, j), brake_text(cur));
            else
                printf("%d %-20s 读取失败: %s\n", j, arms_joint_name(g_arms, j), arms_last_error());
        }
        return ok();
    }
    if (!parse_brake_mode(argv[2], &mode)) return bad_args("模式只能是 normal | release | hold（或 0 | 1 | 2）");
    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
        if (!(m & ARMS_MASK_JOINT(j))) continue;
        if (arms_set_brake(g_arms, j, mode, true) == ARMS_OK) {
            printf("%d %-20s %s\n", j, arms_joint_name(g_arms, j), brake_text(mode));
        } else {
            failures++;
            printf("%d %-20s 失败: %s\n", j, arms_joint_name(g_arms, j), arms_last_error());
        }
    }
    if (mode == ARMS_BRAKE_RELEASE)
        tlog("强制松闸: 失能时抱闸不提供保持力；enable / free 时改为 normal");
    else if (mode == ARMS_BRAKE_HOLD)
        tlog("强制抱闸: 使能时不松闸；enable / free 时改为 normal");
    return failures ? failed("%d 个关节未完成", failures) : ok();
}

/* free：零力矩拖动 */
static int cmd_free(int argc, char **argv)
{
    arms_mask m;
    double kd = 0.0;
    char list[256];
    if (need_session() || selection(argv[1], &m)) return -2;
    if (argc > 2 && !number(argv[2], &kd)) return bad_args("kd 必须是非负数值");
    motion_cancel_refresh(m);
    if (arms_free(g_arms, m, kd) != ARMS_OK) return failed("%s", arms_last_error());
    tlog("零力矩拖动:%s，kd=%.3f", mask_list(m, list, sizeof(list)),
         kd);
    return ok();
}

/* zero / pn_save：可选择任意关节组合，逐个执行 */
static int cmd_multi_service(int argc, char **argv)
{
    arms_mask m;
    bool zero = !strcmp(argv[0], "zero");
    bool soft = zero && argc == 3 && !strcmp(argv[2], "--soft");
    bool hw = zero && !soft;
    int j, failures = 0;
    if (need_session() || selection(argv[1], &m)) return -2;
    if (argc > 2 && !soft && strcmp(argv[2], "--hw")) return bad_args("可选参数: --soft");
    motion_cancel_refresh(m);
    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
        arms_service_op op = !zero ? ARMS_SAVE : hw ? ARMS_ZERO_HW : ARMS_ZERO_SOFT;
        if (!(m & ARMS_MASK_JOINT(j))) continue;
        if (arms_service(g_arms, j, op, true) != ARMS_OK) {
            failures++;
            printf("%d %-20s 失败: %s\n", j, arms_joint_name(g_arms, j), arms_last_error());
            continue;
        }
        if (op == ARMS_ZERO_SOFT) {
            arms_joint_info q;
            arms_get_joint_info(g_arms, j, &q);
            printf("%d %-20s 零位偏置 offset_rad = %.6f，已生效并写入配置\n", j, arms_joint_name(g_arms, j),
                   q.offset_rad);
        } else {
            printf("%d %-20s %s\n", j, arms_joint_name(g_arms, j),
                   op == ARMS_ZERO_HW ? "已发出硬件清零脉冲（Pn101）并回读确认，配置中的 offset_rad 已写为 0"
                                      : "已发送参数保存脉冲并回读（Pn097）");
        }
    }
    if (hw && failures < __builtin_popcount(m))
        tlog("硬件零点在驱动器重新上电后生效；生效前上述关节不能 enable、free");
    return failures ? failed("%d 个关节未完成", failures) : ok();
}

/* Pn 编号为十进制，对象索引可带 0x 前缀；别名在参数目录中解析 */
static int pn_number(const char *key)
{
    const char *digits = !strncasecmp(key, "pn", 2) ? key + 2 : key;
    if (*digits && strlen(digits) <= 3 && strspn(digits, "0123456789") == strlen(digits))
        return (int)strtol(digits, NULL, 10);
    size_t count;
    const arms_pn_info *table = arms_pn_catalog(&count);
    for (size_t i = 0; i < count; i++)
        if (table[i].alias && !strcasecmp(key, table[i].alias)) return (int)table[i].number;
    return -1;
}

static void pn_describe(const arms_pn_info *p)
{
    printf("Pn%03u  0x%04x:00  %s", p->number, arms_pn_index(p->number), p->name);
    if (p->alias) printf(" [%s]", p->alias);
    printf("  范围 ");
    if (p->choices) {
        for (const int *c = p->choices; *c >= 0; c++) printf("%s%d", c == p->choices ? "" : "/", *c);
    } else if (p->min == p->max) printf("%d", p->min);
    else printf("%d..%d", p->min, p->max);
    printf("  %s\n", p->note);
}

static int hex_digit(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* 写入类型默认为 INT16，其他宽度由参数前缀指定 */
static bool encode_value(const char *arg, bool typed, uint8_t *out, size_t cap, size_t *len)
{
    if (typed && !strncmp(arg, "hex:", 4)) {
        const char *hex = arg + 4;
        size_t n = strlen(hex);
        if (!n || n % 2 || n / 2 > cap) return false;
        for (size_t i = 0; i < n / 2; ++i) {
            int hi = hex_digit(hex[2*i]), lo = hex_digit(hex[2*i+1]);
            if (hi < 0 || lo < 0) return false;
            out[i] = (uint8_t)((hi << 4) | lo);
        }
        *len = n / 2; return true;
    }
    unsigned bits = 16;
    bool sign = true;
    if (typed && strchr(arg, ':')) {
        if (*arg != 'u' && *arg != 'i') return false;
        sign = *arg++ == 'i';
        if (!strncmp(arg,"8:",2)) { bits = 8; arg += 2; }
        else if (!strncmp(arg,"16:",3)) arg += 3;
        else if (!strncmp(arg,"32:",3)) { bits = 32; arg += 3; }
        else if (!strncmp(arg,"64:",3)) { bits = 64; arg += 3; }
        else return false;
    }
    const char *digits = (*arg == '+' || *arg == '-') ? arg + 1 : arg;
    int base = !strncasecmp(digits, "0x", 2) ? 16 : 10;
    uint64_t u; char *end;
    errno = 0;
    if (sign) {
        int64_t v = strtoll(arg, &end, base);
        if (bits < 64 && (v < -(INT64_C(1) << (bits-1)) || v >= (INT64_C(1) << (bits-1)))) return false;
        u = (uint64_t)v;
    } else {
        if (*arg == '-') return false;
        u = strtoull(arg, &end, base);
        if (bits < 64 && u >= (UINT64_C(1) << bits)) return false;
    }
    if (errno || end == arg || *end || bits/8 > cap) return false;
    *len = bits/8;
    for (size_t i = 0; i < *len; ++i) out[i] = (uint8_t)(u >> (8*i));
    return true;
}

static int cmd_parameter(int argc, char **argv)
{
    bool pn = !strcmp(argv[0], "pn"), sdo = !strcmp(argv[0], "sdo");
    if (pn && (!strcmp(argv[1], "list") || !strcmp(argv[1], "help"))) {
        if (!strcmp(argv[1], "list") && argc == 2) {
            size_t n; const arms_pn_info *t = arms_pn_catalog(&n);
            for (size_t i = 0; i < n; ++i) pn_describe(&t[i]);
        } else if (!strcmp(argv[1], "help") && argc == 3) {
            const arms_pn_info *p = arms_pn_lookup((unsigned)pn_number(argv[2]));
            if (!p) return failed("未收录的参数：%s", argv[2]);
            pn_describe(p);
        } else return -2;
        return ok();
    }
    int base = sdo ? 4 : pn ? 3 : 2;
    if (argc != base && argc != base + 1) return -2;
    arms_mask mask;
    if (need_session() || selection(argv[1], &mask)) return -1;
    if (__builtin_popcount(mask) != 1) return failed("参数读写一次只选一个关节");
    int j = __builtin_ctz(mask), number;
    unsigned index, sub = 0;
    if (sdo) {
        if (!uint_arg(argv[2], UINT16_MAX, &index) || !uint_arg(argv[3], UINT8_MAX, &sub)) return -2;
        number = sub ? -1 : arms_pn_from_index(index);
    } else {
        number = pn_number(pn ? argv[2] : argv[0]);
        if (number < 0) return failed("无效的参数号或别名");
        index = (unsigned)arms_pn_index((unsigned)number);
    }
    if (argc == base) {
        uint8_t data[4096]; size_t n;
        if (arms_sdo_read(g_arms, j, index, sub, data, sizeof(data), &n) != ARMS_OK) return failed("%s", arms_last_error());
        printf("0x%04x:%02x =", index, sub);
        for (size_t i = 0; i < n; ++i) printf(" %02x", data[i]);
        if (n == 2) {
            int v = data[0] | (data[1] << 8);
            if (number >= 0 && v >= 32768) v -= 65536;
            printf(" (%d)", v);
            if (number == 26) printf("  %d V", v);
            if (number == 82) printf("  %.1f A/N·m", v / 10.0);
        }
        puts("");
        return ok();
    }
    uint8_t payload[256]; size_t len;
    if (!encode_value(argv[base], sdo, payload, sizeof(payload), &len))
        return failed(sdo ? "SDO 值格式无效，应为整数、i/u 位宽前缀或 hex:字节串" : "值应为 INT16 整数");
    unsigned word = len == 2 ? payload[0] | ((unsigned)payload[1] << 8) : UINT_MAX;
    if (number == 75 && word <= 2) {
        char *av[] = {"brake", argv[1], (char *)arms_brake_mode_name((arms_brake_mode)word)};
        return cmd_brake(3, av);
    }
    if ((number == 97 || number == 101) && word == 1) {
        char *av[] = {number == 97 ? "pn_save" : "zero", argv[1], "--hw"};
        return cmd_multi_service(number == 97 ? 2 : 3, av);
    }
    arms_status st = sdo ? arms_sdo_write(g_arms, j, index, sub, payload, len, true)
                         : arms_pn_write(g_arms, j, (unsigned)number, (int)word - (word >= 32768 ? 65536 : 0), true);
    if (st != ARMS_OK) return failed("%s", arms_last_error());
    printf("%d 0x%04x:%02x 驱动器已接受 %zu 字节写入（未回读）\n", j, index, sub, len);
    return ok();
}

static int cmd_service(int argc, char **argv)
{
    arms_mask m;
    if (need_session() || selection(argv[1], &m))
        return -1;
    if (__builtin_popcount(m) != 1)
        return failed("该命令一次只能选择一个关节");
    int j = __builtin_ctz(m);
    const char *op = argv[0];
    arms_status rc = ARMS_OK;
    unsigned a = 0, b = 0;
    uint8_t bytes[64];
    size_t n = 0;
    if (!strcmp(op, "identify")) {
        arms_joint_state s;
        arms_read_joint(g_arms, j, &s);
        printf("%d %s %s slave=%d", j, s.name, arms_arm_name(s.arm), s.slave);
        for (unsigned sub = 1; sub <= 4; sub++) {
            rc = arms_sdo_read(g_arms, j, 0x1018, sub, bytes, 4, &n);
            if (rc != ARMS_OK || n != 4)
                return failed("身份对象 1018:%u 不可读: %s", sub, arms_last_error());
            uint32_t x = (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) | ((uint32_t)bytes[2] << 16) |
                         ((uint32_t)bytes[3] << 24);
            printf(" %s=0x%08x", ((const char *[]){"", "vendor", "product", "revision", "serial"})[sub], x);
        }
        puts("");
    } else if (!strcmp(op, "reg")) {
        if (!uint_arg(argv[2], 65535, &a) || (argc > 3 && !uint_arg(argv[3], 4, &b)))
            return -2;
        if (argc == 3)
            b = 2;
        if ((b != 1 && b != 2 && b != 4) || a + b > 65536)
            return -2;
        arms_joint_state s;
        arms_bus_stats st;
        arms_read_joint(g_arms, j, &s);
        arms_get_stats(g_arms, &st);
        if (mock_backend) {
            if (a != 0x130 || b != 2)
                return failed("模拟 ESC 仅提供 AL Status (0x130, 2)");
            printf("0x%04x\n", s.al_state);
        } else {
            char master[16], slave[16], address[16];
            snprintf(master, 16, "%d", st.arm[s.arm].master_index);
            snprintf(slave, 16, "%d", s.slave);
            snprintf(address, 16, "%u", a);
            char *args[] = {"ethercat", "reg_read", "-m", master,
                            "-p",       slave,      "-t", b == 1 ? "uint8" : b == 2 ? "uint16" : "uint32",
                            address,    NULL};
            pid_t pid;
            int status;
            if (posix_spawn(&pid, "/usr/local/etherlab/bin/ethercat", NULL, NULL, args, environ))
                return failed("无法启动 EtherLab 寄存器读取工具");
            while (waitpid(pid, &status, 0) < 0)
                if (errno != EINTR)
                    return failed("等待 reg_read 失败");
            if (!WIFEXITED(status) || WEXITSTATUS(status))
                return failed("寄存器读取失败");
        }
    }
    return rc == ARMS_OK ? ok() : failed("%s", arms_last_error());
}

static int cmd_help(int argc, char **argv);

static const command commands[] = {
    {"help", "通用", "[命令]", 0, 1, "列出全部命令，或显示指定命令的详细用法",
     "  不带参数时按分组列出命令；带命令名时等同于 <命令> -h。", "  help\n  help commtest", cmd_help},
    {"status", "查看", "[关节] [--raw]", 0, 2, "显示关节状态表",
     "  关节: 21 | l_wrist_z | left | right | all | 15-18 | 21,28，默认为全部已打开关节。\n"
     "  状态: offline / preop / safeop / locked（OP，伺服失能）/ enabled / FAULT；\n"
     "        released / held 表示伺服失能且抱闸模式为强制松闸 / 强制抱闸。\n"
     "  cmd、kp、kd: 当前下发的位置指令和增益。\n"
     "  flag: P 为软位置限位生效，T 为指令超时。flt: 累计故障次数。\n"
     "  温度: 执行器温度（Pn002=77），周期约 2 秒；- 表示未读取。\n"
     "  --raw: 附加显示原始 PDO 数据。",
     "  status\n  status left\n  status 21", cmd_status},
    {"diag", "查看", "[关节] [reset]", 0, 2, "显示驱动器丢帧次数和从站 ESC 错误计数",
     "  驱动器丢帧: 驱动器统计的 EtherCAT 丢帧次数（Pn002=63），断电清零。\n"
     "  ESC 错误计数: 寄存器 0x0300..0x0313，饱和值 255。端口 0 为主站侧，端口 1 为下游侧。\n"
     "    线路类: 无效帧、物理层错误、链路丢失。\n"
     "    处理类: 转发错误、处理单元错误、PDI 错误。\n"
     "  reset: 清零全部从站的 ESC 错误计数。",
     "  diag\n  diag left\n  diag reset", cmd_diag},
    {"watch", "查看", "[关节] [刷新Hz]", 0, 2, "持续刷新显示关节状态，回车结束",
     "  刷新频率 1..50 Hz，默认 5 Hz。以 -T 启动时按时间戳逐次输出。",
     "  watch\n  watch left 10\n  watch 21", cmd_watch},
    {"bus", "查看", "[reset]", 0, 1, "显示两条臂的通信统计；reset 清零统计",
     "  周期: 平均/最短/最长间隔、抖动、唤醒延迟、超时周期、跳过周期。\n"
     "  每条臂: 链路、OP 从站数、WKC、异常次数、最长连续异常、发送错误、收发耗时。",
     "  bus\n  bus reset", cmd_bus},
    {"info", "查看", "[关节]", 0, 1, "显示版本、配置、实时参数及关节映射和限位", "", "  info\n  info 21", cmd_info},
    {"enable", "控制", "<关节> [-t 毫秒]", 1, 3, "使能关节",
     "  使能后指令为零: kp=0、kd=0，速度和前馈为 0。\n"
     "  抱闸模式（Pn075）为 1 或 2 的关节先改为 0，不保存。\n"
     "  -t: 使能等待超时，0 或省略时使用驱动默认值。",
     "  enable 21\n  enable left -t 2000", cmd_enable},
    {"disable", "控制", "[关节]", 0, 1, "使关节失能", "  默认为全部已打开关节。", "  disable", cmd_disable},
    {"reset", "控制", "<关节>", 1, 1, "故障复位", "  复位后关节处于失能状态。", "  reset 17", cmd_reset},
    {"mit", "控制", "<关节> <位置> <kp> <kd> [速度] [前馈]", 4, 6, "下发 MIT 指令",
     "  单位: 位置 rad，速度 rad/s，前馈 N·m。速度和前馈默认为 0。\n"
     "  指令直接下发，不插值、不限幅；kp、kd 为负时按 0 下发。\n"
     "  实际位置超出 pos_limit 时，朝外的速度和前馈置 0。\n"
     "  速度或前馈非零时以 100 Hz 持续刷新。所选关节须已使能。",
     "  mit 17 0.1 5 0.5\n  mit 18 0.1 2 0.1 0.05", cmd_mit},
    {"hold", "控制", "[关节]", 0, 1, "保持当前指令位置",
     "  停止指令刷新，速度和前馈置 0，位置、kp、kd 不变。默认为全部已打开关节。", "  hold", cmd_hold},
    {"swing", "测试", "<关节> <幅值> <周期秒> <次数> <kp> <kd>", 6, 6, "以当前指令位置为中心正弦摆动",
     "  幅值单位 rad，以 200 Hz 下发位置和速度。结束时回到中心；回车或 Ctrl+C 中止并保持当前指令位置。",
     "  swing 17 0.05 4 3 5 0.5", cmd_swing},
    {"selftest", "查看", "", 0, 0, "检查在线状态、故障与保护锁定", "  不改变关节状态。", "  selftest", cmd_selftest},
    {"identify", "查看", "<关节>", 1, 1, "读取从站身份、序列号与配置映射", "", "  identify 21", cmd_service},
    {"free", "控制", "<关节> [kd]", 1, 2, "零力矩拖动",
     "  使能关节并下发 kp=0、速度 0、前馈 0，kd 为阻尼，默认 0。\n"
     "  抱闸模式（Pn075）为 1 或 2 的关节先改为 0，不保存。",
     "  free 21\n  free left 0.2", cmd_free},
    {"brake", "维护", "<关节> [normal|release|hold]", 1, 2, "查看或设置抱闸模式（Pn075）",
     "  不带模式时读取 Pn075。\n"
     "  0 normal  正常控制: 伺服使能时松闸，失能时抱闸。\n"
     "  1 release 强制松闸。\n"
     "  2 hold    强制抱闸。\n"
     "  release、hold 要求关节失能。设置不保存；quit 时将本次修改过的关节恢复为 normal。",
     "  brake all\n  brake 16 normal\n  brake left hold", cmd_brake},
    {"zero", "维护", "<关节> [--soft]", 1, 2, "以当前位置为零点",
     "  已使能的关节先失能，确认静止后校零，完成后保持失能。\n"
     "  默认硬件校零: 写 Pn101 清零脉冲，并将配置中的 offset_rad 置 0；要求 Pn075=0。\n"
     "    驱动器重新上电后生效，生效前该关节不能 enable、free。\n"
     "  --soft: 软件校零，将当前位置写入配置文件的 offset_rad，立即生效。",
     "  zero 21\n  zero left\n  zero 21 --soft", cmd_multi_service},
    {"pn_save", "维护", "<关节>", 1, 1, "保存驱动器参数（Pn097）",
     "  已使能的关节先失能，确认静止后写 Pn097 保存脉冲，完成后保持失能。重新上电后生效。",
     "  pn_save 21\n  pn_save all", cmd_multi_service},
    {"pn", "维护", "<关节> <编号|别名> [值] | list | help <编号|别名>", 1, 3, "读写驱动器参数",
     "  写参数要求关节失能，不回读、不保存。Pn075、Pn097、Pn101 分别由 brake、pn_save、zero 设置。\n"
     "  list 列出手册中的常用参数。",
     "  pn list\n  pn help brake_voltage\n  pn 15 Pn026", cmd_parameter},
    {"sdo", "维护", "<关节> <索引> <子索引> [值]", 3, 4, "读写对象字典",
     "  写入类型默认为 i16；可写 i8/u8/i16/u16/i32/u32/i64/u64:值 或 hex:小端字节串。", "  sdo 15 0x2026 0",
     cmd_parameter},
    {"brake_voltage", "维护", "<关节> [伏]", 1, 2, "读写制动器保持电压（Pn026）", "  等同于 pn <关节> Pn026 [值]。",
     "  brake_voltage 15", cmd_parameter},
    {"torque_scale", "维护", "<关节> [原始值]", 1, 2, "读写转矩系数（Pn082），10 表示 1 A/N·m",
     "  等同于 pn <关节> Pn082 [值]。", "  torque_scale 15", cmd_parameter},
    {"reg", "维护", "<关节> <地址> [1|2|4]", 2, 3, "读取 ESC 寄存器", "  长度为 1、2 或 4 字节，只读。",
     "  reg 21 0x130 2", cmd_service},
    {"check", "查看", "", 0, 0, "检查已打开关节是否全部在线，并显示屏蔽掩码与离线保护状态",
     "  在线条件: 从站处于 OP、WKC 正确且该臂通信正常。被屏蔽的关节不参与检查。", "  check", cmd_check},
    {"power", "控制", "on | off | status", 1, 1, "电机电源上电、断电或显示状态",
     "  off: 先使全部已使能关节失能，再断电。\n"
     "  on: 上电并等待通信恢复，最长 30 秒；恢复后全部关节处于失能状态。\n"
     "  仅适用于具备 PCI 电源接口的硬件。",
     "  power off\n  power on\n  power status", cmd_power},
    {"commtest", "测试", "<秒> [--enabled <kp> <kd>]", 1, 4, "通信质量测试",
     "  默认要求全部关节失能。--enabled: 使能全部已打开关节，以给定 kp、kd 保持使能时的位置，结束后失能。\n"
     "  开始时清零通信统计和 ESC 错误计数，每 10 秒输出一次进度。\n"
     "  通过条件: WKC 异常、发送错误、超时周期、跳过周期均为 0，无线路类 ESC 错误计数，驱动器丢帧不增加。\n"
     "  仅有处理类 ESC 错误计数时给出提示，不判为不通过。",
     "  commtest 300\n  commtest 60 --enabled 5 0.5", cmd_commtest},
    {"version", "通用", "", 0, 0, "显示版本", "", "  version", cmd_version},
    {"quit", "通用", "", 0, 0, "停止通信并退出", "  别名: exit、q。", "  quit", cmd_quit},
};
#define COMMAND_COUNT (sizeof(commands) / sizeof(commands[0]))

static const command *find_command(const char *name)
{
    size_t i;
    if (strcmp(name, "exit") == 0 || strcmp(name, "q") == 0) name = "quit";
    if (strcmp(name, "?") == 0) name = "help";
    for (i = 0; i < COMMAND_COUNT; ++i)
        if (strcmp(commands[i].name, name) == 0) return &commands[i];
    return NULL;
}

/* argv 只含当前词之前的完整参数；命令名、关节名复用实际注册表和会话映射。 */
static void complete_command(size_t argc, const char *const argv[], le_choices *out)
{
    const command *c = argc ? find_command(argv[0]) : NULL;
    if (!argc || (argc == 1 && c && c->fn == cmd_help)) {
        for (size_t i = 0; i < COMMAND_COUNT; ++i) le_add_choice(out, commands[i].name);
        return;
    }
    if (!c) return;
    if (argc == 1) le_add_choice(out, "--help");
    const cmd_fn fn = c->fn;
    bool takes_joints = fn == cmd_status || fn == cmd_watch || fn == cmd_info || fn == cmd_enable ||
                        fn == cmd_disable || fn == cmd_reset || fn == cmd_hold || fn == cmd_mit ||
                        fn == cmd_swing || fn == cmd_free || fn == cmd_brake || fn == cmd_multi_service ||
                        fn == cmd_service || fn == cmd_parameter || fn == cmd_diag;
    if ((argc == 1 && takes_joints) ||
        (argc == 2 && c->fn == cmd_status && !strcmp(argv[1], "--raw"))) {
        bool single = fn == cmd_service || fn == cmd_parameter;
        arms_mask mask = arms_opened_joints(g_arms);
        if (c->fn == cmd_status) mask |= arms_ignored_joints(g_arms) | arms_missing_joints(g_arms);
        if (!single) {
            out->joint_list = true;
            le_add_choice(out, "all");
            if (mask & ARMS_MASK_LEFT) le_add_choice(out, "left");
            if (mask & ARMS_MASK_RIGHT) le_add_choice(out, "right");
        }
        for (int j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
            if (!(mask & ARMS_MASK_JOINT(j))) continue;
            char id[8];
            snprintf(id, sizeof(id), "%d", j);
            le_add_choice(out, id);
            le_add_choice(out, arms_joint_name(g_arms, j));
        }
    }
    if (!strcmp(c->name, "pn")) {
        if (argc == 1) { le_add_choice(out, "list"); le_add_choice(out, "help"); }
        if (argc == 2 && strcmp(argv[1], "list")) {
            size_t n; const arms_pn_info *t = arms_pn_catalog(&n);
            for (size_t i = 0; i < n; ++i) {
                char key[12]; snprintf(key, sizeof(key), "Pn%03u", t[i].number);
                le_add_choice(out, key);
                if (t[i].alias) le_add_choice(out, t[i].alias);
            }
        }
    }
    if (argc == 1 && c->fn == cmd_bus) le_add_choice(out, "reset");
    if (argc <= 2 && c->fn == cmd_diag) le_add_choice(out, "reset");
    if (argc == 1 && c->fn == cmd_power) {
        le_add_choice(out, "on"); le_add_choice(out, "off"); le_add_choice(out, "status");
    }
    if (argc <= 2 && c->fn == cmd_status && (argc == 1 || strcmp(argv[1], "--raw"))) le_add_choice(out, "--raw");
    if (argc == 2 && c->fn == cmd_brake) {
        le_add_choice(out, "normal"); le_add_choice(out, "release"); le_add_choice(out, "hold");
    }
    if (argc == 2 && !strcmp(c->name, "zero")) le_add_choice(out, "--soft");
    if (argc == 2 && c->fn == cmd_enable) le_add_choice(out, "-t");
    if (argc == 2 && c->fn == cmd_commtest) le_add_choice(out, "--enabled");
}

static int cmd_help(int argc, char **argv)
{
    size_t g, i;
    if (argc > 1) {
        const command *c = find_command(argv[1]);
        if (!c) return failed("没有命令 %s；输入 help 查看全部命令", argv[1]);
        print_detail(c);
        return 0;
    }
    for (g = 0; g < COMMAND_COUNT; ++g) {
        const char *group = commands[g].group;
        for (i = 0; i < g; i++)
            if (!strcmp(commands[i].group, group))
                break;
        if (i < g)
            continue;
        printf("[%s]\n", group);
        for (i = 0; i < COMMAND_COUNT; ++i) {
            const command *c = &commands[i];
            char head[96];
            if (strcmp(c->group, group) != 0)
                continue;
            snprintf(head, sizeof(head), "%s %s", c->name, c->args);
            printf("  %-24s %s\n", head, c->brief);
        }
    }
    printf("关节写法: 21 | l_wrist_z | left | right | all | 15-18 | 21,28\n");
    printf("<命令> -h 显示详细用法\n");
    return 0;
}

static int run_command(int argc, char **argv)
{
    const command *c;
    int args;
    if (argc == 0) return 0;
    c = find_command(argv[0]);
    if (!c) {
        printf("未知命令 \"%s\"；输入 help 查看全部命令。命令未执行。\n", argv[0]);
        return -1;
    }
    if (argc > 1 && (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)) {
        print_detail(c);
        return 0;
    }
    args = argc - 1;
    if (args < c->min_args || args > c->max_args) {
        print_usage(c);
        printf("参数个数不对（需要 %d..%d 个，实际 %d 个）。命令未执行；%s -h 查看详细说明。\n", c->min_args,
               c->max_args, args, c->name);
        return -1;
    }
    int rc = c->fn(argc, argv);
    if (rc == -2) print_usage(c);
    return rc;
}

static int split(char *line, char **argv, int max)
{
    int n = 0;
    char *save = NULL, *tok;
    for (tok = strtok_r(line, " \t\r\n", &save); tok && n < max; tok = strtok_r(NULL, " \t\r\n", &save))
        argv[n++] = tok;
    return n;
}

/* 离线保护触发时由监视线程调用：驱动已使全部关节失能，此处切断电机电源 */
static void on_trip(void)
{
    if (mock_backend) {
        tlog_err("离线保护: 模拟后端不操作电源");
        return;
    }
    if (!powered) {
        tlog_err("离线保护: 电源不由本程序控制，需手动断电");
        return;
    }
    if (request_power(false) == 0) {
        tlog_err("离线保护: 已请求断电；power on 重新上电并检查在线状态");
    } else {
        tlog_err("离线保护: 下电失败，需手动断电");
    }
}

static void build_prompt(char *buf, size_t size)
{
    arms_bus_stats st;
    char hz[16];
    int j, enabled = 0, fault = -1;
    arms_get_stats(g_arms, &st);
    hz_short(st.cycle_hz, hz, sizeof(hz));
    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
        arms_joint_state s;
        if (!(arms_opened_joints(g_arms) & ARMS_MASK_JOINT(j))) continue;
        arms_read_joint(g_arms, j, &s);
        if (s.mode == ARMS_JS_ENABLED) enabled++;
        if (s.mode == ARMS_JS_FAULT && fault < 0) fault = j;
    }
    {
        char state[32] = "";
        if (fault >= 0) snprintf(state, sizeof(state), " FAULT:%d", fault);
        else if (st.protect.reason != ARMS_PROTECT_NONE) snprintf(state, sizeof(state), " PROT:%d", st.protect.joint);
        snprintf(buf, size, "arms %s L%d/%d R%d/%d E%d%s%s%s> ", hz, st.arm[0].slaves_op, st.arm[0].slaves_expected,
                 st.arm[1].slaves_op, st.arm[1].slaves_expected, enabled, state,
                 (st.arm[0].comm_failed || st.arm[1].comm_failed) ? " COMM!" : "", st.tripped ? " TRIP" : "");
    }
}

static void usage(FILE *out)
{
    fprintf(
        out,
        "用法: arms_test [选项]\n"
        "BXI 双臂 EtherCAT 测试工具（KGU 驱动器，MIT 模式）。\n"
        "启动时依次: 获取 root 权限 -> 启动 EtherLab 主站 -> 配置实时运行环境 -> 电机上电 -> 扫描、预热，\n"
        "全部关节保持伺服失能。退出时停止通信、断电并还原实时运行环境。\n"
        "\n"
        "选项:\n"
        "  -f, --freq HZ        周期频率 2000|1000|500|250|200|125（默认取配置）\n"
        "  -j, --joints LIST    参与的关节: all | left | right | 15-21 | 21,28（默认 all）\n"
        "  -r, --rt-cpu N       实时线程 CPU，-1 表示不绑定（默认取配置 realtime.rt_cpu）\n"
        "  -c, --config FILE    配置文件（默认 $ARMS_CONFIG、/etc/bxi/arms.json、./config/arms.json）\n"
        "  -e, --exec CMD       执行命令后退出，可重复，按顺序执行\n"
        "      --mock           使用模拟后端，不访问硬件，不需要 root\n"
        "      --no-power       不操作电机电源（电源已由其他程序打开时使用）\n"
        "      --keep-power     退出时不断电\n"
        "      --power-wait MS  上电后等待总线稳定的最长时间（默认 15000）；从站数达到配置并稳定后立即继续\n"
        "      --no-rt-setup    不配置实时运行环境（中断绑定、CPU 隔离，以及 EtherCAT 网口退出 NetworkManager、\n"
        "                       去掉 IP 地址、关闭 IPv6；退出时还原）\n"
        "  -m, --mask MASK      屏蔽掩码（31 位，第 n 位对应关节 n）。被屏蔽的关节不打开、不做在线检查，\n"
        "                       掉线不触发下电保护（默认取配置 safety.ignore_mask）\n"
        "      --check / --no-check  本次要求 / 不要求未屏蔽关节全部在线（不改变运行时掉线策略）\n"
        "      --check-motor 0|1     显式选择启动检查；--motor-disable MASK 等价于 --mask\n"
        "      --event-log FILE  通信事件日志（默认 arms_driver/logs/comm_events.log，即配置目录旁的 logs/）。\n"
        "                       WKC 异常、通信失败与恢复、周期延迟、离线保护触发时，追加事件前 64 个周期的时序和从站状态\n"
        "  -T, --timestamps     每一行输出都加时间戳（默认只在事件行加）\n"
        "  -h, --help           显示本帮助\n"
        "  -V, --version        显示版本\n"
        "\n"
        "关节编号: 15-21 左臂（肩y/x/z、肘y、腕x/y/z），22-28 右臂，顺序相同。\n"
        "示例:\n"
        "  arms_test                           上电、扫描两臂、预热，进入交互终端\n"
        "  arms_test -f 2000 -j left           只打开左臂，2 kHz\n"
        "  arms_test -e \"commtest 300\"         通信测试 300 秒后退出\n"
        "  arms_test --mock                    使用模拟后端\n"
        "  arms_test -m 0x10200000             屏蔽关节 21、28 后启动\n"
        "\n"
        "启动在线检查（默认开启）: 上电扫描后，未屏蔽的关节必须全部在线，否则下电并退出。\n"
        "运行中未屏蔽的关节掉线: 全部关节失能并下电（safety.offline_action = power_off），power on 恢复。\n"
        "交互终端中输入 help 列出命令，<命令> -h 显示详细用法。\n");
}

int main(int argc, char **argv)
{
    static const struct option long_opts[] = {
        {"freq", required_argument, NULL, 'f'},
        {"joints", required_argument, NULL, 'j'},
        {"config", required_argument, NULL, 'c'},
        {"exec", required_argument, NULL, 'e'},
        {"mock", no_argument, NULL, 'M'},
        {"rt-cpu", required_argument, NULL, 'r'},
        {"power", no_argument, NULL, 'p'},
        {"help", no_argument, NULL, 'h'},
        {"version", no_argument, NULL, 'V'},
        {"power-wait", required_argument, NULL, 'w'},
        {"keep-power", no_argument, NULL, 'k'},
        {"no-power", no_argument, NULL, 'P'},
        {"no-rt-setup", no_argument, NULL, 'R'},
        {"timestamps", no_argument, NULL, 'T'},
        {"check", no_argument, NULL, 1000},
        {"check-motor", required_argument, NULL, 1001},
        {"motor-disable", required_argument, NULL, 'm'},
        {"mask", required_argument, NULL, 'm'},
        {"no-check", no_argument, NULL, 'N'},
        {"event-log", required_argument, NULL, 1002},
        {NULL, 0, NULL, 0},
    };
    arms_options opt;
    const char *execs[16];
    int nexec = 0, ch, rc = 0, i;
    bool power = true, keep_power = false, rt_setup = true, stamp_all = false;
    char started[32];
    char err[256];
    struct sigaction sa;
    arms_status st;

    arms_options_init(&opt);
    opt.mode = ARMS_MODE_STANDALONE;
    opt.log = log_line;
    while ((ch = getopt_long(argc, argv, "f:j:r:c:e:m:ThV", long_opts, NULL)) != -1) {
        switch (ch) {
        case 'f': {
            char *end;
            long v = strtol(optarg, &end, 10);
            if (*end || v <= 0 || (unsigned long)v > UINT_MAX) {
                fprintf(stderr, "错误: --freq 需要整数频率\n");
                return 2;
            }
            opt.cycle_hz = (unsigned int)v;
            break;
        }
        case 'j':
            if (arms_parse_joints(optarg, 0, &opt.joints, err, sizeof(err)) != 0) {
                fprintf(stderr, "错误: --joints %s\n", err);
                return 2;
            }
            break;
        case 'c': opt.config_path = optarg; break;
        case 'e':
            if (nexec >= 16) {
                fprintf(stderr, "错误: -e 最多 16 次\n");
                return 2;
            }
            execs[nexec++] = optarg;
            break;
        case 'M': opt.backend = ARMS_BACKEND_MOCK; break;
        case 'm': {
            char *end;
            unsigned long long v;
            errno = 0;
            v = strtoull(optarg, &end, 0);
            if (end == optarg || *end || errno || optarg[0] == '-' || v > 0x7FFFFFFFull) {
                fprintf(stderr, "错误: --mask 需要 31 位掩码 0..0x7fffffff，第 n 位对应关节 n\n");
                return 2;
            }
            opt.ignore_mask = (int64_t)v;
            break;
        }
        case 1000:
            opt.check_motors = 1;
            break;
        case 1001:
            if (strcmp(optarg, "0") && strcmp(optarg, "1")) {
                fprintf(stderr, "--check-motor: 0 | 1\n");
                return 2;
            }
            opt.check_motors = optarg[0] - '0';
            break;
        case 'N': opt.check_motors = 0; break;
        case 1002: opt.event_log = optarg; break;
        case 'r': {
            char *end;
            long v = strtol(optarg, &end, 10);
            if (*end || v < -1 || v > 1023) {
                fprintf(stderr, "错误: --rt-cpu 需要 CPU 编号，或 -1 表示不绑定\n");
                return 2;
            }
            opt.rt_cpu = (int)v;
            break;
        }
        case 'p': power = true; break;
        case 'P': power = false; break;
        case 'T': stamp_all = g_stamp_all = true; break;
        case 'R': rt_setup = false; break;
        case 'k': keep_power = true; break;
        case 'w': {
            char *end;
            long v = strtol(optarg, &end, 10);
            if (*end || v < 1000 || v > 60000) {
                fprintf(stderr, "错误: --power-wait 需要 1000..60000 的毫秒数\n");
                return 2;
            }
            power_wait = (unsigned int)v;
            break;
        }
        case 'h': usage(stdout); return 0;
        case 'V': printf("arms_test %s\n", arms_version()); return 0;
        default: usage(stderr); return 2;
        }
    }
    if (optind < argc) {
        fprintf(stderr, "错误: 多余的参数 \"%s\"；arms_test -h 查看用法\n", argv[optind]);
        return 2;
    }

    if (opt.backend != ARMS_BACKEND_MOCK && arms_env_ensure_root(argc, argv) != 0) return 1;
    if (stamp_all) {
        if (tlog_stamp_all_lines() == 0) atexit(tlog_shutdown);
        else fprintf(stderr, "警告: 无法启用逐行时间戳\n");
    }

    memset(&sa, 0, sizeof(sa));
    if (pipe2(stop_pipe, O_NONBLOCK | O_CLOEXEC) < 0) { perror("pipe2"); return 1; }
    sa.sa_handler = on_sigint;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    tlog("arms_test %s  启动时间 %s", arms_version(), tlog_datetime(started, sizeof(started)));
    mock_backend = opt.backend == ARMS_BACKEND_MOCK;
    if (mock_backend) {
        power = false;
        rt_setup = false;
    } else {
        if (arms_acquire_lease() < 0) {
            tlog_err("硬件会话锁不可用：权限不足或另一个控制程序正在运行");
            return 1;
        }
        if (arms_env_start_master() != 0) return 1;
        if (rt_setup && arms_env_rt_apply(opt.config_path, opt.rt_cpu >= 0 ? opt.rt_cpu : -2) < 0) return 1;
        if (power) {
            if (arms_power_on(0) != 0) {
                arms_power_off(); /* 请求失败时不视为已断电 */
                arms_env_rt_revert();
                return 1;
            }
            powered = power_known = true;
            /* 不固定等待：扫描时等到各臂从站数达到配置并保持稳定，最长 power_wait */
            opt.scan_timeout_ms = power_wait;
        }
    }
    tlog("启动: %s后端，扫描并预热，全部关节保持伺服失能", opt.backend == ARMS_BACKEND_MOCK ? "模拟" : "EtherLab ");
    st = arms_open(&opt, &g_arms);
    if (st != ARMS_OK) {
        tlog_err("启动失败: %s", arms_strerror(st));
        tlog_err("  原因: %s", arms_last_error());
        if (powered && (st == ARMS_E_TOPOLOGY || st == ARMS_E_LINK)) {
            if (arms_power_off() == 0)
                tlog_err("启动在线检查未通过，已请求断电保护");
            else tlog_err("启动在线检查未通过，下电失败，需手动断电");
        } else if (powered && !keep_power) {
            arms_power_off();
        }
        arms_env_rt_revert();
        return 1;
    }
    {
        arms_bus_stats bs;
        arms_get_stats(g_arms, &bs);
        print_stats(&bs);
    }
    tlog("通信事件日志: %s", arms_event_log_path(g_arms));
    if (g_terminate) goto close_session;
    startup_selftest();
    motion_set_trip_handler(on_trip);
    if (motion_start(g_arms, &g_interrupted, nexec > 0) != 0) {
        rc = 1;
        goto close_session;
    }
    if (nexec > 0) {
        for (i = 0; i < nexec && !g_interrupted; ++i) {
            char line[512], *av[16];
            int ac;
            if (snprintf(line, sizeof(line), "%s", execs[i]) >= (int)sizeof(line)) {
                tlog_err("命令过长，未执行");
                rc = 1;
                break;
            }
            ac = split(line, av, 16);
            tlog(">> %s", execs[i]);
            if (run_command(ac, av) != 0) {
                rc = 1;
                break;
            }
        }
    } else {
        tlog("就绪。help 列出命令，Tab 补全，quit 退出");
        while (!g_quit && !g_terminate) {
            char prompt[128], line[512], *av[16];
            int ac;
            int r;
            build_prompt(prompt, sizeof(prompt));
            g_interrupted = 0;
            r = le_readline(prompt, line, sizeof(line), stop_pipe[0], complete_command);
            if (g_terminate) break;
            if (r == LE_INTERRUPTED) {
                printf("（Ctrl+C：输入 quit 退出）\n");
                continue;
            }
            if (r == LE_EOF) break;
            le_history_add(line);
            ac = split(line, av, 16);
            if (ac > 0 && !stamp_all) {
                char ts[16];
                printf("[%s] %s\n", tlog_time(ts, sizeof(ts)), av[0]);
            }
            run_command(ac, av);
        }
    }
    motion_stop();
    if (motion_enabled() && disable_joints(arms_opened_joints(g_arms), 50)) rc = 1;
close_session:
    if (arms_log_events(g_arms) > 0) tlog("已记录通信事件: %s", arms_event_log_path(g_arms));
    tlog("退出: 停止通信并释放主站");
    arms_close(g_arms);
    if (powered) {
        if (keep_power && !g_terminate && rc == 0)
            tlog("电源: 保持上电（--keep-power）");
        else if (arms_power_off() != 0) rc = 1;
    }
    arms_env_rt_revert();
    tlog("已退出");
    return rc;
}
