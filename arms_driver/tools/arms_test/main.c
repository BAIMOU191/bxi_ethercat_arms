/**
 * @file main.c
 * @brief arms_test：双臂 EtherCAT 驱动测试工具
 *
 * 命令统一登记在 commands[] 中，help、"命令 -h" 和参数个数检查均由该表生成。
 */
#define _GNU_SOURCE
#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/select.h>
#include <time.h>
#include <unistd.h>

#include "arms/arms.h"
#include "joint_select.h"
#include "power.h"
#include "runtime_env.h"

static volatile sig_atomic_t g_interrupted;
static arms_t *g_arms;

static void on_sigint(int sig)
{
    (void)sig;
    g_interrupted = 1;
}

static void log_line(void *user, const char *line)
{
    (void)user;
    printf("  %s\n", line);
    fflush(stdout);
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
    arms_mask sel = arms_opened_joints(g_arms);
    char err[256];
    int j;
    if (need_session()) return -1;
    if (argc > 1 && arms_parse_joints(argv[1], arms_opened_joints(g_arms), &sel, err, sizeof(err)) != 0)
        return failed("%s", err);
    printf("%-3s %-20s %-11s %-8s %-6s %-6s %-6s %10s %10s %9s\n", "关节", "名称", "臂/位置", "状态", "AL",
           "SW", "ERR", "pos[rad]", "vel[rad/s]", "tor[N.m]");
    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
        arms_joint_state s;
        char where[16];
        if (!(sel & ARMS_MASK_JOINT(j))) continue;
        arms_read_joint(g_arms, j, &s);
        snprintf(where, sizeof(where), "%s/%d", s.arm == 0 ? "L" : "R", s.slave);
        printf("%-3d %-20s %-11s %-8s 0x%02x   0x%04x 0x%04x %10.4f %10.4f %9.3f\n", j, s.name ? s.name : "-",
               where, arms_joint_mode_name(s.mode), s.al_state, s.status_word, s.error_code, s.position,
               s.velocity, s.torque);
    }
    return ok();
}

/* ---- bus ---- */
static void print_stats(const arms_bus_stats *st)
{
    int i;
    printf("周期 %u Hz  已运行 %llu 周期  平均 %.1f us  最短 %.1f  最长 %.1f  抖动RMS %.2f us\n", st->cycle_hz,
           (unsigned long long)st->cycles, st->period.mean, st->period.min, st->period.max, st->period.rms_jitter);
    printf("唤醒最大延迟 %.1f us  单周期最大耗时 %.1f us  超时(>1.5T) %llu  跳过周期 %llu\n", st->wake_late_us_max,
           st->work_us_max, (unsigned long long)st->overruns, (unsigned long long)st->skipped);
    for (i = 0; i < ARMS_ARM_COUNT; ++i) {
        const arms_arm_stats *a = &st->arm[i];
        if (!a->present) {
            printf("%-5s 未打开\n", arms_arm_name(i));
            continue;
        }
        printf("%-5s %s 主站%d 链路%s  从站 OP %d/%d(总线上 %d)  WKC %u/%u  异常 %llu (最长连续 %u)  发送错误 %llu"
               "  收 %.1f us 发 %.1f us%s\n",
               arms_arm_name(i), a->interface, a->master_index, a->link_up ? "UP" : "DOWN", a->slaves_op,
               a->slaves_expected, a->slaves_found, a->wkc_last, a->wkc_expected, (unsigned long long)a->wkc_bad,
               a->wkc_bad_consecutive_max, (unsigned long long)a->send_errors, a->recv_us_max, a->send_us_max,
               a->comm_failed ? "  [超过容错上限]" : "");
    }
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
    return ok();
}

/* ---- commtest ---- */
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

static int cmd_commtest(int argc, char **argv)
{
    char *end;
    long seconds = strtol(argv[1], &end, 10);
    arms_bus_stats st;
    long t;
    int i;
    bool pass = true;
    (void)argc;
    if (need_session()) return -1;
    if (*end || seconds < 1 || seconds > 86400) return failed("秒数必须是 1..86400 的整数");
    arms_reset_stats(g_arms);
    usleep(20000);
    printf("失能通信测试 %ld 秒，回车或 Ctrl+C 提前结束...\n", seconds);
    g_interrupted = 0;
    for (t = 1; t <= seconds; ++t) {
        if (wait_or_enter(1000) || g_interrupted) {
            printf("已提前结束（%ld 秒）\n", t - 1);
            break;
        }
        if (t % 10 == 0 || t == seconds) {
            arms_get_stats(g_arms, &st);
            printf("  %3ld s  周期 %llu  WKC异常 左 %llu 右 %llu  周期最长 %.1f us\n", t,
                   (unsigned long long)st.cycles, (unsigned long long)st.arm[0].wkc_bad,
                   (unsigned long long)st.arm[1].wkc_bad, st.period.max);
        }
    }
    arms_get_stats(g_arms, &st);
    print_stats(&st);
    for (i = 0; i < ARMS_ARM_COUNT; ++i)
        if (st.arm[i].present && (st.arm[i].wkc_bad || st.arm[i].send_errors || st.arm[i].comm_failed)) pass = false;
    if (st.overruns || st.skipped) pass = false;
    if (pass) {
        printf("结果: 通过（无 WKC 异常、无发送错误、无超时周期）\n");
        return ok();
    }
    return failed("通信测试未通过（见上方统计）");
}

/* ---- watch ---- */
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
    printf("持续刷新，回车或 Ctrl+C 结束\n");
    g_interrupted = 0;
    for (;;) {
        int j;
        printf("\r");
        for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
            arms_joint_state s;
            if (!(sel & ARMS_MASK_JOINT(j))) continue;
            arms_read_joint(g_arms, j, &s);
            printf("%d:%+.3f%s ", j, s.position, s.mode == ARMS_JS_FAULT ? "!" : "");
        }
        fflush(stdout);
        if (wait_or_enter((unsigned int)(1000 / hz)) || g_interrupted) break;
    }
    printf("\n");
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
    printf("关节映射:\n");
    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
        arms_joint_state s;
        if (!(arms_opened_joints(g_arms) & ARMS_MASK_JOINT(j))) continue;
        arms_read_joint(g_arms, j, &s);
        printf("  %d %-20s %s 臂 slave %d\n", j, s.name, arms_arm_name(s.arm), s.slave);
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

static int cmd_help(int argc, char **argv);

static const command commands[] = {
    {"help", "通用", "[命令]", 0, 1, "列出所有命令，或显示某个命令的详细用法",
     "  不带参数: 按分组列出命令\n  带命令名: 等同于 <命令> -h", "  help\n  help commtest", cmd_help},
    {"status", "查看", "[关节]", 0, 1, "显示关节状态表（位置/速度/力矩、状态、AL、状态字、错误码）",
     "  关节写法: 21 | l_wrist_z | left | right | all | 15-18 | 21,28（默认全部已打开关节）\n"
     "  状态: offline / preop / safeop / locked（OP，伺服失能）/ enabled / FAULT",
     "  status\n  status left\n  status 21", cmd_status},
    {"watch", "查看", "[关节] [刷新Hz]", 0, 2, "持续刷新显示关节位置，回车结束", "  刷新频率 1..50 Hz，默认 5",
     "  watch left 10", cmd_watch},
    {"bus", "查看", "[reset]", 0, 1, "显示两条臂的通信统计；reset 清零",
     "  周期: 平均/最短/最长间隔、抖动、唤醒延迟、超时周期、跳过周期\n"
     "  每条臂: 链路、OP 从站数、WKC、异常次数、最长连续异常、发送错误、收发耗时",
     "  bus\n  bus reset", cmd_bus},
    {"info", "查看", "", 0, 0, "显示版本、配置文件、周期、实时 CPU、关节与总线位置的对应关系", "", "  info",
     cmd_info},
    {"commtest", "测试", "<秒>", 1, 1, "伺服失能状态下的通信质量测试，结束时给出结论",
     "  先清零统计，再运行指定秒数，每 10 秒打印一次进度\n"
     "  通过条件: WKC 异常 0 次、发送错误 0 次、无超时周期、无跳过周期",
     "  commtest 300", cmd_commtest},
    {"version", "通用", "", 0, 0, "显示版本", "", "  version", cmd_version},
    {"quit", "通用", "", 0, 0, "退出：停止通信并释放主站", "  也可以用 exit 或 q", "  quit", cmd_quit},
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

static int cmd_help(int argc, char **argv)
{
    static const char *groups[] = {"查看", "测试", "通用"};
    size_t g, i;
    if (argc > 1) {
        const command *c = find_command(argv[1]);
        if (!c) return failed("没有命令 %s；输入 help 查看全部命令", argv[1]);
        print_detail(c);
        return 0;
    }
    for (g = 0; g < sizeof(groups) / sizeof(groups[0]); ++g) {
        printf("[%s]\n", groups[g]);
        for (i = 0; i < COMMAND_COUNT; ++i) {
            const command *c = &commands[i];
            char head[96];
            if (strcmp(c->group, groups[g]) != 0) continue;
            snprintf(head, sizeof(head), "%s %s", c->name, c->args);
            printf("  %-24s %s\n", head, c->brief);
        }
    }
    printf("关节写法: 21 | l_wrist_z | left | right | all | 15-18 | 21,28\n");
    printf("任一命令加 -h 查看详细用法，例如: commtest -h\n");
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
    return c->fn(argc, argv);
}

static int split(char *line, char **argv, int max)
{
    int n = 0;
    char *save = NULL, *tok;
    for (tok = strtok_r(line, " \t\r\n", &save); tok && n < max; tok = strtok_r(NULL, " \t\r\n", &save))
        argv[n++] = tok;
    return n;
}

static void build_prompt(char *buf, size_t size)
{
    arms_bus_stats st;
    char hz[16];
    int j, locked = 0, fault = -1, opened = 0;
    arms_get_stats(g_arms, &st);
    hz_short(st.cycle_hz, hz, sizeof(hz));
    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) {
        arms_joint_state s;
        if (!(arms_opened_joints(g_arms) & ARMS_MASK_JOINT(j))) continue;
        opened++;
        arms_read_joint(g_arms, j, &s);
        if (s.mode == ARMS_JS_LOCKED) locked++;
        if (s.mode == ARMS_JS_FAULT && fault < 0) fault = j;
    }
    {
        char state[32];
        if (fault >= 0) snprintf(state, sizeof(state), "FAULT:%d", fault);
        else if (locked == opened) snprintf(state, sizeof(state), "locked");
        else snprintf(state, sizeof(state), "%d/%d locked", locked, opened);
        snprintf(buf, size, "arms %s L%d/%d R%d/%d %s%s> ", hz, st.arm[0].slaves_op, st.arm[0].slaves_expected,
                 st.arm[1].slaves_op, st.arm[1].slaves_expected, state,
                 (st.arm[0].comm_failed || st.arm[1].comm_failed) ? " COMM!" : "");
    }
}

static void usage(FILE *out)
{
    fprintf(out,
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
            "  -e, --exec CMD       执行命令后退出，例如 -e \"commtest 300\"（可重复，按顺序执行）\n"
            "      --mock           使用模拟后端，不访问硬件，不需要 root\n"
            "      --no-power       不操作电机电源（电源已由其他程序打开时使用）\n"
            "      --keep-power     退出时不断电\n"
            "      --power-wait MS  上电后等待驱动器启动的时间（默认 3000）\n"
            "      --no-rt-setup    不配置实时运行环境（中断绑定、CPU 隔离）\n"
            "  -h, --help           显示本帮助\n"
            "  -V, --version        显示版本\n"
            "\n"
            "关节编号: 15-21 左臂（肩y/x/z、肘y、腕x/y/z），22-28 右臂，顺序相同。\n"
            "示例:\n"
            "  arms_test                           上电、扫描两臂、预热，进入交互终端\n"
            "  arms_test -f 2000 -j left           只打开左臂，2 kHz\n"
            "  arms_test -e \"commtest 300\"         通信测试 300 秒后退出\n"
            "  arms_test --mock                    使用模拟后端\n"
            "进入终端后输入 help 查看命令，任一命令加 -h 查看详细用法。\n");
}

int main(int argc, char **argv)
{
    static const struct option long_opts[] = {
        {"freq", required_argument, NULL, 'f'},   {"joints", required_argument, NULL, 'j'},
        {"config", required_argument, NULL, 'c'}, {"exec", required_argument, NULL, 'e'},
        {"mock", no_argument, NULL, 'm'},         {"rt-cpu", required_argument, NULL, 'r'},
        {"power", no_argument, NULL, 'p'},        {"help", no_argument, NULL, 'h'},
        {"version", no_argument, NULL, 'V'},      {"power-wait", required_argument, NULL, 'w'},
        {"keep-power", no_argument, NULL, 'k'},   {"no-power", no_argument, NULL, 'P'},
        {"no-rt-setup", no_argument, NULL, 'R'},  {NULL, 0, NULL, 0},
    };
    arms_options opt;
    const char *execs[16];
    int nexec = 0, ch, rc = 0, i;
    bool power = true, keep_power = false, powered = false, rt_setup = true;
    unsigned int power_wait = 3000;
    char err[256];
    struct sigaction sa;
    arms_status st;

    arms_options_init(&opt);
    opt.mode = ARMS_MODE_STANDALONE;
    opt.log = log_line;
    while ((ch = getopt_long(argc, argv, "f:j:r:c:e:hV", long_opts, NULL)) != -1) {
        switch (ch) {
        case 'f': {
            char *end;
            long v = strtol(optarg, &end, 10);
            if (*end || v <= 0) {
                fprintf(stderr, "错误: --freq 需要整数，例如 1000 或 2000\n");
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
        case 'm': opt.backend = ARMS_BACKEND_MOCK; break;
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
        case 'R': rt_setup = false; break;
        case 'k': keep_power = true; break;
        case 'w': {
            char *end;
            long v = strtol(optarg, &end, 10);
            if (*end || v < 0 || v > 60000) {
                fprintf(stderr, "错误: --power-wait 需要 0..60000 的毫秒数\n");
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

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_sigint;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    printf("arms_test %s\n", arms_version());
    if (opt.backend == ARMS_BACKEND_MOCK) {
        power = false;
        rt_setup = false;
    } else {
        if (arms_env_start_master() != 0) return 1;
        if (rt_setup && arms_env_rt_apply(opt.config_path, opt.rt_cpu >= 0 ? opt.rt_cpu : -2) < 0) return 1;
        if (power) {
            if (arms_power_on(power_wait) != 0) {
                arms_env_rt_revert();
                return 1;
            }
            powered = true;
            opt.scan_timeout_ms = 20000;
        }
    }
    printf("启动: %s后端，扫描并预热，全部关节保持伺服失能\n", opt.backend == ARMS_BACKEND_MOCK ? "模拟" : "EtherLab ");
    st = arms_open(&opt, &g_arms);
    if (st != ARMS_OK) {
        fprintf(stderr, "启动失败: %s\n  原因: %s\n", arms_strerror(st), arms_last_error());
        if (powered && !keep_power) arms_power_off();
        arms_env_rt_revert();
        return 1;
    }
    {
        arms_bus_stats bs;
        arms_get_stats(g_arms, &bs);
        print_stats(&bs);
    }
    if (nexec > 0) {
        for (i = 0; i < nexec && !g_interrupted; ++i) {
            char line[512], *av[16];
            int ac;
            snprintf(line, sizeof(line), "%s", execs[i]);
            ac = split(line, av, 16);
            printf(">> %s\n", execs[i]);
            if (run_command(ac, av) != 0) rc = 1;
        }
    } else {
        printf("就绪。输入 help 查看命令，quit 退出。\n");
        while (!g_quit) {
            char prompt[128], line[512], *av[16];
            int ac;
            build_prompt(prompt, sizeof(prompt));
            printf("%s", prompt);
            fflush(stdout);
            g_interrupted = 0;
            if (!fgets(line, sizeof(line), stdin)) {
                if (g_interrupted && !feof(stdin)) {
                    clearerr(stdin);
                    printf("\n（Ctrl+C：输入 quit 退出）\n");
                    continue;
                }
                printf("\n");
                break;
            }
            ac = split(line, av, 16);
            run_command(ac, av);
        }
    }
    printf("退出: 停止通信并释放主站\n");
    arms_close(g_arms);
    if (powered) {
        if (keep_power) printf("电源: 保持上电（--keep-power）\n");
        else arms_power_off();
    }
    arms_env_rt_revert();
    printf("已退出\n");
    return rc;
}
