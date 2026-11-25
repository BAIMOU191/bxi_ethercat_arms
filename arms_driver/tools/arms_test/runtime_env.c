/**
 * @file runtime_env.c
 * @brief arms_test 运行环境准备：root 权限、EtherLab 主站、实时运行环境
 */
#define _GNU_SOURCE
#include "runtime_env.h"

#include <errno.h>
#include <fcntl.h>
#include <libgen.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#include "tlog.h"

#define MASTER_PARAM "/sys/module/ec_master/parameters/main_devices"
#define MASTER_INIT "/etc/init.d/ethercat"
#define RT_STATE "/run/arms_rt_setup.state"
#define RT_LOG "/run/arms_rt_setup.log"

static char g_rt_script[PATH_MAX];
static bool g_rt_applied;

static int exe_path(char *out, size_t size)
{
    ssize_t n = readlink("/proc/self/exe", out, size - 1);
    if (n <= 0) return -1;
    out[n] = '\0';
    return 0;
}

int arms_env_ensure_root(int argc, char **argv)
{
    char exe[PATH_MAX];
    char **args;
    int i;

    if (geteuid() == 0) return 0;
    if (exe_path(exe, sizeof(exe)) != 0) return -1;
    args = calloc((size_t)argc + 2, sizeof(char *));
    if (!args) return -1;
    args[0] = "sudo";
    args[1] = exe;
    for (i = 1; i < argc; ++i) args[i + 1] = argv[i];
    tlog("需要 root 权限，通过 sudo 启动");
    execvp("sudo", args);
    tlog_err("无法执行 sudo: %s", strerror(errno));
    free(args);
    return -1;
}

static int run(const char *cmd)
{
    int rc = system(cmd);
    if (rc == -1) return -1;
    return WIFEXITED(rc) ? WEXITSTATUS(rc) : -1;
}

int arms_env_start_master(void)
{
    int i;
    if (access(MASTER_PARAM, R_OK) == 0) return 0;
    if (access(MASTER_INIT, X_OK) != 0) {
        tlog_err("EtherLab 主站未加载，且找不到 %s", MASTER_INIT);
        return -1;
    }
    tlog("启动 EtherLab 主站");
    if (run(MASTER_INIT " start > /dev/null 2>&1") != 0) {
        tlog_err("EtherLab 主站启动失败（配置文件 /etc/sysconfig/ethercat）");
        return -1;
    }
    for (i = 0; i < 50 && access(MASTER_PARAM, R_OK) != 0; ++i) usleep(100000);
    if (access(MASTER_PARAM, R_OK) != 0) {
        tlog_err("EtherLab 主站启动后仍不可用");
        return -1;
    }
    tlog("EtherLab 主站已就绪");
    return 0;
}

static int locate_script(void)
{
    char exe[PATH_MAX], dir[PATH_MAX];
    const char *candidates[] = {"%s/../scripts/rt_setup.sh", "%s/scripts/rt_setup.sh",
                                "%s/../share/arms_driver/scripts/rt_setup.sh",
                                "%s/../../share/arms_ros/scripts/rt_setup.sh"};
    size_t i;
    if (exe_path(exe, sizeof(exe)) != 0) return -1;
    snprintf(dir, sizeof(dir), "%s", dirname(exe));
    for (i = 0; i < sizeof(candidates) / sizeof(candidates[0]); ++i) {
        snprintf(g_rt_script, sizeof(g_rt_script), candidates[i], dir);
        if (access(g_rt_script, X_OK) == 0) return 0;
    }
    g_rt_script[0] = '\0';
    return -1;
}

static int run_rt(char *const args[])
{
    int fd = open(RT_LOG, O_NOFOLLOW | O_CLOEXEC | O_CREAT | O_TRUNC | O_WRONLY, 0600);
    if (fd < 0)
        return -1;
    pid_t pid = fork();
    if (pid == 0) {
        if (dup2(fd, 1) < 0 || dup2(fd, 2) < 0)
            _exit(126);
        close(fd);
        execv(g_rt_script, args);
        _exit(127);
    }
    close(fd);
    if (pid < 0)
        return -1;
    int status;
    while (waitpid(pid, &status, 0) < 0)
        if (errno != EINTR)
            return -1;
    return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}
int arms_env_rt_apply(const char *config, int rt_cpu)
{
    char cpu[16];
    char *args[10] = {g_rt_script, "apply"};
    int n = 2;

    if (locate_script() != 0) {
        tlog("实时环境: 未找到 rt_setup.sh，跳过");
        return 1;
    }
    if (access(RT_STATE, F_OK) == 0) {
        tlog("实时环境: 发现上次未还原的设置，先还原");
        if (run_rt((char *[]){g_rt_script, "revert", NULL}))
            return -1;
    }
    if (config) {
        args[n++] = "-c";
        args[n++] = (char *)config;
    }
    if (rt_cpu >= 0) {
        snprintf(cpu, sizeof(cpu), "%d", rt_cpu);
        args[n++] = "--rt-cpu";
        args[n++] = cpu;
        args[n++] = "--irq-cpu";
        args[n++] = cpu;
    }
    args[n] = NULL;
    if (run_rt(args) != 0) {
        run_rt((char *[]){g_rt_script, "revert", NULL});
        tlog_err("实时环境配置失败，详见 " RT_LOG);
        return -1;
    }
    g_rt_applied = true;
    {
        FILE *f = fopen(RT_LOG, "r");
        char line[256];
        if (f) {
            while (fgets(line, sizeof(line), f)) {
                size_t n = strlen(line);
                if (strncmp(line, "配置:", strlen("配置:")) != 0) continue;
                if (n > 0 && line[n - 1] == '\n') line[n - 1] = '\0';
                tlog("实时环境:%s", line + strlen("配置:"));
            }
            fclose(f);
        }
    }
    return 0;
}

void arms_env_rt_revert(void)
{

    if (!g_rt_applied || !g_rt_script[0]) return;
    if (run_rt((char *[]){g_rt_script, "revert", NULL}) == 0)
        tlog("实时环境: 已还原");
    else tlog_err("实时环境还原失败，需执行 sudo %s revert", g_rt_script);
    g_rt_applied = false;
}
