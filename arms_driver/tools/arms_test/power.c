/**
 * @file power.c
 * @brief 电机总电源控制（BXI PCI SDK），与其他使用 PCI 的程序共用电源
 *
 * PCI SDK 以 flock 锁定 /tmp/bxi_pci.lock 实现独占。bxi_pci_exit 会把整个 GPIO 输出写 0，
 * 即断开电机电源，因此本程序从不对非本程序接通的电源调用它；释放 PCI 时只停止 CAN 并解除锁。
 */
#include "power.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <unistd.h>

#include "tlog.h"

#ifdef ARMS_HAVE_PCI_POWER
#include "bxi_pci_drv.h"

/* SDK 导出但未在头文件中声明的接口 */
int bxi_pci_lock(void);
int bxi_pci_unlock(void);
int canfd_exit(void);

#define PCI_LOCK_FILE "/tmp/bxi_pci.lock"

static int g_mapped;  /* 已调用 bxi_pci_init，GPIO 已映射 */
static int g_locked;  /* 持有 PCI 锁 */
static int g_can_on;  /* SDK 的 CAN 收发仍在运行 */
static int g_owned;   /* 电源由本程序接通 */

static int ignore_can(void *arg, canfd_packet *msg)
{
    (void)arg;
    (void)msg;
    return 0;
}

static const char *lock_path(void)
{
    const char *p = getenv("ARMS_PCI_LOCK_FILE"); /* 单元测试使用 */
    return p && *p ? p : PCI_LOCK_FILE;
}

/* 1 其他程序持有 PCI 锁；0 空闲；-1 无法判断（原因已输出） */
static int pci_busy(void)
{
    int fd = open(lock_path(), O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        if (errno == ENOENT) return 0;
        tlog_err("电源: 无法打开 %s: %s（文件属于其他用户时 root 也无法打开，需删除该文件）", lock_path(),
                 strerror(errno));
        return -1;
    }
    if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
        int busy = errno == EWOULDBLOCK;
        close(fd);
        if (busy) return 1;
        tlog_err("电源: 检查 PCI 锁失败: %s", strerror(errno));
        return -1;
    }
    flock(fd, LOCK_UN);
    close(fd);
    return 0;
}

/* 取得 PCI：首次映射时调用 bxi_pci_init，之后只重新加锁。返回 1 被占用，0 成功，-1 失败 */
static int pci_take(void)
{
    int busy;
    if (g_locked) return 0;
    busy = pci_busy();
    if (busy) return busy;
    if (!g_mapped) {
        if (bxi_pci_init(ignore_can, NULL, -1) == -1) {
            tlog_err("电源: bxi_pci_init 失败");
            return -1;
        }
        g_mapped = g_locked = g_can_on = 1;
        return 0;
    }
    if (bxi_pci_lock() != 0) return 1;
    g_locked = 1;
    return 0;
}

/* 释放 PCI 给其他程序：停止 CAN、解除锁，不改动 GPIO（电源保持原状） */
static void pci_release(void)
{
    if (g_can_on) {
        canfd_exit();
        g_can_on = 0;
    }
    if (g_locked) {
        bxi_pci_unlock();
        g_locked = 0;
    }
}

int arms_power_on(unsigned int wait_ms)
{
    int rc;
    if (g_owned) return ARMS_POWER_OWNED;
    rc = pci_take();
    if (rc < 0) return -1;
    if (rc > 0) {
        tlog("电源: PCI 正被其他程序使用，电机电源由该程序管理，本程序不接通也不断开");
        return ARMS_POWER_EXTERNAL;
    }
    if (motor_pwr_get() > 0) {
        pci_release();
        tlog("电源: 电机电源已由其他程序接通，本程序不接管，退出时不断电");
        return ARMS_POWER_EXTERNAL;
    }
    if (motor_pwr_set(1u) < 0) {
        pci_release();
        tlog_err("电源: motor_pwr_set(1) 失败");
        return -1;
    }
    g_owned = 1;
    pci_release();
    if (wait_ms) {
        tlog("电源: 已接通（由本程序负责断开），等待驱动器启动 %u ms", wait_ms);
        usleep(wait_ms * 1000u);
    } else {
        tlog("电源: 已接通（由本程序负责断开）");
    }
    return ARMS_POWER_OWNED;
}

int arms_power_off(void)
{
    int rc;
    if (!g_owned) return 0;
    rc = pci_take();
    if (rc < 0) return -1;
    if (rc > 0) {
        g_owned = 0;
        tlog("电源: 其他程序正在使用 PCI，电机电源保持接通，断电由该程序负责");
        return 1;
    }
    if (motor_pwr_set(0u) < 0) {
        tlog_err("电源: motor_pwr_set(0) 失败，电源状态未知");
        return -1; /* 保留 PCI 锁和断电责任，允许重试 */
    }
    g_owned = 0;
    pci_release();
    tlog("电源: 已断开");
    return 0;
}

int arms_power_owned(void) { return g_owned; }

int arms_power_available(void) { return 1; }

#else

int arms_power_on(unsigned int wait_ms)
{
    (void)wait_ms;
    tlog_err("电源: 本程序编译时没有 PCI 电源支持");
    return -1;
}

int arms_power_off(void) { return 0; }
int arms_power_owned(void) { return 0; }
int arms_power_available(void) { return 0; }

#endif
