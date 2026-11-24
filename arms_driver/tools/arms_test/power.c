/**
 * @file power.c
 * @brief 电机总电源控制（BXI PCI SDK motor_pwr_set）
 */
#include "power.h"

#include <stdio.h>
#include <unistd.h>

#include "tlog.h"

#ifdef ARMS_HAVE_PCI_POWER
#include "bxi_pci_drv.h"

static int g_pci_open;

static int ignore_can(void *arg, canfd_packet *msg)
{
    (void)arg;
    (void)msg;
    return 0;
}

int arms_power_on(unsigned int wait_ms)
{
    if (!g_pci_open) {
        if (bxi_pci_init(ignore_can, NULL, -1) == -1) {
            tlog_err("电源: bxi_pci_init 失败（PCI 可能被 hw_lib_test_elf3 / hardware_elf3 占用，或权限不足）");
            return -1;
        }
        g_pci_open = 1;
    }
    if (motor_pwr_set(1u) < 0) {
        tlog_err("电源: motor_pwr_set(1) 失败");
        return -1;
    }
    if (wait_ms) {
        tlog("电源: 已请求上电，等待驱动器启动 %u ms", wait_ms);
        usleep(wait_ms * 1000u);
    } else {
        tlog("电源: 已请求上电");
    }
    return 0;
}

int arms_power_off(void)
{
    if (!g_pci_open) return 0;
    if (motor_pwr_set(0u) < 0) {
        tlog_err("电源: motor_pwr_set(0) 失败，请人工确认断电");
        return -1; /* 保留已打开的 PCI 句柄，允许重试断电 */
    }
    tlog("电源: 已请求断电（实际断电请以现场指示为准）");
    bxi_pci_exit();
    g_pci_open = 0;
    return 0;
}

int arms_power_available(void) { return 1; }

#else

int arms_power_on(unsigned int wait_ms)
{
    (void)wait_ms;
    tlog_err("电源: 本程序编译时没有 PCI 电源支持");
    return -1;
}

int arms_power_off(void) { return 0; }
int arms_power_available(void) { return 0; }

#endif
