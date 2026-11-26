/**
 * @file arms_power.c
 * @brief 电机总电源开关（BXI PCI SDK），不启动 EtherCAT 主站
 *
 * 用法: sudo arms_power on | off | status
 * on 后进程直接退出，电源保持接通；off 断电后释放 PCI。
 */
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "bxi_pci_drv.h"

static int ignore_can(void *arg, canfd_packet *msg)
{
    (void)arg;
    (void)msg;
    return 0;
}

static void print_state(void)
{
    int s = motor_pwr_get();
    if (s < 0) printf("电机电源: 状态读取失败\n");
    else printf("电机电源: %s\n", s ? "接通" : "断开");
}

int main(int argc, char **argv)
{
    const char *op = argc == 2 ? argv[1] : "";
    if (strcmp(op, "on") && strcmp(op, "off") && strcmp(op, "status")) {
        fprintf(stderr, "用法: sudo %s on | off | status\n", argv[0]);
        return 2;
    }
    /* 非 root 运行时 SDK 会以当前用户创建 /tmp/bxi_pci.lock，之后 root 因 fs.protected_regular 无法打开该文件 */
    if (geteuid() != 0) {
        fprintf(stderr, "需要 root: sudo %s %s\n", argv[0], op);
        return 1;
    }
    if (bxi_pci_init(ignore_can, NULL, -1) == -1) {
        fprintf(stderr, "bxi_pci_init 失败：PCI 正被其他程序占用（arms_test、arms_node、motor_console 等），"
                        "或 /tmp/bxi_pci.lock 属于其他用户\n");
        return 1;
    }
    if (!strcmp(op, "on")) {
        if (motor_pwr_set(1u) < 0) {
            fprintf(stderr, "motor_pwr_set(1) 失败\n");
            return 1;
        }
        print_state();
        return 0; /* 不调用 bxi_pci_exit，电源保持接通 */
    }
    if (!strcmp(op, "off")) {
        if (motor_pwr_set(0u) < 0) {
            fprintf(stderr, "motor_pwr_set(0) 失败，电源状态未知\n");
            return 1;
        }
        print_state();
        bxi_pci_exit();
        return 0;
    }
    print_state();
    return 0;
}
