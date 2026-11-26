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

#define FEEDBACK_TIMEOUT_MS 5000
#define FEEDBACK_POLL_MS 100

/* motor_pwr_get 读取电源反馈（PCI GPIO 0），motor_pwr_set 驱动控制输出（GPIO 2）；反馈滞后于控制 */
static void print_state(void)
{
    int s = motor_pwr_get();
    if (s < 0) printf("电机电源反馈: 读取失败\n");
    else printf("电机电源反馈: %s\n", s ? "接通" : "断开");
}

/* 等待反馈与请求一致；返回 0 一致，-1 超时或读取失败 */
static int wait_feedback(int want)
{
    for (int ms = 0; ms <= FEEDBACK_TIMEOUT_MS; ms += FEEDBACK_POLL_MS) {
        int s = motor_pwr_get();
        if (s < 0) {
            printf("电机电源反馈: 读取失败\n");
            return -1;
        }
        if ((s != 0) == (want != 0)) {
            printf("电机电源反馈: %s（%d ms 后确认）\n", want ? "接通" : "断开", ms);
            return 0;
        }
        usleep(FEEDBACK_POLL_MS * 1000);
    }
    printf("电机电源反馈: %d ms 内未变为%s\n", FEEDBACK_TIMEOUT_MS, want ? "接通" : "断开");
    return -1;
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
        return wait_feedback(1) ? 1 : 0; /* 不调用 bxi_pci_exit，电源保持接通 */
    }
    if (!strcmp(op, "off")) {
        if (motor_pwr_set(0u) < 0) {
            fprintf(stderr, "motor_pwr_set(0) 失败，电源状态未知\n");
            return 1;
        }
        int rc = wait_feedback(0) ? 1 : 0;
        bxi_pci_exit();
        return rc;
    }
    print_state();
    return 0;
}
