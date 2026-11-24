/* 用桩替代 PCI SDK：只验证断电失败后的重试，不访问设备。 */
#include <stdint.h>
#include <stdio.h>
#include "bxi_pci_drv.h"
#include "power.h"
#include "tlog.h"

static int off_attempts, closed;
int bxi_pci_init(canfd_rx_call fn, void *arg, int cpu) { (void)fn; (void)arg; (void)cpu; return 0; }
int bxi_pci_exit(void) { ++closed; return 0; }
int motor_pwr_set(unsigned int on) { return !on && ++off_attempts == 1 ? -1 : 0; }
void tlog(const char *fmt, ...) { (void)fmt; }
void tlog_err(const char *fmt, ...) { (void)fmt; }

int main(void)
{
    if (arms_power_on(0) || !arms_power_off() || closed || arms_power_off() || closed != 1 || off_attempts != 2) {
        fputs("failed power-off must remain retryable\n", stderr);
        return 1;
    }
    return 0;
}
