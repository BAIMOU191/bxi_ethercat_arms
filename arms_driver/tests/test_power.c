/* 用桩替代 PCI SDK，验证电源归属规则，不访问设备。 */
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/file.h>
#include <unistd.h>
#include "bxi_pci_drv.h"
#include "power.h"
#include "tlog.h"

static int inits, locks, unlocks, can_exits, exits, pwr = 0, fail_off_once;
int bxi_pci_init(canfd_rx_call fn, void *arg, int cpu) { (void)fn; (void)arg; (void)cpu; ++inits; return 0; }
int bxi_pci_exit(void) { ++exits; pwr = 0; return 0; }
int bxi_pci_lock(void) { ++locks; return 0; }
int bxi_pci_unlock(void) { ++unlocks; return 0; }
int canfd_exit(void) { ++can_exits; return 0; }
int motor_pwr_get(void) { return pwr; }
int motor_pwr_set(unsigned int on)
{
    if (!on && fail_off_once) {
        fail_off_once = 0;
        return -1;
    }
    pwr = (int)on;
    return 0;
}
void tlog(const char *fmt, ...) { (void)fmt; }
void tlog_err(const char *fmt, ...) { (void)fmt; }

#define CHECK(c, msg)                                \
    do {                                             \
        if (!(c)) {                                  \
            fprintf(stderr, "FAIL: %s\n", msg);      \
            return 1;                                \
        }                                            \
    } while (0)

int main(void)
{
    char path[] = "/tmp/arms_power_test_XXXXXX";
    int fd = mkstemp(path), other;
    CHECK(fd >= 0, "temp lock file");
    close(fd);
    setenv("ARMS_PCI_LOCK_FILE", path, 1);

    /* 电源断开时由本程序接通并负责断开；接通后立即释放 PCI，不调用 bxi_pci_exit */
    CHECK(arms_power_on(0) == ARMS_POWER_OWNED && pwr == 1 && arms_power_owned(), "owned power-on");
    CHECK(inits == 1 && can_exits == 1 && unlocks == 1 && exits == 0, "PCI released after power-on");
    /* 断电失败保留责任，可重试；成功后再次释放 PCI */
    fail_off_once = 1;
    CHECK(arms_power_off() == -1 && arms_power_owned() && pwr == 1, "failed power-off stays retryable");
    CHECK(arms_power_off() == 0 && pwr == 0 && !arms_power_owned(), "retry switches power off");
    CHECK(locks == 1 && unlocks == 2 && exits == 0, "re-lock for power-off, no bxi_pci_exit");

    /* 电源已由其他程序接通：不接管，断电为空操作 */
    pwr = 1;
    CHECK(arms_power_on(0) == ARMS_POWER_EXTERNAL && !arms_power_owned(), "external power not owned");
    CHECK(arms_power_off() == 0 && pwr == 1, "external power left on");

    /* 其他程序持有 PCI 锁：不初始化 PCI，按外部电源处理 */
    pwr = 0;
    other = open(path, O_RDWR);
    CHECK(other >= 0 && flock(other, LOCK_EX | LOCK_NB) == 0, "hold lock as another program");
    CHECK(arms_power_on(0) == ARMS_POWER_EXTERNAL && pwr == 0 && locks == 2, "busy PCI treated as external");

    /* 本程序接通后，断电时其他程序已在使用 PCI：放弃断电并保持电源 */
    flock(other, LOCK_UN);
    CHECK(arms_power_on(0) == ARMS_POWER_OWNED && pwr == 1, "owned again");
    CHECK(flock(other, LOCK_EX | LOCK_NB) == 0, "other program takes PCI");
    CHECK(arms_power_off() == 1 && pwr == 1 && !arms_power_owned(), "power kept while another program uses PCI");
    close(other);
    unlink(path);
    return 0;
}
