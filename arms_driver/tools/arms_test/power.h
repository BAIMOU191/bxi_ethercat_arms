#ifndef ARMS_TEST_POWER_H
#define ARMS_TEST_POWER_H

/* 打开电机总电源；wait_ms > 0 时再固定等待（扫描会自行等待总线稳定，通常传 0） */
int arms_power_on(unsigned int wait_ms);
int arms_power_off(void);
int arms_power_available(void);

#endif
