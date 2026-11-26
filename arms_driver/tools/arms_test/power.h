#ifndef ARMS_TEST_POWER_H
#define ARMS_TEST_POWER_H

/*
 * 电机总电源与其他程序（如腰部控制程序）共用，按“谁接通谁断开”管理：
 *   ARMS_POWER_OWNED     本程序把电源从断开接通，退出或保护时由本程序断开
 *   ARMS_POWER_EXTERNAL  PCI 正被其他程序使用，或电源已由其他程序接通；本程序不断开电源
 * 本程序接通电源后即释放 PCI（停止 CAN、解除锁，不改动 GPIO），其他程序可以随后打开 PCI。
 */
enum { ARMS_POWER_OWNED = 0, ARMS_POWER_EXTERNAL = 1 };

/* 返回 ARMS_POWER_OWNED、ARMS_POWER_EXTERNAL，失败返回 -1；wait_ms > 0 时接通后额外等待 */
int arms_power_on(unsigned int wait_ms);

/*
 * 断开本程序接通的电源。返回 0 已断开或本程序不负责断电；1 其他程序此时正在使用 PCI，未断电，
 * 并放弃断电责任；-1 断电失败（保留责任，可重试）。
 */
int arms_power_off(void);

/* 本程序当前是否负责断电 */
int arms_power_owned(void);

int arms_power_available(void);

#endif
