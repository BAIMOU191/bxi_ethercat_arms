#ifndef ARMS_TEST_MOTION_H
#define ARMS_TEST_MOTION_H

#include <signal.h>
#include "arms/arms.h"

/* 总线打开后启动；释放总线前必须停止并等待后台线程退出。 */
int motion_start(arms_t *arms, volatile sig_atomic_t *interrupted, bool batch);
void motion_stop(void);
void motion_cancel_refresh(arms_mask mask);
int motion_hold(arms_mask mask);
int motion_wait_or_enter(unsigned int ms);
arms_mask motion_enabled(void);
/* 电机已主动断电时，通信中断按正常事件提示，不打印诊断快照 */
void motion_set_power_off(bool off);
/* 离线保护触发时在监视线程中调用一次（用于切断电机电源） */
void motion_set_trip_handler(void (*fn)(void));

/* 指令直接下发，不做轨迹插值和限幅；kp、kd 由调用方给出。 */
int motion_mit(arms_mask mask, arms_command cmd);
int motion_swing(arms_mask mask, double amplitude, double period, long count, double kp, double kd);
#endif
