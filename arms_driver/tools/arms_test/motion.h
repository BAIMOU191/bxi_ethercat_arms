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

/* NAN 表示使用每个关节的默认增益，move_s < 0 表示自动计算时间。 */
int motion_mit(arms_mask mask, arms_command cmd, double move_s);
int motion_home(arms_mask mask, double move_s);
int motion_swing(arms_mask mask, double amplitude, double period, long count, double kp, double kd);
#endif
