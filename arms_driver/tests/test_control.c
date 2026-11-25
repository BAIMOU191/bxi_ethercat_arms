/* 单元测试：使能零指令、MIT 指令原样下发、软位置限位、超时、故障与复位（模拟后端，独立模式） */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "arms/arms.h"

static int failures;
#define CHECK(cond, msg)                                                   \
    do {                                                                   \
        if (!(cond)) {                                                     \
            fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, msg);   \
            failures++;                                                    \
        }                                                                  \
    } while (0)

static arms_joint_state rd(arms_t *a, int j)
{
    arms_joint_state s;
    arms_read_joint(a, j, &s);
    return s;
}

int main(int argc, char **argv)
{
    arms_options opt;
    arms_t *a = NULL;
    arms_joint_limits lim;
    arms_command cmd;
    arms_joint_state s;
    int j;

    if (argc < 2) {
        fprintf(stderr, "用法: %s <arms.json>\n", argv[0]);
        return 2;
    }
    /* 左臂 slave 0（关节 15）在第 3000 周期后报故障 */
    setenv("ARMS_MOCK_FAULT_CYCLE", "3000", 1);
    arms_options_init(&opt);
    opt.config_path = argv[1];
    opt.backend = ARMS_BACKEND_MOCK;
    opt.mode = ARMS_MODE_STANDALONE;
    opt.cycle_hz = 1000;
    opt.rt_cpu = -1;
    CHECK(arms_open(&opt, &a) == ARMS_OK, arms_last_error());
    if (!a) return 1;

    /* 未使能时不接受指令 */
    cmd.position = 0.0; cmd.velocity = 0.0; cmd.torque = 0.0; cmd.kp = 1.0; cmd.kd = 0.1;
    CHECK(arms_set_command(a, 17, &cmd) == ARMS_E_STATE, "command before enable");

    /* 使能后零指令：kp=kd=0，速度、前馈为 0 */
    s = rd(a, 17);
    CHECK(arms_enable(a, ARMS_MASK_JOINT(17) | ARMS_MASK_JOINT(28), 0) == ARMS_OK, arms_last_error());
    CHECK(rd(a, 17).mode == ARMS_JS_ENABLED, "17 enabled");
    CHECK(rd(a, 28).mode == ARMS_JS_ENABLED, "28 enabled");
    usleep(20000);
    s = rd(a, 17);
    CHECK(s.cmd_kp == 0.0 && s.cmd_kd == 0.0, "zero gains on enable");
    CHECK(s.cmd_velocity == 0.0 && s.cmd_torque == 0.0, "zero vel/tff on enable");
    CHECK(rd(a, 22).mode == ARMS_JS_LOCKED, "22 untouched");

    /* 位置指令下一周期原样生效，不做插值 */
    CHECK(arms_get_limits(a, 17, &lim) == ARMS_OK, "limits");
    CHECK(lim.pos_limit_min < 0.0 && lim.pos_limit_max > 0.0 && lim.vel_limit > 0.0 && lim.tor_limit > 0.0,
          "limits from joint_defaults");
    cmd.position = 0.2; cmd.velocity = 0.0; cmd.torque = 0.0; cmd.kp = 1.0; cmd.kd = 0.1;
    CHECK(arms_set_command(a, 17, &cmd) == ARMS_OK, arms_last_error());
    usleep(10000);
    s = rd(a, 17);
    CHECK(fabs(s.cmd_position - 0.2) < 1e-6, "target applied immediately");
    usleep(500000);
    s = rd(a, 17);
    CHECK(fabs(s.position - 0.2) < 0.02, "mock follows");

    /* 不截断：超出软限位的目标和很大的增益原样下发 */
    cmd.position = lim.pos_limit_max + 1.0;
    cmd.kp = 500.0;
    cmd.kd = 0.5;
    CHECK(arms_set_command(a, 17, &cmd) == ARMS_OK, arms_last_error());
    usleep(10000);
    s = rd(a, 17);
    CHECK(fabs(s.cmd_position - cmd.position) < 1e-6, "pos not clamped");
    CHECK(fabs(s.cmd_kp - 500.0) < 1e-6, "kp not clamped");

    /* kp、kd 为负或 NaN 时按 0 下发 */
    cmd.position = 0.2;
    cmd.kp = -1.0;
    cmd.kd = NAN;
    CHECK(arms_set_command(a, 17, &cmd) == ARMS_OK, "negative kp / nan kd accepted");
    usleep(10000);
    s = rd(a, 17);
    CHECK(s.cmd_kp == 0.0 && s.cmd_kd == 0.0, "negative / nan gains sent as 0");

    /* 位置、速度、前馈必须为有限值 */
    cmd.position = NAN;
    CHECK(arms_set_command(a, 17, &cmd) == ARMS_E_ARG, "nan position rejected");
    cmd.position = 0.0;
    cmd.torque = INFINITY;
    CHECK(arms_set_command(a, 17, &cmd) == ARMS_E_ARG, "inf torque rejected");
    cmd.torque = 0.0;

    /* 软位置限位：越过上限后屏蔽正向速度和前馈，反向不受影响 */
    cmd.position = lim.pos_limit_max + 0.5;
    cmd.velocity = 0.1;
    cmd.torque = 0.2;
    cmd.kp = 5.0;
    cmd.kd = 0.1;
    for (j = 0; j < 300 && rd(a, 17).position <= lim.pos_limit_max + 0.05; ++j) {
        arms_set_command(a, 17, &cmd);
        usleep(10000);
    }
    arms_set_command(a, 17, &cmd);
    usleep(10000);
    s = rd(a, 17);
    CHECK(s.position > lim.pos_limit_max, "moved past soft limit");
    CHECK(s.cmd_pos_blocked, "blocked flag");
    CHECK(s.cmd_velocity == 0.0 && s.cmd_torque == 0.0, "outward vel/tff blocked");
    CHECK(fabs(s.cmd_position - cmd.position) < 1e-6, "position still passed through");
    cmd.velocity = -0.1;
    cmd.torque = -0.2;
    arms_set_command(a, 17, &cmd);
    usleep(10000);
    s = rd(a, 17);
    CHECK(!s.cmd_pos_blocked, "inward command not blocked");
    CHECK(fabs(s.cmd_velocity + 0.1) < 1e-6 && fabs(s.cmd_torque + 0.2) < 1e-6, "inward vel/tff passed");

    /* 超时：速度与前馈清零 */
    cmd.position = 0.2;
    cmd.kp = 1.0;
    cmd.kd = 0.1;
    cmd.velocity = 0.1;
    cmd.torque = 0.2;
    CHECK(arms_set_command(a, 17, &cmd) == ARMS_OK, arms_last_error());
    usleep(10000);
    CHECK(fabs(rd(a, 17).cmd_velocity - 0.1) < 1e-6, "velocity applied");
    usleep(200000);
    s = rd(a, 17);
    CHECK(s.cmd_timeout, "timeout flag");
    CHECK(s.cmd_velocity == 0.0 && s.cmd_torque == 0.0, "timeout hold");
    CHECK(s.mode == ARMS_JS_ENABLED, "still enabled after hold timeout");

    /* 失能 */
    CHECK(arms_disable(a, ARMS_MASK_JOINT(28)) == ARMS_OK, "disable");
    usleep(20000);
    CHECK(rd(a, 28).mode == ARMS_JS_LOCKED, "28 locked");

    /* 故障：关节 15 使能后在第 3000 周期报故障，自动失能；复位后可再次使能 */
    CHECK(arms_enable(a, ARMS_MASK_JOINT(15), 0) == ARMS_OK, arms_last_error());
    for (j = 0; j < 400 && rd(a, 15).mode != ARMS_JS_FAULT; ++j) usleep(10000);
    s = rd(a, 15);
    CHECK(s.mode == ARMS_JS_FAULT, "fault detected");
    CHECK(s.fault_count == 1, "fault counted");
    CHECK(s.error_code != 0, "error code");
    CHECK(arms_enable(a, ARMS_MASK_JOINT(15), 0) == ARMS_E_STATE, "enable refused while faulted");
    CHECK(arms_reset(a, ARMS_MASK_JOINT(15), 0) == ARMS_OK, arms_last_error());
    CHECK(rd(a, 15).mode == ARMS_JS_LOCKED, "locked after reset");
    CHECK(arms_enable(a, ARMS_MASK_JOINT(15), 0) == ARMS_OK, arms_last_error());

    /* 全部关节 */
    CHECK(arms_enable(a, ARMS_MASK_ALL, 0) == ARMS_OK, arms_last_error());
    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) CHECK(rd(a, j).mode == ARMS_JS_ENABLED, "all enabled");
    CHECK(arms_disable(a, ARMS_MASK_ALL) == ARMS_OK, "disable all");
    usleep(20000);
    for (j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; ++j) CHECK(rd(a, j).mode == ARMS_JS_LOCKED, "all locked");

    arms_close(a);
    if (failures) fprintf(stderr, "%d 项失败\n", failures);
    else printf("OK\n");
    return failures ? 1 : 0;
}
