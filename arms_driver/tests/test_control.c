/* 单元测试：使能、MIT 指令、限幅、超时、故障与复位（模拟后端，独立模式） */
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

    /* 使能后保持原位 */
    s = rd(a, 17);
    CHECK(arms_enable(a, ARMS_MASK_JOINT(17) | ARMS_MASK_JOINT(28), 0) == ARMS_OK, arms_last_error());
    CHECK(rd(a, 17).mode == ARMS_JS_ENABLED, "17 enabled");
    CHECK(rd(a, 28).mode == ARMS_JS_ENABLED, "28 enabled");
    CHECK(fabs(rd(a, 17).cmd_position - s.position) < 1e-3, "hold position on enable");
    CHECK(rd(a, 22).mode == ARMS_JS_LOCKED, "22 untouched");

    /* 关节 21 的模拟初始位置 0.7 在限位 0.5 之外：使能后保持原位，不被拉回 */
    s = rd(a, 21);
    CHECK(arms_enable(a, ARMS_MASK_JOINT(21), 0) == ARMS_OK, arms_last_error());
    usleep(20000);
    CHECK(fabs(rd(a, 21).cmd_position - s.position) < 1e-3, "out-of-range joint holds in place");
    CHECK(!rd(a, 21).cmd_limited, "out-of-range hold not limited");

    /* 位置指令下一周期原样生效，不做插值 */
    CHECK(arms_get_limits(a, 17, &lim) == ARMS_OK, "limits");
    cmd.position = 0.2;
    CHECK(arms_set_command(a, 17, &cmd) == ARMS_OK, arms_last_error());
    usleep(10000);
    s = rd(a, 17);
    CHECK(fabs(s.cmd_position - 0.2) < 1e-6, "target applied immediately");
    usleep(500000);
    s = rd(a, 17);
    CHECK(fabs(s.position - 0.2) < 0.02, "mock follows");

    /* 限幅 */
    cmd.position = lim.pos_max + 1.0;
    cmd.kp = lim.kp_max * 10.0;
    CHECK(arms_set_command(a, 17, &cmd) == ARMS_OK, arms_last_error());
    usleep(20000);
    s = rd(a, 17);
    CHECK(s.cmd_limited, "limited flag");
    CHECK(fabs(s.cmd_position - lim.pos_max) < 1e-6, "pos clamped");
    CHECK(fabs(s.cmd_kp - lim.kp_max) < 1e-6, "kp clamped");

    /* 无效指令 */
    cmd.position = NAN;
    CHECK(arms_set_command(a, 17, &cmd) == ARMS_E_ARG, "nan rejected");
    cmd.position = 0.0;
    cmd.kp = -1.0;
    CHECK(arms_set_command(a, 17, &cmd) == ARMS_E_ARG, "negative kp rejected");

    /* 超时：速度与前馈清零 */
    cmd.kp = 1.0;
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
