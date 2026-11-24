/* 单元测试：掉电后通信失败、自动恢复、故障自动复位、恢复后不自动使能（模拟后端） */
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

static arms_arm_stats left(arms_t *a)
{
    arms_bus_stats st;
    arms_get_stats(a, &st);
    return st.arm[ARMS_ARM_LEFT];
}

static arms_joint_mode mode(arms_t *a, int j)
{
    arms_joint_state s;
    arms_read_joint(a, j, &s);
    return s.mode;
}

int main(int argc, char **argv)
{
    arms_options opt;
    arms_t *a = NULL;
    int i;

    if (argc < 2) {
        fprintf(stderr, "用法: %s <arms.json>\n", argv[0]);
        return 2;
    }
    /* 左臂在第 2500 周期掉电，持续 1000 周期 */
    setenv("ARMS_MOCK_POWER_OFF", "2500,1000", 1);
    arms_options_init(&opt);
    opt.config_path = argv[1];
    opt.backend = ARMS_BACKEND_MOCK;
    opt.mode = ARMS_MODE_STANDALONE;
    opt.cycle_hz = 1000;
    opt.rt_cpu = -1;
    CHECK(arms_open(&opt, &a) == ARMS_OK, arms_last_error());
    if (!a) return 1;

    CHECK(arms_enable(a, ARMS_MASK_JOINT(15) | ARMS_MASK_JOINT(22), 0) == ARMS_OK, arms_last_error());

    for (i = 0; i < 400 && !left(a).comm_failed; ++i) usleep(10000);
    CHECK(left(a).comm_failed, "left comm failed");
    CHECK(left(a).comm_drops == 1, "drop counted");
    CHECK(mode(a, 15) != ARMS_JS_ENABLED, "15 disabled on drop");
    CHECK(arms_enable(a, ARMS_MASK_JOINT(16), 0) != ARMS_OK, "enable refused while offline");

    /* 默认 offline_action = power_off：触发离线保护，所有关节失能 */
    {
        arms_bus_stats st;
        arms_get_stats(a, &st);
        CHECK(st.tripped, "trip on offline");
        CHECK((st.trip_joints & ARMS_MASK_LEFT) == ARMS_MASK_LEFT && !(st.trip_joints & ARMS_MASK_RIGHT),
              "trip joints = left arm");
    }
    usleep(20000);
    CHECK(mode(a, 22) == ARMS_JS_LOCKED, "right arm disabled by trip");
    CHECK(arms_clear_trip(a) != ARMS_OK, "clear refused while offline");

    for (i = 0; i < 500 && left(a).comm_failed; ++i) usleep(10000);
    CHECK(!left(a).comm_failed, "left recovered");
    CHECK(left(a).comm_recoveries == 1, "recovery counted");
    usleep(100000);
    for (i = 15; i <= 21; ++i) CHECK(mode(a, i) == ARMS_JS_LOCKED, "locked after recovery (auto reset, not enabled)");
    CHECK(arms_enable(a, ARMS_MASK_LEFT, 0) != ARMS_OK, "enable refused while tripped");
    CHECK(arms_clear_trip(a) == ARMS_OK, arms_last_error());
    CHECK(arms_enable(a, ARMS_MASK_LEFT, 0) == ARMS_OK, arms_last_error());

    arms_close(a);
    if (failures) fprintf(stderr, "%d 项失败\n", failures);
    else printf("OK\n");
    return failures ? 1 : 0;
}
