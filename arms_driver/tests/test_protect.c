/* 单元测试：反馈超速保护（模拟后端，关节 15 使能后反馈速度 10 rad/s，超过 vel_fault） */
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
    arms_bus_stats st;
    int i;

    if (argc < 2) {
        fprintf(stderr, "用法: %s <arms.json>\n", argv[0]);
        return 2;
    }
    setenv("ARMS_MOCK_OVERSPEED_CYCLE", "1", 1);
    arms_options_init(&opt);
    opt.config_path = argv[1];
    opt.backend = ARMS_BACKEND_MOCK;
    opt.mode = ARMS_MODE_STANDALONE;
    opt.cycle_hz = 1000;
    opt.rt_cpu = -1;
    CHECK(arms_open(&opt, &a) == ARMS_OK, arms_last_error());
    if (!a) return 1;

    /* 关节 22 不受注入影响，用来确认保护作用于全部关节 */
    CHECK(arms_enable(a, ARMS_MASK_JOINT(15) | ARMS_MASK_JOINT(22), 0) == ARMS_OK, arms_last_error());
    for (i = 0; i < 200; ++i) {
        arms_get_stats(a, &st);
        if (st.protect.reason != ARMS_PROTECT_NONE) break;
        usleep(10000);
    }
    CHECK(st.protect.reason == ARMS_PROTECT_OVERSPEED, "overspeed protection");
    CHECK(st.protect.joint == 15, "protect joint");
    CHECK(st.protect.value > st.protect.limit, "protect value above limit");
    usleep(20000);
    CHECK(mode(a, 15) == ARMS_JS_LOCKED, "15 disabled");
    CHECK(mode(a, 22) == ARMS_JS_LOCKED, "22 disabled");
    CHECK(arms_enable(a, ARMS_MASK_JOINT(22), 0) != ARMS_OK, "enable refused while protected");

    /* reset 清除保护后可以重新使能未注入的关节 */
    CHECK(arms_reset(a, ARMS_MASK_JOINT(22), 0) == ARMS_OK, arms_last_error());
    usleep(20000);
    arms_get_stats(a, &st);
    CHECK(st.protect.reason == ARMS_PROTECT_NONE, "protection cleared");
    CHECK(arms_enable(a, ARMS_MASK_JOINT(22), 0) == ARMS_OK, arms_last_error());

    arms_close(a);
    if (failures) fprintf(stderr, "%d 项失败\n", failures);
    else printf("OK\n");
    return failures ? 1 : 0;
}
