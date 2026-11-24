/* 单元测试：外部驱动模式（模拟后端） */
#include <stdio.h>
#include <time.h>
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

static uint64_t mono_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

int main(int argc, char **argv)
{
    arms_options opt;
    arms_t *arms = NULL;
    arms_joint_state s;
    arms_bus_stats st;
    uint64_t deadline;
    int i;

    if (argc < 2) {
        fprintf(stderr, "用法: %s <arms.json>\n", argv[0]);
        return 2;
    }
    arms_options_init(&opt);
    opt.config_path = argv[1];
    opt.backend = ARMS_BACKEND_MOCK;
    opt.mode = ARMS_MODE_EXTERNAL;
    opt.cycle_hz = 1000;

    CHECK(arms_open(&opt, &arms) == ARMS_OK, arms_last_error());
    if (!arms) return 1;
    CHECK(arms_mode(arms) == ARMS_MODE_EXTERNAL, "mode");
    CHECK(arms_period_ns(arms) == 1000000ull, "period");
    CHECK(!arms_ready(arms), "ready before cycling");

    deadline = mono_ns() + 3000000000ull;
    while (!arms_ready(arms) && mono_ns() < deadline) {
        CHECK(arms_rx(arms, mono_ns()) == ARMS_OK, "rx");
        CHECK(arms_tx(arms, mono_ns()) == ARMS_OK, "tx");
        usleep(1000);
    }
    CHECK(arms_ready(arms), "not ready within 3 s");
    for (i = 0; i < 100; ++i) {
        arms_rx(arms, mono_ns());
        arms_tx(arms, mono_ns());
        usleep(1000);
    }
    CHECK(arms_read_joint(arms, 21, &s) == ARMS_OK && s.mode == ARMS_JS_LOCKED, "joint 21 locked");
    CHECK(arms_read_joint(arms, 28, &s) == ARMS_OK && s.mode == ARMS_JS_LOCKED, "joint 28 locked");
    CHECK(arms_get_stats(arms, &st) == ARMS_OK, "stats");
    CHECK(st.arm[0].present && st.arm[1].present, "both arms present");
    CHECK(st.arm[0].wkc_bad == 0 && st.arm[1].wkc_bad == 0, "no WKC errors");
    CHECK(st.cycles >= 100, "cycle count");
    arms_close(arms);

    arms_options_init(&opt);
    opt.config_path = argv[1];
    opt.backend = ARMS_BACKEND_MOCK;
    opt.mode = ARMS_MODE_STANDALONE;
    CHECK(arms_open(&opt, &arms) == ARMS_OK, arms_last_error());
    if (arms) {
        CHECK(arms_ready(arms), "standalone ready after open");
        CHECK(arms_rx(arms, mono_ns()) == ARMS_E_STATE, "rx rejected in standalone mode");
        arms_close(arms);
    }

    if (failures) {
        fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
