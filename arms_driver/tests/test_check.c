/* 单元测试：启动在线检查与屏蔽掩码（模拟后端，每臂只有 5 个从站，关节 20/21/27/28 不在线） */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "arms/arms.h"

static int failures;
#define CHECK(cond, msg)                                                   \
    do {                                                                   \
        if (!(cond)) {                                                     \
            fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, msg);   \
            failures++;                                                    \
        }                                                                  \
    } while (0)

#define MISSING (ARMS_MASK_JOINT(20) | ARMS_MASK_JOINT(21) | ARMS_MASK_JOINT(27) | ARMS_MASK_JOINT(28))

static void init(arms_options *o, const char *path)
{
    arms_options_init(o);
    o->config_path = path;
    o->backend = ARMS_BACKEND_MOCK;
    o->mode = ARMS_MODE_STANDALONE;
    o->cycle_hz = 1000;
    o->rt_cpu = -1;
}

int main(int argc, char **argv)
{
    arms_options opt;
    arms_t *a = NULL;
    arms_mask off = 0;
    arms_joint_state s;

    if (argc < 2) {
        fprintf(stderr, "用法: %s <arms.json>\n", argv[0]);
        return 2;
    }
    setenv("ARMS_MOCK_SLAVES", "5", 1);

    /* 检查开启（配置默认）：打开失败并列出不在线关节 */
    init(&opt, argv[1]);
    CHECK(arms_open(&opt, &a) == ARMS_E_TOPOLOGY, "check must fail");
    CHECK(a == NULL, "no handle on failure");
    CHECK(strstr(arms_last_error(), "20(l_wrist_y_joint") != NULL, arms_last_error());
    CHECK(strstr(arms_last_error(), "21(l_wrist_z_joint") != NULL, arms_last_error());

    /* 屏蔽不在线的关节后检查通过 */
    init(&opt, argv[1]);
    opt.ignore_mask = MISSING;
    CHECK(arms_open(&opt, &a) == ARMS_OK, arms_last_error());
    if (a) {
        CHECK(arms_ignored_joints(a) == MISSING, "ignored mask");
        CHECK(arms_opened_joints(a) == (ARMS_MASK_ALL & ~MISSING), "opened joints");
        CHECK(arms_check_online(a, &off) == ARMS_OK && off == 0, arms_last_error());
        arms_read_joint(a, 21, &s);
        CHECK(s.mode == ARMS_JS_ABSENT, "ignored joint absent");
        CHECK(arms_enable(a, ARMS_MASK_JOINT(19), 0) == ARMS_OK, arms_last_error());
        arms_close(a);
        a = NULL;
    }

    /* 屏蔽位以外的位（非 15..28）被忽略 */
    init(&opt, argv[1]);
    opt.ignore_mask = MISSING | 0x7u;
    CHECK(arms_open(&opt, &a) == ARMS_OK, arms_last_error());
    if (a) {
        CHECK(arms_ignored_joints(a) == MISSING, "non-arm bits dropped");
        arms_close(a);
        a = NULL;
    }

    /* 超出 31 位 */
    init(&opt, argv[1]);
    opt.ignore_mask = 0x80000000ll;
    CHECK(arms_open(&opt, &a) == ARMS_E_ARG, "mask > 31 bits rejected");

    /* 检查关闭：跳过不在线关节 */
    init(&opt, argv[1]);
    opt.check_motors = 0;
    CHECK(arms_open(&opt, &a) == ARMS_OK, arms_last_error());
    if (a) {
        CHECK(arms_missing_joints(a) == MISSING, "missing mask");
        CHECK(arms_opened_joints(a) == (ARMS_MASK_ALL & ~MISSING), "opened without missing");
        arms_close(a);
    }

    if (failures) fprintf(stderr, "%d 项失败\n", failures);
    else printf("OK\n");
    return failures ? 1 : 0;
}
