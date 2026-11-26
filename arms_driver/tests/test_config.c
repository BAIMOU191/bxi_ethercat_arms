/* 单元测试：配置校验、关节选择、KGU 编解码 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "config.h"
#include "joint_select.h"
#include "kgu.h"

static int failures;
#define CHECK(cond, msg)                                                   \
    do {                                                                   \
        if (!(cond)) {                                                     \
            fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, msg);   \
            failures++;                                                    \
        }                                                                  \
    } while (0)

static bool load_text(const char *text, arms_config *c, char *err, size_t n)
{
    char path[] = "/tmp/arms_cfg_XXXXXX";
    int fd = mkstemp(path);
    bool ok;
    if (fd < 0) return false;
    if (write(fd, text, strlen(text)) < 0) { close(fd); return false; }
    close(fd);
    ok = arms_config_load(path, c, err, n);
    unlink(path);
    return ok;
}

int main(int argc, char **argv)
{
    static arms_config c;
    char err[512];
    arms_mask m;

    /* 随附的配置文件 */
    CHECK(argc > 1 && arms_config_load(argv[1], &c, err, sizeof(err)), err);
    CHECK(c.cycle_hz == 1000, "cycle_hz");
    CHECK(c.mode == ARMS_MODE_STANDALONE && c.rt_cpu == 3 && c.irq_cpu == 3 && c.softirq_priority == 70, "realtime");
    CHECK(c.joint[21].present && c.joint[21].slave == 6 && c.joint[21].arm == ARMS_ARM_LEFT, "joint 21");

    /* wrong name for index */
    CHECK(!load_text("{\"arms\":{\"left\":{\"interface\":\"eth0\"}},\"joints\":[{\"joint\":15,\"name\":\"l_wrist_z_joint\","
                     "\"arm\":\"left\",\"slave\":0}]}", &c, err, sizeof(err)), "name mismatch accepted");
    CHECK(strstr(err, "l_shoulder_y_joint") != NULL, "name mismatch message");
    /* duplicate slave */
    CHECK(!load_text("{\"arms\":{\"left\":{\"interface\":\"eth0\"}},\"joints\":["
                     "{\"joint\":15,\"name\":\"l_shoulder_y_joint\",\"arm\":\"left\",\"slave\":0},"
                     "{\"joint\":16,\"name\":\"l_shoulder_x_joint\",\"arm\":\"left\",\"slave\":0}]}", &c, err, sizeof(err)),
          "duplicate slave accepted");
    /* right joint on left arm */
    CHECK(!load_text("{\"arms\":{\"left\":{\"interface\":\"eth0\"}},\"joints\":[{\"joint\":22,\"name\":\"r_shoulder_y_joint\","
                     "\"arm\":\"left\",\"slave\":0}]}", &c, err, sizeof(err)), "arm mismatch accepted");
    /* unsupported cycle */
    CHECK(!load_text("{\"cycle_hz\":3000,\"arms\":{\"left\":{\"interface\":\"eth0\"}},\"joints\":[{\"joint\":15,"
                     "\"name\":\"l_shoulder_y_joint\",\"arm\":\"left\",\"slave\":0}]}", &c, err, sizeof(err)),
          "3000 Hz accepted");
    /* realtime 段 */
    CHECK(load_text("{\"realtime\":{\"mode\":\"external\",\"rt_cpu\":0},\"arms\":{\"left\":{\"interface\":\"eth0\"}},"
                    "\"joints\":[{\"joint\":15,\"name\":\"l_shoulder_y_joint\",\"arm\":\"left\",\"slave\":0}]}",
                    &c, err, sizeof(err)) && c.mode == ARMS_MODE_EXTERNAL && c.rt_cpu == 0, err);
    CHECK(!load_text("{\"realtime\":{\"mode\":\"fast\"},\"arms\":{\"left\":{\"interface\":\"eth0\"}},\"joints\":[{\"joint\":15,"
                     "\"name\":\"l_shoulder_y_joint\",\"arm\":\"left\",\"slave\":0}]}", &c, err, sizeof(err)),
          "invalid mode accepted");
    CHECK(!load_text("{\"realtime\":{\"priority\":70,\"softirq_priority\":80},\"arms\":{\"left\":{\"interface\":\"eth0\"}},"
                     "\"joints\":[{\"joint\":15,\"name\":\"l_shoulder_y_joint\",\"arm\":\"left\",\"slave\":0}]}",
                     &c, err, sizeof(err)), "softirq priority above control thread accepted");
    /* 最小合法配置 */
    CHECK(load_text("{\"cycle_hz\":2000,\"arms\":{\"left\":{\"interface\":\"eth0\"}},\"joints\":[{\"joint\":15,"
                    "\"name\":\"l_shoulder_y_joint\",\"arm\":\"left\",\"slave\":0}]}", &c, err, sizeof(err)), err);

    /* 关节选择 */
    CHECK(arms_parse_joints("left", 0, &m, err, sizeof(err)) == 0 && m == ARMS_MASK_LEFT, "left");
    CHECK(arms_parse_joints("15-18,28", 0, &m, err, sizeof(err)) == 0 &&
          m == (ARMS_MASK_JOINT(15) | ARMS_MASK_JOINT(16) | ARMS_MASK_JOINT(17) | ARMS_MASK_JOINT(18) | ARMS_MASK_JOINT(28)),
          "range list");
    CHECK(arms_parse_joints("l_wrist_z", 0, &m, err, sizeof(err)) == 0 && m == ARMS_MASK_JOINT(21), "short name");
    CHECK(arms_parse_joints("r_wrist_z_joint", 0, &m, err, sizeof(err)) == 0 && m == ARMS_MASK_JOINT(28), "full name");
    CHECK(arms_parse_joints("14", 0, &m, err, sizeof(err)) != 0, "14 accepted");
    CHECK(arms_parse_joints("foo", 0, &m, err, sizeof(err)) != 0, "foo accepted");
    CHECK(arms_parse_joints("22", ARMS_MASK_LEFT, &m, err, sizeof(err)) != 0, "unopened joint accepted");

    CHECK(arms_parse_joints("4294967311", 0, &m, err, sizeof(err)) != 0, "wrapped joint accepted");
    CHECK(arms_parse_joints("15,,28", 0, &m, err, sizeof(err)) != 0, "empty joint accepted");

    CHECK(arms_parse_joints("15,22", ARMS_MASK_LEFT, &m, err, sizeof(err)) != 0, "partial list accepted");
    CHECK(arms_parse_joints("20-22", ARMS_MASK_LEFT, &m, err, sizeof(err)) != 0, "partial range accepted");
    CHECK(arms_parse_joints("all", ARMS_MASK_LEFT, &m, err, sizeof(err)) == 0 && m == ARMS_MASK_LEFT, "all intersects session");

    /* KGU 编解码 */
    {
        uint8_t out[KGU_OUT_BYTES], in[KGU_IN_BYTES];
        kgu_command cmd = {KGU_CW_ENABLE, 0.2f, -1.5f, 0.25f, 5.0f, 0.5f, KGU_MODE_MIT};
        kgu_feedback fb;
        kgu_encode(out, &cmd);
        CHECK(out[0] == 0x0F && out[1] == 0x00 && out[22] == 10, "controlword/mode bytes");
        CHECK(fabsf(kgu_get_f32(out, KGU_OUT_TARGET_POS) - 0.2f) < 1e-7f, "pos");
        CHECK(fabsf(kgu_get_f32(out, KGU_OUT_KP) - 5.0f) < 1e-7f, "kp");
        memset(in, 0, sizeof(in));
        kgu_put_u16(in, KGU_IN_STATUSWORD, 0x0237);
        kgu_put_f32(in, KGU_IN_POS, 0.115f);
        kgu_put_u16(in, KGU_IN_ERROR, 0x1234);
        kgu_decode(in, &fb);
        CHECK(fb.statusword == 0x0237 && fabsf(fb.pos - 0.115f) < 1e-7f && fb.error == 0x1234, "decode");
    }

    if (failures) {
        fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
