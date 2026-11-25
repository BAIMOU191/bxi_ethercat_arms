#include "config.h"

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "cJSON.h"
#include "kgu.h"

const char *const arms_standard_joint_names[31] = {
    "waist_y_joint", "waist_x_joint", "waist_z_joint",
    "l_hip_y_joint", "l_hip_x_joint", "l_hip_z_joint", "l_knee_y_joint", "l_ankle_y_joint", "l_ankle_x_joint",
    "r_hip_y_joint", "r_hip_x_joint", "r_hip_z_joint", "r_knee_y_joint", "r_ankle_y_joint", "r_ankle_x_joint",
    "l_shoulder_y_joint", "l_shoulder_x_joint", "l_shoulder_z_joint", "l_elbow_y_joint",
    "l_wrist_x_joint", "l_wrist_y_joint", "l_wrist_z_joint",
    "r_shoulder_y_joint", "r_shoulder_x_joint", "r_shoulder_z_joint", "r_elbow_y_joint",
    "r_wrist_x_joint", "r_wrist_y_joint", "r_wrist_z_joint",
    "head_z_joint", "head_y_joint",
};

bool arms_cycle_hz_supported(unsigned int hz)
{
    /* KGU manual (Er301): 0.5, 1, 2, 4, 5, 8 ms */
    return hz == 2000u || hz == 1000u || hz == 500u || hz == 250u || hz == 200u || hz == 125u;
}

static void fail(char *error, size_t size, const char *fmt, ...)
{
    va_list ap;
    if (!error || size == 0) return;
    va_start(ap, fmt);
    vsnprintf(error, size, fmt, ap);
    va_end(ap);
}

static char *read_file(const char *path, char *error, size_t error_size)
{
    FILE *f = fopen(path, "rb");
    long len;
    char *buf;
    if (!f) {
        fail(error, error_size, "无法打开配置文件 %s: %s", path, strerror(errno));
        return NULL;
    }
    if (fseek(f, 0, SEEK_END) != 0 || (len = ftell(f)) < 0 || len > 1024 * 1024 ||
        fseek(f, 0, SEEK_SET) != 0) {
        fclose(f);
        fail(error, error_size, "配置文件 %s 读取失败或过大", path);
        return NULL;
    }
    buf = malloc((size_t)len + 1);
    if (!buf) {
        fclose(f);
        fail(error, error_size, "内存不足");
        return NULL;
    }
    if (fread(buf, 1, (size_t)len, f) != (size_t)len) {
        fclose(f);
        free(buf);
        fail(error, error_size, "配置文件 %s 读取不完整", path);
        return NULL;
    }
    buf[len] = '\0';
    fclose(f);
    return buf;
}

static const cJSON *obj(const cJSON *parent, const char *key)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(parent, key);
    return cJSON_IsObject(item) ? item : NULL;
}

static bool get_num(const cJSON *parent, const char *key, double *out)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(parent, key);
    if (!cJSON_IsNumber(item) || !isfinite(item->valuedouble)) return false;
    *out = item->valuedouble;
    return true;
}

static bool get_int(const cJSON *parent, const char *key, long min, long max, long *out)
{
    double v;
    if (!get_num(parent, key, &v) || v != floor(v) || v < (double)min || v > (double)max) return false;
    *out = (long)v;
    return true;
}

static bool get_str(const cJSON *parent, const char *key, char *out, size_t size)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(parent, key);
    if (!cJSON_IsString(item) || !item->valuestring || strlen(item->valuestring) >= size) return false;
    snprintf(out, size, "%s", item->valuestring);
    return true;
}

static bool get_u32_hex(const cJSON *parent, const char *key, uint32_t *out)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(parent, key);
    if (cJSON_IsNumber(item) && item->valuedouble >= 0 && item->valuedouble <= 4294967295.0 &&
        floor(item->valuedouble) == item->valuedouble) {
        *out = (uint32_t)item->valuedouble;
        return true;
    }
    if (cJSON_IsString(item) && item->valuestring) {
        char *end = NULL;
        unsigned long v;
        errno = 0;
        v = strtoul(item->valuestring, &end, 0);
        if (errno == 0 && end && end != item->valuestring && item->valuestring[0] != '-' && *end == '\0' &&
            v <= 0xFFFFFFFFul) {
            *out = (uint32_t)v;
            return true;
        }
    }
    return false;
}

static void set_defaults(arms_config *c)
{
    memset(c, 0, sizeof(*c));
    c->version = 1;
    c->cycle_hz = 1000;
    c->mode = ARMS_MODE_STANDALONE;
    c->rt_cpu = -1;
    c->irq_cpu = -1;
    c->rt_priority = 80;
    c->softirq_priority = 70;
    c->wkc_tolerance = 3;
    c->command_timeout_ms = 50;
    snprintf(c->timeout_action, sizeof(c->timeout_action), "hold");
    c->check_motors = true;
    c->ignore_mask = 0;
    snprintf(c->offline_action, sizeof(c->offline_action), "power_off");
    c->overspeed_ms = 500;
    c->overtorque_ms = 100;
    c->op_timeout_ms = 30000;
    c->drive_monitor = true;
    c->temp_warn_c = 75;
    c->temp_max_c = 90;
    c->vendor_id = KGU_VENDOR_ID;
    c->product_code = KGU_PRODUCT_CODE;
}

static bool parse_arm(const cJSON *node, arms_arm_config *arm, const char *label, char *error, size_t size)
{
    long master = -1;
    const cJSON *m;
    if (!node) return true; /* arm absent */
    arm->present = true;
    arm->master = -1;
    if (!get_str(node, "interface", arm->interface, sizeof(arm->interface)) || arm->interface[0] == '\0') {
        fail(error, size, "arms.%s.interface 缺失或无效", label);
        return false;
    }
    if (cJSON_GetObjectItemCaseSensitive(node, "mac") && !get_str(node, "mac", arm->mac, sizeof(arm->mac))) {
        fail(error, size, "arms.%s.mac 必须为字符串", label);
        return false;
    }
    m = cJSON_GetObjectItemCaseSensitive(node, "master");
    if (m && !cJSON_IsNull(m)) {
        if (!get_int(node, "master", 0, 7, &master)) {
            fail(error, size, "arms.%s.master 必须是 0..7 的整数", label);
            return false;
        }
        arm->master = (int)master;
    }
    return true;
}

/* 允许省略使用默认值，但不把已填写的错误类型静默当作未配置。 */
static bool optional_number(const cJSON *object, const char *key, double *out)
{
    return !cJSON_GetObjectItemCaseSensitive(object, key) || get_num(object, key, out);
}

static bool parse_pair(const cJSON *arr, double *a, double *b)
{
    const cJSON *x, *y;
    if (!cJSON_IsArray(arr) || cJSON_GetArraySize(arr) != 2) return false;
    x = cJSON_GetArrayItem(arr, 0);
    y = cJSON_GetArrayItem(arr, 1);
    if (!cJSON_IsNumber(x) || !cJSON_IsNumber(y)) return false;
    *a = x->valuedouble;
    *b = y->valuedouble;
    return isfinite(*a) && isfinite(*b) && *a < *b;
}

static bool parse_joint(const cJSON *node, arms_config *c, int idx, char *error, size_t size)
{
    long joint, slave, direction;
    char arm_name[16], name[ARMS_NAME_LEN];
    arms_joint_config *j;
    const cJSON *limit, *gmax, *gdef;

    if (!cJSON_IsObject(node)) {
        fail(error, size, "joints[%d] 不是对象", idx);
        return false;
    }
    if (!get_int(node, "joint", ARMS_JOINT_FIRST, ARMS_JOINT_LAST, &joint)) {
        fail(error, size, "joints[%d].joint 必须是 %d..%d", idx, ARMS_JOINT_FIRST, ARMS_JOINT_LAST);
        return false;
    }
    j = &c->joint[joint];
    if (j->present) {
        fail(error, size, "关节 %ld 重复出现", joint);
        return false;
    }
    if (!get_str(node, "name", name, sizeof(name))) {
        fail(error, size, "关节 %ld 缺少 name", joint);
        return false;
    }
    if (strcmp(name, arms_standard_joint_names[joint]) != 0) {
        fail(error, size, "关节 %ld 的名称应为 %s（BXI 标准顺序），配置写的是 %s", joint,
             arms_standard_joint_names[joint], name);
        return false;
    }
    if (!get_str(node, "arm", arm_name, sizeof(arm_name))) {
        fail(error, size, "关节 %ld 缺少 arm", joint);
        return false;
    }
    j->arm = strcmp(arm_name, "left") == 0 ? ARMS_ARM_LEFT : strcmp(arm_name, "right") == 0 ? ARMS_ARM_RIGHT : -1;
    if (j->arm < 0) {
        fail(error, size, "关节 %ld 的 arm 必须是 left 或 right", joint);
        return false;
    }
    if ((j->arm == ARMS_ARM_LEFT) != (joint <= 21)) {
        fail(error, size, "关节 %ld (%s) 不属于 %s 臂", joint, name, arm_name);
        return false;
    }
    if (!get_int(node, "slave", 0, ARMS_MAX_SLAVES_PER_ARM - 1, &slave)) {
        fail(error, size, "关节 %ld 的 slave 必须是 0..%d（EtherLab 链上位置，从 0 开始）", joint,
             ARMS_MAX_SLAVES_PER_ARM - 1);
        return false;
    }
    direction = 1;
    if (cJSON_GetObjectItemCaseSensitive(node, "direction") &&
        (!get_int(node, "direction", -1, 1, &direction) || direction == 0)) {
        fail(error, size, "关节 %ld 的 direction 只能是 1 或 -1", joint);
        return false;
    }
    j->present = true;
    j->joint = (int)joint;
    snprintf(j->name, sizeof(j->name), "%s", name);
    j->slave = (int)slave;
    j->direction = (int)direction;
    if (!optional_number(node, "offset_rad", &j->offset_rad) || !optional_number(node, "home", &j->home)) {
        fail(error, size, "关节 %ld 的 offset_rad 和 home 必须是有限数值", joint);
        return false;
    }

    j->pos_min = -0.5; j->pos_max = 0.5; j->vel_max = 0.5; j->tff_max = 1.0;
    j->vel_fault = 3.0; j->tor_fault = 30.0;
    j->kp_max = 5; j->kd_max = 1; j->kp_default = 5; j->kd_default = 0;
    limit = cJSON_GetObjectItemCaseSensitive(node, "limit");
    if (limit) {
        const cJSON *pos = cJSON_GetObjectItemCaseSensitive(limit, "pos");
        if (pos && !parse_pair(pos, &j->pos_min, &j->pos_max)) {
            fail(error, size, "关节 %ld 的 limit.pos 必须是 [min, max] 且 min<max", joint);
            return false;
        }
        if (!cJSON_IsObject(limit) ||
            !optional_number(limit, "vel", &j->vel_max) ||
            !optional_number(limit, "tff", &j->tff_max) ||
            !optional_number(limit, "vel_fault", &j->vel_fault) ||
            !optional_number(limit, "torque_fault", &j->tor_fault)) {
            fail(error, size, "关节 %ld 的 limit 必须是对象，数值必须有限", joint);
            return false;
        }
    }
    if (j->vel_fault <= 0 || j->tor_fault <= 0) {
        fail(error, size, "关节 %ld 的 limit.vel_fault / limit.torque_fault 必须大于 0", joint);
        return false;
    }
    gmax = cJSON_GetObjectItemCaseSensitive(node, "gain_max");
    if (gmax) {
        if (!cJSON_IsObject(gmax) ||
            !optional_number(gmax, "kp", &j->kp_max) ||
            !optional_number(gmax, "kd", &j->kd_max)) {
            fail(error, size, "关节 %ld 的 gain_max 必须是对象，数值必须有限", joint);
            return false;
        }
    }
    gdef = cJSON_GetObjectItemCaseSensitive(node, "gain_default");
    if (gdef) {
        if (!cJSON_IsObject(gdef) ||
            !optional_number(gdef, "kp", &j->kp_default) ||
            !optional_number(gdef, "kd", &j->kd_default)) {
            fail(error, size, "关节 %ld 的 gain_default 必须是对象，数值必须有限", joint);
            return false;
        }
    }
    if (j->vel_max < 0 || j->tff_max < 0 || j->kp_max < 0 || j->kd_max < 0 ||
        j->kp_default < 0 || j->kd_default < 0 || j->kp_default > j->kp_max || j->kd_default > j->kd_max) {
        fail(error, size, "关节 %ld 的限位/增益无效（不能为负，默认值不能超过上限）", joint);
        return false;
    }
    if (j->home < j->pos_min || j->home > j->pos_max) {
        fail(error, size, "关节 %ld 的 home %.3f 超出 limit.pos [%.3f, %.3f]", joint, j->home, j->pos_min, j->pos_max);
        return false;
    }
    return true;
}

bool arms_config_load(const char *path, arms_config *c, char *error, size_t size)
{
    char *text;
    cJSON *root;
    const cJSON *rt, *safety, *arms, *joints, *drive;
    long v;
    int i, a, n;
    bool ok = false;

    set_defaults(c);
    snprintf(c->path, sizeof(c->path), "%s", path);
    text = read_file(path, error, size);
    if (!text) return false;
    root = cJSON_ParseWithOpts(text, NULL, 1);
    if (!root) {
        const char *where = cJSON_GetErrorPtr();
        int line = 1;
        const char *p;
        for (p = text; where && p < where && *p; ++p) if (*p == '\n') line++;
        fail(error, size, "%s 不是合法 JSON（约第 %d 行附近）", path, line);
        free(text);
        return false;
    }
    if (!cJSON_IsObject(root)) {
        fail(error, size, "%s 顶层必须是对象", path);
        goto out;
    }
    const char *sections[] = {"arms", "safety", "realtime", "drive"};
    for (size_t k = 0; k < sizeof(sections) / sizeof(sections[0]); ++k) {
        const cJSON *section = cJSON_GetObjectItemCaseSensitive(root, sections[k]);
        if (section && !cJSON_IsObject(section)) {
            fail(error, size, "%s 必须是对象", sections[k]);
            goto out;
        }
    }
    if (cJSON_GetObjectItemCaseSensitive(root, "version") && !get_int(root, "version", 1, 1, &v)) {
        fail(error, size, "version 只支持 1");
        goto out;
    }
    if (cJSON_GetObjectItemCaseSensitive(root, "cycle_hz")) {
        if (!get_int(root, "cycle_hz", 1, 100000, &v) || !arms_cycle_hz_supported((unsigned int)v)) {
            fail(error, size, "cycle_hz 只能是 2000/1000/500/250/200/125（驱动器支持 0.5/1/2/4/5/8 ms）");
            goto out;
        }
        c->cycle_hz = (unsigned int)v;
    }
    rt = obj(root, "realtime");
    if (rt) {
        long ncpu = sysconf(_SC_NPROCESSORS_CONF);
        char mode[16];
        if (ncpu < 1) ncpu = 1;
        if (cJSON_GetObjectItemCaseSensitive(rt, "mode")) {
            if (!get_str(rt, "mode", mode, sizeof(mode)) ||
                (strcmp(mode, "standalone") != 0 && strcmp(mode, "external") != 0)) {
                fail(error, size, "realtime.mode 只能是 standalone 或 external");
                goto out;
            }
            c->mode = strcmp(mode, "external") == 0 ? ARMS_MODE_EXTERNAL : ARMS_MODE_STANDALONE;
        }
        if (cJSON_GetObjectItemCaseSensitive(rt, "softirq_priority")) {
            if (!get_int(rt, "softirq_priority", 1, 98, &v)) {
                fail(error, size, "realtime.softirq_priority 必须是 1..98");
                goto out;
            }
            c->softirq_priority = (int)v;
        }
        if (cJSON_GetObjectItemCaseSensitive(rt, "rt_cpu") && !get_int(rt, "rt_cpu", -1, ncpu - 1, &v)) {
            fail(error, size, "realtime.rt_cpu 必须是 -1..%ld", ncpu - 1);
            goto out;
        }
        if (cJSON_GetObjectItemCaseSensitive(rt, "rt_cpu")) c->rt_cpu = (int)v;
        if (cJSON_GetObjectItemCaseSensitive(rt, "irq_cpu") && !get_int(rt, "irq_cpu", -1, ncpu - 1, &v)) {
            fail(error, size, "realtime.irq_cpu 必须是 -1..%ld", ncpu - 1);
            goto out;
        }
        if (cJSON_GetObjectItemCaseSensitive(rt, "irq_cpu")) c->irq_cpu = (int)v;
        if (cJSON_GetObjectItemCaseSensitive(rt, "priority") && !get_int(rt, "priority", 1, 98, &v)) {
            fail(error, size, "realtime.priority 必须是 1..98");
            goto out;
        }
        if (cJSON_GetObjectItemCaseSensitive(rt, "priority")) c->rt_priority = (int)v;
    }
    safety = obj(root, "safety");
    if (safety) {
        if (cJSON_GetObjectItemCaseSensitive(safety, "wkc_tolerance")) {
            if (!get_int(safety, "wkc_tolerance", 1, 1000, &v)) {
                fail(error, size, "safety.wkc_tolerance 必须是 1..1000");
                goto out;
            }
            c->wkc_tolerance = (unsigned int)v;
        }
        if (cJSON_GetObjectItemCaseSensitive(safety, "command_timeout_ms")) {
            if (!get_int(safety, "command_timeout_ms", 1, 60000, &v)) {
                fail(error, size, "safety.command_timeout_ms 必须是 1..60000");
                goto out;
            }
            c->command_timeout_ms = (unsigned int)v;
        }
        if (cJSON_GetObjectItemCaseSensitive(safety, "op_timeout_ms")) {
            if (!get_int(safety, "op_timeout_ms", 1000, 300000, &v)) {
                fail(error, size, "safety.op_timeout_ms 必须是 1000..300000");
                goto out;
            }
            c->op_timeout_ms = (unsigned int)v;
        }
        if (cJSON_GetObjectItemCaseSensitive(safety, "overspeed_ms")) {
            if (!get_int(safety, "overspeed_ms", 1, 10000, &v)) {
                fail(error, size, "safety.overspeed_ms 必须是 1..10000");
                goto out;
            }
            c->overspeed_ms = (unsigned int)v;
        }
        if (cJSON_GetObjectItemCaseSensitive(safety, "overtorque_ms")) {
            if (!get_int(safety, "overtorque_ms", 1, 10000, &v)) {
                fail(error, size, "safety.overtorque_ms 必须是 1..10000");
                goto out;
            }
            c->overtorque_ms = (unsigned int)v;
        }
        {
            const cJSON *dm = cJSON_GetObjectItemCaseSensitive(safety, "drive_monitor");
            if (dm && !cJSON_IsBool(dm)) {
                fail(error, size, "safety.drive_monitor 必须是 true/false");
                goto out;
            }
            if (dm) c->drive_monitor = cJSON_IsTrue(dm);
        }
        if (cJSON_GetObjectItemCaseSensitive(safety, "temp_warn_c")) {
            if (!get_int(safety, "temp_warn_c", 20, 150, &v)) {
                fail(error, size, "safety.temp_warn_c 必须是 20..150");
                goto out;
            }
            c->temp_warn_c = v;
        }
        if (cJSON_GetObjectItemCaseSensitive(safety, "temp_max_c")) {
            if (!get_int(safety, "temp_max_c", 20, 150, &v)) {
                fail(error, size, "safety.temp_max_c 必须是 20..150");
                goto out;
            }
            c->temp_max_c = v;
        }
        if (c->temp_warn_c > c->temp_max_c) {
            fail(error, size, "safety.temp_warn_c 不能大于 safety.temp_max_c");
            goto out;
        }
        if (cJSON_GetObjectItemCaseSensitive(safety, "timeout_action") &&
            (!get_str(safety, "timeout_action", c->timeout_action, sizeof(c->timeout_action)) ||
             (strcmp(c->timeout_action, "hold") != 0 && strcmp(c->timeout_action, "disable") != 0))) {
            fail(error, size, "safety.timeout_action 只能是 hold 或 disable");
            goto out;
        }
        {
            const cJSON *cm = cJSON_GetObjectItemCaseSensitive(safety, "check_motors");
            if (cm && !cJSON_IsBool(cm)) {
                fail(error, size, "safety.check_motors 必须是 true/false");
                goto out;
            }
            if (cm) c->check_motors = cJSON_IsTrue(cm);
        }
        if (cJSON_GetObjectItemCaseSensitive(safety, "ignore_mask") &&
            (!get_u32_hex(safety, "ignore_mask", &c->ignore_mask) || c->ignore_mask > 0x7FFFFFFFu)) {
            fail(error, size, "safety.ignore_mask 必须是 0..0x7fffffff（第 n 位对应关节 n）");
            goto out;
        }
        if (cJSON_GetObjectItemCaseSensitive(safety, "offline_action") &&
            (!get_str(safety, "offline_action", c->offline_action, sizeof(c->offline_action)) ||
             (strcmp(c->offline_action, "power_off") != 0 && strcmp(c->offline_action, "disable") != 0))) {
            fail(error, size, "safety.offline_action 只能是 power_off 或 disable");
            goto out;
        }
    }
    drive = obj(root, "drive");
    if (drive) {
        if (cJSON_GetObjectItemCaseSensitive(drive, "vendor_id") && !get_u32_hex(drive, "vendor_id", &c->vendor_id)) {
            fail(error, size, "drive.vendor_id 无效");
            goto out;
        }
        if (cJSON_GetObjectItemCaseSensitive(drive, "product_code") &&
            !get_u32_hex(drive, "product_code", &c->product_code)) {
            fail(error, size, "drive.product_code 无效");
            goto out;
        }
    }
    arms = obj(root, "arms");
    if (!arms) {
        fail(error, size, "缺少 arms 段（left/right 的网口配置）");
        goto out;
    }
    if (!parse_arm(obj(arms, "left"), &c->arm[ARMS_ARM_LEFT], "left", error, size) ||
        !parse_arm(obj(arms, "right"), &c->arm[ARMS_ARM_RIGHT], "right", error, size))
        goto out;
    if (!c->arm[0].present && !c->arm[1].present) {
        fail(error, size, "arms 段至少要有 left 或 right");
        goto out;
    }
    if (c->arm[0].present && c->arm[1].present && strcmp(c->arm[0].interface, c->arm[1].interface) == 0) {
        fail(error, size, "左右臂不能使用同一个网口 %s", c->arm[0].interface);
        goto out;
    }
    joints = cJSON_GetObjectItemCaseSensitive(root, "joints");
    if (!cJSON_IsArray(joints) || cJSON_GetArraySize(joints) == 0) {
        fail(error, size, "缺少 joints 数组");
        goto out;
    }
    n = cJSON_GetArraySize(joints);
    for (i = 0; i < n; ++i)
        if (!parse_joint(cJSON_GetArrayItem(joints, i), c, i, error, size)) goto out;
    for (a = 0; a < ARMS_ARM_COUNT; ++a) {
        bool used[ARMS_MAX_SLAVES_PER_ARM] = {false};
        for (i = ARMS_JOINT_FIRST; i <= ARMS_JOINT_LAST; ++i) {
            const arms_joint_config *j = &c->joint[i];
            if (!j->present || j->arm != a) continue;
            if (!c->arm[a].present) {
                fail(error, size, "关节 %d 属于 %s 臂，但 arms 段没有配置该臂", i, a == 0 ? "left" : "right");
                goto out;
            }
            if (used[j->slave]) {
                fail(error, size, "%s 臂的 slave %d 被多个关节使用", a == 0 ? "left" : "right", j->slave);
                goto out;
            }
            used[j->slave] = true;
        }
    }
    if (c->softirq_priority >= c->rt_priority) {
        fail(error, size, "realtime.softirq_priority (%d) 必须低于 realtime.priority (%d)", c->softirq_priority,
             c->rt_priority);
        goto out;
    }
    ok = true;
out:
    cJSON_Delete(root);
    free(text);
    return ok;
}

bool arms_config_default_path(char *out, size_t size)
{
    const char *env = getenv("ARMS_CONFIG");
    char exe[PATH_MAX];
    ssize_t len;
    if (env && env[0] && access(env, R_OK) == 0) {
        snprintf(out, size, "%s", env);
        return true;
    }
    if (access("/etc/bxi/arms.json", R_OK) == 0) {
        snprintf(out, size, "/etc/bxi/arms.json");
        return true;
    }
    if (access("config/arms.json", R_OK) == 0) {
        snprintf(out, size, "config/arms.json");
        return true;
    }
    len = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
    if (len > 0) {
        char *slash;
        exe[len] = '\0';
        slash = strrchr(exe, '/');
        if (slash) {
            *slash = '\0';
            snprintf(out, size, "%s/../config/arms.json", exe);
            if (access(out, R_OK) == 0) return true;
            snprintf(out, size, "%s/config/arms.json", exe);
            if (access(out, R_OK) == 0) return true;
        }
    }
    return false;
}
