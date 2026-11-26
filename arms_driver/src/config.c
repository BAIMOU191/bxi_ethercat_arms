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
    /* KGU 手册（Er301）支持的周期: 0.5、1、2、4、5、8 ms */
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
    c->hub_slave = -1;
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
    c->esc_summary_s = 60;
    c->vendor_id = KGU_VENDOR_ID;
    c->product_code = KGU_PRODUCT_CODE;
}

static bool parse_arm(const cJSON *node, arms_arm_config *arm, const char *label, char *error, size_t size)
{
    long master = -1;
    const cJSON *m;
    if (!node) return true; /* 未配置该臂 */
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

/* 字段可省略并使用默认值；已填写但类型错误时报错。 */
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

/* 关节限位：未写的项沿用 base（joint_defaults 或内置默认值） */
typedef struct {
    double pos_min, pos_max, vel, tor;
} joint_limits;

static bool parse_limits(const cJSON *node, joint_limits *l, const char *label, char *error, size_t size)
{
    const cJSON *pos = cJSON_GetObjectItemCaseSensitive(node, "pos_limit");
    if (pos && !parse_pair(pos, &l->pos_min, &l->pos_max)) {
        fail(error, size, "%s 的 pos_limit 必须是 [min, max] 且 min<max", label);
        return false;
    }
    if (!optional_number(node, "vel_limit", &l->vel) || !optional_number(node, "tor_limit", &l->tor) ||
        l->vel <= 0 || l->tor <= 0) {
        fail(error, size, "%s 的 vel_limit / tor_limit 必须是大于 0 的数值", label);
        return false;
    }
    return true;
}

/* 已删除或改名的关节字段，给出明确的迁移提示 */
static bool reject_old_fields(const cJSON *node, long joint, char *error, size_t size)
{
    static const char *const removed[] = {"limit", "gain_max", "gain_default", "home"};
    for (size_t k = 0; k < sizeof(removed) / sizeof(removed[0]); ++k) {
        if (!cJSON_GetObjectItemCaseSensitive(node, removed[k])) continue;
        fail(error, size,
             "关节 %ld 使用了已删除的字段 %s：limit.pos 改为 pos_limit，limit.vel_fault 改为 vel_limit，"
             "limit.torque_fault 改为 tor_limit；limit.vel、limit.tff、gain_max、gain_default、home 已删除。"
             "可用 arms_driver/scripts/migrate_config.py 转换",
             joint, removed[k]);
        return false;
    }
    return true;
}

static bool parse_joint(const cJSON *node, arms_config *c, int idx, const joint_limits *base, char *error,
                        size_t size)
{
    long joint, slave, direction;
    char arm_name[16], name[ARMS_NAME_LEN], label[32];
    arms_joint_config *j;
    joint_limits lim = *base;

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
    if (!optional_number(node, "offset_rad", &j->offset_rad)) {
        fail(error, size, "关节 %ld 的 offset_rad 必须是有限数值", joint);
        return false;
    }
    if (!reject_old_fields(node, joint, error, size)) return false;
    snprintf(label, sizeof(label), "关节 %ld", joint);
    if (!parse_limits(node, &lim, label, error, size)) return false;
    j->pos_limit_min = lim.pos_min;
    j->pos_limit_max = lim.pos_max;
    j->vel_limit = lim.vel;
    j->tor_limit = lim.tor;
    return true;
}

bool arms_config_load(const char *path, arms_config *c, char *error, size_t size)
{
    char *text;
    cJSON *root;
    const cJSON *rt, *safety, *arms, *joints, *drive;
    long v = 0;
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
        if (cJSON_GetObjectItemCaseSensitive(safety, "esc_summary_s")) {
            if (!get_int(safety, "esc_summary_s", 1, 3600, &v)) {
                fail(error, size, "safety.esc_summary_s 必须是 1..3600");
                goto out;
            }
            c->esc_summary_s = (unsigned int)v;
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
        /* 同一网口：两臂经分支器接在同一条链上，链路参数以左臂为准 */
        if (strcmp(c->arm[0].mac, c->arm[1].mac) != 0 || c->arm[0].master != c->arm[1].master) {
            fail(error, size, "左右臂共用网口 %s 时 mac、master 必须一致", c->arm[0].interface);
            goto out;
        }
        c->shared_link = true;
    }
    joints = cJSON_GetObjectItemCaseSensitive(root, "joints");
    if (!cJSON_IsArray(joints) || cJSON_GetArraySize(joints) == 0) {
        fail(error, size, "缺少 joints 数组");
        goto out;
    }
    {
        /* 内置默认值: pos_limit ±3 rad，vel_limit 3 rad/s，tor_limit 30 N·m */
        joint_limits base = {-3.0, 3.0, 3.0, 30.0};
        const cJSON *defaults = cJSON_GetObjectItemCaseSensitive(root, "joint_defaults");
        if (defaults && (!cJSON_IsObject(defaults) || !parse_limits(defaults, &base, "joint_defaults", error, size)))
            goto out;
        n = cJSON_GetArraySize(joints);
        for (i = 0; i < n; ++i)
            if (!parse_joint(cJSON_GetArrayItem(joints, i), c, i, &base, error, size)) goto out;
    }
    for (i = ARMS_JOINT_FIRST; i <= ARMS_JOINT_LAST; ++i) {
        arms_joint_config *j = &c->joint[i];
        if (!j->present) continue;
        if (!c->arm[j->arm].present) {
            fail(error, size, "关节 %d 属于 %s 臂，但 arms 段没有配置该臂", i, j->arm == 0 ? "left" : "right");
            goto out;
        }
        j->link = c->shared_link ? ARMS_ARM_LEFT : j->arm;
    }
    for (a = 0; a < ARMS_ARM_COUNT; ++a) {
        bool used[ARMS_MAX_SLAVES_PER_ARM] = {false};
        for (i = ARMS_JOINT_FIRST; i <= ARMS_JOINT_LAST; ++i) {
            const arms_joint_config *j = &c->joint[i];
            if (!j->present || j->link != a) continue;
            if (used[j->slave]) {
                if (c->shared_link)
                    fail(error, size, "共用链路上的 slave %d 被多个关节使用", j->slave);
                else
                    fail(error, size, "%s 臂的 slave %d 被多个关节使用", a == 0 ? "left" : "right", j->slave);
                goto out;
            }
            used[j->slave] = true;
        }
    }
    {
        /* arms.hub：两臂经集线器共用链路时，各臂所接的集线器输出口，启动时按实际拓扑核对 */
        const cJSON *hub = obj(arms, "hub");
        long v;
        if (hub) {
            if (!cJSON_IsObject(hub)) {
                fail(error, size, "arms.hub 必须是对象");
                goto out;
            }
            if (!c->shared_link) {
                fail(error, size, "arms.hub 只用于左右臂共用网口的配置");
                goto out;
            }
            if (!get_int(hub, "slave", 0, ARMS_MAX_SLAVES_PER_ARM - 1, &v)) {
                fail(error, size, "arms.hub.slave 必须是 0..%d", ARMS_MAX_SLAVES_PER_ARM - 1);
                goto out;
            }
            c->hub_slave = (int)v;
            for (i = ARMS_JOINT_FIRST; i <= ARMS_JOINT_LAST; ++i)
                if (c->joint[i].present && c->joint[i].slave == c->hub_slave) {
                    fail(error, size, "arms.hub.slave %d 与关节 %d 的 slave 相同", c->hub_slave, i);
                    goto out;
                }
            for (a = 0; a < ARMS_ARM_COUNT; ++a) {
                const char *key = a == ARMS_ARM_LEFT ? "left" : "right";
                if (!cJSON_GetObjectItemCaseSensitive(hub, key)) continue;
                if (!get_int(hub, key, 1, 3, &v)) {
                    fail(error, size, "arms.hub.%s 必须是集线器输出口 1..3", key);
                    goto out;
                }
                c->hub_out[a] = (int)v;
            }
            if (c->hub_out[0] && c->hub_out[0] == c->hub_out[1]) {
                fail(error, size, "arms.hub 中左右臂不能接同一个输出口 OUT%d", c->hub_out[0]);
                goto out;
            }
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
    if (!out || !size) return false;
    const char *candidates[] = {env, "/etc/bxi/arms.json", "config/arms.json"};
    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); ++i) {
        const char *p = candidates[i];
        if (p && *p && access(p, R_OK) == 0) {
            if (strlen(p) >= size) return false;
            strcpy(out, p);
            return true;
        }
    }
    len = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
    if (len > 0) {
        char *slash;
        exe[len] = '\0';
        slash = strrchr(exe, '/');
        if (slash) {
            *slash = '\0';
            const char *paths[] = {"../config/arms.json", "config/arms.json", "../share/arms_driver/config/arms.json",
                                   "../../share/arms_ros/config/arms.json"};
            for (size_t i = 0; i < sizeof(paths) / sizeof(paths[0]); ++i) {
                int n = snprintf(out, size, "%s/%s", exe, paths[i]);
                if (n >= 0 && (size_t)n < size && access(out, R_OK) == 0) return true;
            }
        }
    }
    return false;
}
