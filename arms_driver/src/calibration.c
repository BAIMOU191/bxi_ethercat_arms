/**
 * @file calibration.c
 * @brief 软件校零：只修改配置文件中指定关节的 offset_rad 数值，保留原有排版
 */
#define _GNU_SOURCE
#include "config.h"
#include "cJSON.h"

#include <ctype.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static const char *skip_ws(const char *p)
{
    while (*p && isspace((unsigned char)*p)) p++;
    return p;
}

/* 用解析器取得直接成员的源文本区间，不在字符串、嵌套对象或相邻关节中搜索。 */
static int member_span(const char *object, const char *name, const char **begin, const char **end)
{
    if (*object != '{') return -1;
    const char *p = skip_ws(object + 1);
    while (*p != '}') {
        const char *next;
        cJSON *key = cJSON_ParseWithOpts(p, &next, 0);
        bool match = cJSON_IsString(key) && strcmp(key->valuestring, name) == 0;
        bool valid = cJSON_IsString(key);
        cJSON_Delete(key);
        if (!valid || *(p = skip_ws(next)) != ':') return -1;
        p = skip_ws(p + 1);
        cJSON *item = cJSON_ParseWithOpts(p, &next, 0);
        if (!item) return -1;
        cJSON_Delete(item);
        if (match) {
            *begin = p;
            *end = next;
            return 1;
        }
        p = skip_ws(next);
        if (*p == '}') break;
        if (*p != ',') return -1;
        p = skip_ws(p + 1);
    }
    return 0;
}

static char *replace_offset(const char *text, int joint, double value)
{
    const char *array, *next;
    const char *root = strncmp(text, "\xef\xbb\xbf", 3) == 0 ? text + 3 : text;
    if (member_span(skip_ws(root), "joints", &array, &next) != 1 || *array != '[') return NULL;
    const char *p = skip_ws(array + 1);
    while (*p == '{') {
        cJSON *item = cJSON_ParseWithOpts(p, &next, 0);
        if (!item) return NULL;
        const cJSON *id = cJSON_GetObjectItemCaseSensitive(item, "joint");
        bool match = cJSON_IsNumber(id) && id->valuedouble == joint;
        cJSON_Delete(item);
        if (match) {
            const char *from, *to;
            char replacement[96];
            int found = member_span(p, "offset_rad", &from, &to);
            if (found < 0) return NULL;
            if (!found) from = to = p + 1;
            int n = snprintf(replacement, sizeof(replacement), found ? "%.6f" : " \"offset_rad\": %.6f,", value);
            if (n < 0 || (size_t)n >= sizeof(replacement)) return NULL;
            size_t prefix = (size_t)(from - text), tail = strlen(to) + 1;
            char *out = malloc(prefix + (size_t)n + tail);
            if (!out) return NULL;
            memcpy(out, text, prefix);
            memcpy(out + prefix, replacement, (size_t)n);
            memcpy(out + prefix + (size_t)n, to, tail);
            return out;
        }
        p = skip_ws(next);
        if (*p != ',') break;
        p = skip_ws(p + 1);
    }
    return NULL;
}

/* 写入临时文件、重新加载校验，通过后替换原文件 */
bool arms_store_offset(const char *path, int joint, double value)
{
    struct stat st;
    char tmp[1024];
    char *buf, *text;
    FILE *in;
    bool ok;
    int fd;

    if (!isfinite(value) || lstat(path, &st) || !S_ISREG(st.st_mode) || st.st_size <= 0 ||
        st.st_size > 1024 * 1024)
        return false;
    if (snprintf(tmp, sizeof(tmp), "%s.XXXXXX", path) >= (int)sizeof(tmp)) return false;
    in = fopen(path, "rb");
    if (!in) return false;
    buf = calloc((size_t)st.st_size + 1, 1);
    ok = buf && fread(buf, 1, (size_t)st.st_size, in) == (size_t)st.st_size;
    fclose(in);
    text = ok ? replace_offset(buf, joint, value) : NULL;
    free(buf);
    if (!text) return false;

    ok = false;
    fd = mkstemp(tmp);
    if (fd >= 0) {
        FILE *out = fdopen(fd, "w");
        if (out) {
            ok = !fchown(fd, st.st_uid, st.st_gid) && !fchmod(fd, st.st_mode & 0777) && fputs(text, out) >= 0 &&
                 !fflush(out) && !fsync(fd);
            if (fclose(out)) ok = false;
        } else {
            close(fd);
        }
        if (ok) {
            arms_config cfg;
            char error[256];
            ok = arms_config_load(tmp, &cfg, error, sizeof(error)) && fabs(cfg.joint[joint].offset_rad - value) < 1e-6;
        }
        if (ok) ok = rename(tmp, path) == 0;
        if (!ok) unlink(tmp);
    }
    free(text);
    return ok;
}
