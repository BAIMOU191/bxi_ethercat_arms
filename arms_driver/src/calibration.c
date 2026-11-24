/**
 * @file calibration.c
 * @brief 软件校零：只修改配置文件中指定关节的 offset_rad 数值，保留原有排版
 */
#define _GNU_SOURCE
#include "config.h"

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

/* 在 text 中查找 "key": 的值起点，搜索范围 [from, end)；未找到返回 NULL */
static const char *find_value(const char *from, const char *end, const char *key)
{
    size_t klen = strlen(key);
    const char *p = from;
    while (p < end && (p = strstr(p, key)) != NULL && p < end) {
        const char *q = skip_ws(p + klen);
        if (*q == ':') return skip_ws(q + 1);
        p += klen;
    }
    return NULL;
}

/* 查找 "joint": <joint> 所在关节对象的起点（数值之后的位置），以及下一个关节对象的起点 */
static bool locate_joint(const char *text, int joint, const char **after_number, const char **object_end)
{
    const char *p = text;
    while ((p = find_value(p, text + strlen(text), "\"joint\"")) != NULL) {
        char *num_end;
        long v = strtol(p, &num_end, 10);
        if (num_end != p && v == joint) {
            const char *next = find_value(num_end, text + strlen(text), "\"joint\"");
            *after_number = num_end;
            *object_end = next ? next : text + strlen(text);
            return true;
        }
        p = num_end != p ? num_end : p + 1;
    }
    return false;
}

static char *replace_offset(const char *text, int joint, double value)
{
    const char *after, *end, *val;
    char number[48];
    char *out;
    size_t prefix, total;

    if (!locate_joint(text, joint, &after, &end)) return NULL;
    snprintf(number, sizeof(number), "%.6f", value);
    val = find_value(after, end, "\"offset_rad\"");
    if (val) {
        char *val_end;
        strtod(val, &val_end);
        if (val_end == val) return NULL;
        prefix = (size_t)(val - text);
        total = prefix + strlen(number) + strlen(val_end) + 1;
        out = malloc(total);
        if (!out) return NULL;
        memcpy(out, text, prefix);
        snprintf(out + prefix, total - prefix, "%s%s", number, val_end);
        return out;
    }
    /* 原来没有 offset_rad：插在 "joint": N 之后的逗号后面 */
    {
        const char *comma = skip_ws(after);
        char insert[80];
        if (*comma != ',') return NULL;
        snprintf(insert, sizeof(insert), " \"offset_rad\": %s,", number);
        prefix = (size_t)(comma + 1 - text);
        total = prefix + strlen(insert) + strlen(comma + 1) + 1;
        out = malloc(total);
        if (!out) return NULL;
        memcpy(out, text, prefix);
        snprintf(out + prefix, total - prefix, "%s%s", insert, comma + 1);
        return out;
    }
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
