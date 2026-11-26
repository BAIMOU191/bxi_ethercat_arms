#include "joint_select.h"

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int parse_number(const char *s, int *out)
{
    char *end;
    long v;
    if (!s || !*s || !isdigit((unsigned char)*s)) return -1;
    errno = 0;
    v = strtol(s, &end, 10);
    if (errno || *end != '\0' || v > INT_MAX) return -1;
    *out = (int)v;
    return 0;
}

static int parse_item(const char *item, arms_mask *mask, char *err, size_t err_size)
{
    const char *dash;
    int a, b, j;
    if (strcmp(item, "all") == 0) {
        *mask |= ARMS_MASK_ALL;
        return 0;
    }
    if (strcmp(item, "left") == 0 || strcmp(item, "l") == 0) {
        *mask |= ARMS_MASK_LEFT;
        return 0;
    }
    if (strcmp(item, "right") == 0 || strcmp(item, "r") == 0) {
        *mask |= ARMS_MASK_RIGHT;
        return 0;
    }
    dash = strchr(item, '-');
    if (dash && isdigit((unsigned char)item[0])) {
        char left[16];
        size_t n = (size_t)(dash - item);
        if (n >= sizeof(left)) goto bad;
        memcpy(left, item, n);
        left[n] = '\0';
        if (parse_number(left, &a) != 0 || parse_number(dash + 1, &b) != 0 || a > b) goto bad;
        if (a < ARMS_JOINT_FIRST || b > ARMS_JOINT_LAST) {
            snprintf(err, err_size, "关节范围 %s 超出 %d-%d", item, ARMS_JOINT_FIRST, ARMS_JOINT_LAST);
            return -1;
        }
        for (j = a; j <= b; ++j) *mask |= ARMS_MASK_JOINT(j);
        return 0;
    }
    if (parse_number(item, &a) == 0) {
        if (a < ARMS_JOINT_FIRST || a > ARMS_JOINT_LAST) {
            snprintf(err, err_size, "关节号 %d 超出范围（双臂是 %d-%d）", a, ARMS_JOINT_FIRST, ARMS_JOINT_LAST);
            return -1;
        }
        *mask |= ARMS_MASK_JOINT(a);
        return 0;
    }
    j = arms_joint_by_name(NULL, item);
    if (j >= 0) {
        *mask |= ARMS_MASK_JOINT(j);
        return 0;
    }
bad:
    snprintf(err, err_size, "无法识别关节 \"%s\"（可用: 15-28、l_wrist_z、left、right、all、15-18、21,28）", item);
    return -1;
}

int arms_parse_joints(const char *text, arms_mask allowed, arms_mask *out, char *err, size_t err_size)
{
    char buf[256];
    char *save = NULL, *tok;
    arms_mask mask = 0;
    if (!text || !*text) {
        snprintf(err, err_size, "未指定关节");
        return -1;
    }
    if (text[0] == ',' || text[strlen(text) - 1] == ',' || strstr(text, ",,")) {
        snprintf(err, err_size, "关节列表含空项");
        return -1;
    }
    if (strlen(text) >= sizeof(buf)) {
        snprintf(err, err_size, "关节列表过长");
        return -1;
    }
    snprintf(buf, sizeof(buf), "%s", text);
    for (tok = strtok_r(buf, ",", &save); tok; tok = strtok_r(NULL, ",", &save)) {
        arms_mask item = 0;
        if (parse_item(tok, &item, err, err_size) != 0) return -1;
        /* all / left / right 按会话取交集；显式编号、名称或范围不能只执行一部分。 */
        if (allowed && (item & ~allowed) && strcmp(tok, "all") && strcmp(tok, "left") &&
            strcmp(tok, "l") && strcmp(tok, "right") && strcmp(tok, "r")) {
            snprintf(err, err_size, "关节选择 %s 包含本次不可用的关节", tok);
            return -1;
        }
        mask |= item;
    }
    if (allowed) {
        arms_mask outside = mask & ~allowed;
        if (outside == mask) {
            snprintf(err, err_size, "所选关节都没有在本次会话中打开");
            return -1;
        }
        mask &= allowed;
    }
    *out = mask;
    return 0;
}
