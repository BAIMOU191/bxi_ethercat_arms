#define _GNU_SOURCE
#include "config.h"
#include "cJSON.h"
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

/* Replace only a verified, regular configuration file on its original filesystem. */
bool arms_store_offset(const char *path, int joint, double value)
{
    struct stat st;
    if (!isfinite(value) || lstat(path, &st) || !S_ISREG(st.st_mode) || st.st_size <= 0 ||
        st.st_size > 1024 * 1024)
        return false;
    char tmp[1024];
    if (snprintf(tmp, sizeof(tmp), "%s.XXXXXX", path) >= (int)sizeof(tmp))
        return false;
    FILE *in = fopen(path, "rb");
    if (!in)
        return false;
    size_t n = (size_t)st.st_size;
    char *buf = calloc(n + 1, 1);
    bool ok = buf && fread(buf, 1, n, in) == n;
    fclose(in);
    cJSON *doc = ok ? cJSON_Parse(buf) : NULL;
    free(buf);
    cJSON *entry = NULL, *array = cJSON_GetObjectItemCaseSensitive(doc, "joints");
    ok = false;
    cJSON_ArrayForEach(entry, array)
    {
        const cJSON *id = cJSON_GetObjectItemCaseSensitive(entry, "joint");
        if (!cJSON_IsNumber(id) || id->valueint != joint)
            continue;
        cJSON_DeleteItemFromObjectCaseSensitive(entry, "offset_rad");
        ok = cJSON_AddNumberToObject(entry, "offset_rad", value) != NULL;
        break;
    }
    char *text = ok ? cJSON_Print(doc) : NULL;
    cJSON_Delete(doc);
    if (!text)
        return false;
    int fd = mkstemp(tmp);
    ok = false;
    if (fd >= 0) {
        FILE *out = fdopen(fd, "w");
        if (out) {
            ok = !fchown(fd, st.st_uid, st.st_gid) && !fchmod(fd, st.st_mode & 0777) &&
                 fprintf(out, "%s\n", text) > 0 && !fflush(out) && !fsync(fd);
            if (fclose(out))
                ok = false;
        } else
            close(fd);
        arms_config cfg;
        char error[128];
        if (ok)
            ok = arms_config_load(tmp, &cfg, error, sizeof(error)) &&
                 fabs(cfg.joint[joint].offset_rad - value) < 1e-9;
        if (ok)
            ok = rename(tmp, path) == 0;
        if (!ok)
            unlink(tmp);
    }
    cJSON_free(text);
    return ok;
}
