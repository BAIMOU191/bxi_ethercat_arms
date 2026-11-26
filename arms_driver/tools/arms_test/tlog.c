/**
 * @file tlog.c
 * @brief arms_test 时间戳输出
 *
 * 事件行通过 tlog()/tlog_err() 输出并带时间戳。
 * tlog_stamp_all_lines() 将 stdout/stderr 重定向到管道，由转发线程为每一行加时间戳。
 */
#define _GNU_SOURCE
#include "tlog.h"

#include <pthread.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static bool g_all_lines;
static int g_saved_out = -1;
static int g_saved_err = -1;
static int g_pipe_rd = -1;
static pthread_t g_thread;

/* strftime 生成秒之前的部分；毫秒最多三位，缓冲区不够时不输出半截时间。 */
static const char *stamp(char *buf, size_t size, const char *format)
{
    struct timespec ts;
    struct tm tm;
    if (!size) return buf;
    size_t n = 0;
    if (clock_gettime(CLOCK_REALTIME, &ts) == 0 && localtime_r(&ts.tv_sec, &tm))
        n = strftime(buf, size, format, &tm);
    if (!n || size - n < 5) buf[0] = '\0';
    else snprintf(buf + n, size - n, ".%03u", (unsigned)(ts.tv_nsec / 1000000) % 1000u);
    return buf;
}

const char *tlog_time(char *buf, size_t size) { return stamp(buf, size, "%H:%M:%S"); }
const char *tlog_datetime(char *buf, size_t size) { return stamp(buf, size, "%Y-%m-%d %H:%M:%S"); }

static void vline(FILE *stream, const char *fmt, va_list ap)
{
    char ts[16];
    if (stream == stderr) fflush(stdout);
    if (!g_all_lines) fprintf(stream, "[%s] ", tlog_time(ts, sizeof(ts)));
    vfprintf(stream, fmt, ap);
    fputc('\n', stream);
    fflush(stream);
}

void tlog(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vline(stdout, fmt, ap);
    va_end(ap);
}

void tlog_err(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vline(stderr, fmt, ap);
    va_end(ap);
}

static void write_all(int fd, const char *p, size_t n)
{
    while (n > 0) {
        ssize_t w = write(fd, p, n);
        if (w <= 0) return;
        p += w;
        n -= (size_t)w;
    }
}

static void *forward_main(void *arg)
{
    char buf[4096], tm[16], prefix[24];
    bool line_start = true;
    ssize_t n;
    (void)arg;
    while ((n = read(g_pipe_rd, buf, sizeof(buf))) > 0) {
        ssize_t i, begin = 0;
        for (i = 0; i < n; ++i) {
            if (line_start) {
                int len = snprintf(prefix, sizeof(prefix), "[%s] ", tlog_time(tm, sizeof(tm)));
                write_all(g_saved_out, prefix, (size_t)len);
                line_start = false;
            }
            if (buf[i] == '\n') {
                write_all(g_saved_out, buf + begin, (size_t)(i - begin + 1));
                begin = i + 1;
                line_start = true;
            }
        }
        if (begin < n) write_all(g_saved_out, buf + begin, (size_t)(n - begin));
    }
    return NULL;
}

int tlog_stamp_all_lines(void)
{
    int fds[2];
    fflush(stdout);
    fflush(stderr);
    if (pipe(fds) != 0) return -1;
    g_saved_out = dup(STDOUT_FILENO);
    g_saved_err = dup(STDERR_FILENO);
    if (g_saved_out < 0 || g_saved_err < 0) goto fail;
    if (dup2(fds[1], STDOUT_FILENO) < 0 || dup2(fds[1], STDERR_FILENO) < 0) goto fail;
    g_pipe_rd = fds[0];
    if (pthread_create(&g_thread, NULL, forward_main, NULL) != 0) goto fail;
    close(fds[1]);
    setvbuf(stdout, NULL, _IOLBF, 0);
    g_all_lines = true;
    return 0;

fail:
    if (g_saved_out >= 0) { dup2(g_saved_out, STDOUT_FILENO); close(g_saved_out); }
    if (g_saved_err >= 0) { dup2(g_saved_err, STDERR_FILENO); close(g_saved_err); }
    close(fds[0]);
    close(fds[1]);
    g_saved_out = g_saved_err = g_pipe_rd = -1;
    return -1;
}

void tlog_shutdown(void)
{
    if (!g_all_lines) return;
    fflush(stdout);
    fflush(stderr);
    dup2(g_saved_out, STDOUT_FILENO);
    dup2(g_saved_err, STDERR_FILENO);
    pthread_join(g_thread, NULL);
    close(g_pipe_rd);
    close(g_saved_out);
    close(g_saved_err);
    g_all_lines = false;
}
