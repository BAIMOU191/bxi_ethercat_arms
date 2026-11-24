/**
 * @file lineedit.c
 * @brief 交互终端的行编辑
 *
 * 支持的按键：
 *   ← → / Ctrl+B Ctrl+F   移动光标        Home End / Ctrl+A Ctrl+E   行首/行尾
 *   ↑ ↓ / Ctrl+P Ctrl+N   历史记录        Backspace Delete           删除
 *   Ctrl+U / Ctrl+K       删除光标前/后   Ctrl+C 放弃本行            Ctrl+D 空行时退出
 * 小键盘在 NumLock 关闭时发送与方向键相同的序列。
 * 提示符和回显直接写入控制终端，不经过 -T 的时间戳管道。
 */
#define _GNU_SOURCE
#include "lineedit.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#define HISTORY_MAX 200

static char *g_history[HISTORY_MAX];
static int g_history_len;
static int g_out = -1;

static void out_str(const char *s, size_t n)
{
    while (n > 0) {
        ssize_t w = write(g_out, s, n);
        if (w < 0 && errno == EINTR) continue;
        if (w <= 0) return;
        s += w;
        n -= (size_t)w;
    }
}

static void out_cstr(const char *s) { out_str(s, strlen(s)); }

/* 显示宽度：ASCII 记 1 列，其余按 UTF-8 字符记 2 列 */
static size_t width(const char *s, size_t n)
{
    size_t w = 0, i;
    for (i = 0; i < n; ++i) {
        unsigned char c = (unsigned char)s[i];
        if (c < 0x80) w++;
        else if ((c & 0xC0) != 0x80) w += 2;
    }
    return w;
}

static void refresh(const char *prompt, const char *buf, size_t len, size_t pos)
{
    char seq[32];
    out_cstr("\r");
    out_cstr(prompt);
    out_str(buf, len);
    out_cstr("\x1b[0K\r");
    snprintf(seq, sizeof(seq), "\x1b[%zuC", width(prompt, strlen(prompt)) + width(buf, pos));
    if (width(prompt, strlen(prompt)) + width(buf, pos) > 0) out_cstr(seq);
}

void le_history_add(const char *line)
{
    if (!line || !*line) return;
    if (g_history_len > 0 && strcmp(g_history[g_history_len - 1], line) == 0) return;
    if (g_history_len == HISTORY_MAX) {
        free(g_history[0]);
        memmove(g_history, g_history + 1, sizeof(g_history[0]) * (HISTORY_MAX - 1));
        g_history_len--;
    }
    g_history[g_history_len] = strdup(line);
    if (g_history[g_history_len]) g_history_len++;
}

static int read_byte(unsigned char *c)
{
    ssize_t n = read(STDIN_FILENO, c, 1);
    if (n == 1) return 0;
    if (n < 0 && errno == EINTR) return LE_INTERRUPTED;
    return LE_EOF;
}

static int plain_readline(const char *prompt, char *buf, size_t size)
{
    size_t n;
    fputs(prompt, stdout);
    fflush(stdout);
    if (!fgets(buf, (int)size, stdin)) {
        if (ferror(stdin) && errno == EINTR) {
            clearerr(stdin);
            return LE_INTERRUPTED;
        }
        return LE_EOF;
    }
    n = strlen(buf);
    while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == '\r')) buf[--n] = '\0';
    return LE_OK;
}

int le_readline(const char *prompt, char *buf, size_t size)
{
    struct termios orig, raw;
    size_t len = 0, pos = 0;
    int hist = g_history_len, rc = LE_OK;
    char saved[1024] = "";

    if (size < 2) return LE_EOF;
    if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &orig) != 0) return plain_readline(prompt, buf, size);
    if (g_out < 0) {
        g_out = open("/dev/tty", O_WRONLY | O_CLOEXEC);
        if (g_out < 0) g_out = STDOUT_FILENO;
    }
    fflush(stdout);
    fflush(stderr);
    raw = orig;
    raw.c_iflag &= (tcflag_t) ~(ICRNL | IXON | INLCR);
    raw.c_lflag &= (tcflag_t) ~(ECHO | ICANON | IEXTEN); /* 保留 ISIG：Ctrl+C 仍产生 SIGINT */
    raw.c_cc[VMIN] = 1;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw) != 0) return plain_readline(prompt, buf, size);

    buf[0] = '\0';
    refresh(prompt, buf, len, pos);
    for (;;) {
        unsigned char c;
        int r = read_byte(&c);
        if (r != 0) {
            rc = r;
            break;
        }
        if (c == '\r' || c == '\n') break;
        if (c == 4) { /* Ctrl+D */
            if (len == 0) {
                rc = LE_EOF;
                break;
            }
            if (pos < len) {
                memmove(buf + pos, buf + pos + 1, len - pos);
                len--;
            }
        } else if (c == 127 || c == 8) { /* Backspace */
            if (pos > 0) {
                size_t start = pos - 1;
                while (start > 0 && ((unsigned char)buf[start] & 0xC0) == 0x80) start--;
                memmove(buf + start, buf + pos, len - pos + 1);
                len -= pos - start;
                pos = start;
            }
        } else if (c == 1) {
            pos = 0;
        } else if (c == 5) {
            pos = len;
        } else if (c == 2 || c == 6) {
            c = c == 2 ? 'D' : 'C';
            goto arrow;
        } else if (c == 16 || c == 14) {
            c = c == 16 ? 'A' : 'B';
            goto arrow;
        } else if (c == 21) { /* Ctrl+U */
            memmove(buf, buf + pos, len - pos + 1);
            len -= pos;
            pos = 0;
        } else if (c == 11) { /* Ctrl+K */
            len = pos;
            buf[len] = '\0';
        } else if (c == 27) { /* ESC 序列 */
            unsigned char s1, s2;
            if (read_byte(&s1) || read_byte(&s2)) continue;
            if (s1 == '[' && s2 >= '0' && s2 <= '9') {
                unsigned char s3;
                if (read_byte(&s3) || s3 != '~') continue;
                if (s2 == '1' || s2 == '7') pos = 0;
                else if (s2 == '4' || s2 == '8') pos = len;
                else if (s2 == '3' && pos < len) {
                    size_t end = pos + 1;
                    while (end < len && ((unsigned char)buf[end] & 0xC0) == 0x80) end++;
                    memmove(buf + pos, buf + end, len - end + 1);
                    len -= end - pos;
                }
            } else if (s1 == '[' || s1 == 'O') {
                c = s2;
            arrow:
                if (c == 'D' && pos > 0) {
                    pos--;
                    while (pos > 0 && ((unsigned char)buf[pos] & 0xC0) == 0x80) pos--;
                } else if (c == 'C' && pos < len) {
                    pos++;
                    while (pos < len && ((unsigned char)buf[pos] & 0xC0) == 0x80) pos++;
                } else if (c == 'H') {
                    pos = 0;
                } else if (c == 'F') {
                    pos = len;
                } else if ((c == 'A' && hist > 0) || (c == 'B' && hist < g_history_len)) {
                    if (hist == g_history_len) snprintf(saved, sizeof(saved), "%s", buf);
                    hist += c == 'A' ? -1 : 1;
                    snprintf(buf, size, "%s", hist == g_history_len ? saved : g_history[hist]);
                    len = pos = strlen(buf);
                }
            }
        } else if (c >= 32 && len + 1 < size) {
            memmove(buf + pos + 1, buf + pos, len - pos + 1);
            buf[pos++] = (char)c;
            len++;
        }
        refresh(prompt, buf, len, pos);
    }
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig);
    out_cstr("\r\n");
    if (rc != LE_OK) buf[0] = '\0';
    return rc;
}
