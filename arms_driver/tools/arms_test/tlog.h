#ifndef ARMS_TEST_TLOG_H
#define ARMS_TEST_TLOG_H

#include <stddef.h>

/** @brief 当前本地时间 "HH:MM:SS.mmm" */
const char *tlog_time(char *buf, size_t size);

/** @brief 当前本地时间 "YYYY-MM-DD HH:MM:SS.mmm" */
const char *tlog_datetime(char *buf, size_t size);

/** @brief 输出带时间戳的事件行到 stdout，自动换行 */
void tlog(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/** @brief 输出带时间戳的错误行到 stderr，自动换行 */
void tlog_err(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/**
 * @brief 为 stdout/stderr 的每一行加时间戳
 * @return 0 成功；-1 失败（输出保持不变）
 * @note 启用后 tlog()/tlog_err() 不再单独加时间戳
 */
int tlog_stamp_all_lines(void);

/** @brief 刷新并停止逐行时间戳，恢复原始 stdout/stderr */
void tlog_shutdown(void);

#endif
