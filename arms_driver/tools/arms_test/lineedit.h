/**
 * @file lineedit.h
 * @brief 交互终端的行编辑：左右移动、Home/End、Delete、上下键历史记录
 *
 * 标准输入不是终端时退化为普通的整行读取。
 */
#ifndef ARMS_TEST_LINEEDIT_H
#define ARMS_TEST_LINEEDIT_H

#include <stddef.h>

#define LE_OK 0
#define LE_EOF (-1)
#define LE_INTERRUPTED (-2)

/**
 * @brief 显示提示符并读取一行（不含换行符）
 * @return LE_OK；LE_EOF（Ctrl+D 或输入结束）；LE_INTERRUPTED（Ctrl+C）
 */
int le_readline(const char *prompt, char *buf, size_t size);

/** @brief 加入历史记录（空行和与上一条相同的行不记录） */
void le_history_add(const char *line);

#endif
