#ifndef ARMS_JOINT_SELECT_H
#define ARMS_JOINT_SELECT_H

#include <stddef.h>

#include "arms/arms.h"

/**
 * @brief 解析关节选择
 * @param text    "all"、"left"、"right"、"21"、"15-18"、"l_wrist_z"、"l_wrist_z_joint"，或以逗号分隔的组合
 * @param allowed 允许的关节，0 表示不限制
 * @param out     解析结果
 * @return 0 成功；-1 失败，err 中给出原因
 */
int arms_parse_joints(const char *text, arms_mask allowed, arms_mask *out, char *err, size_t err_size);

#endif
