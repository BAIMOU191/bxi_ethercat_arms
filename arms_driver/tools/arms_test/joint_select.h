#ifndef ARMS_JOINT_SELECT_H
#define ARMS_JOINT_SELECT_H

#include <stddef.h>

#include "arms/arms.h"

/*
 * Parse a joint selection: "all", "left", "right", "21", "15-18", "l_wrist_z",
 * "l_wrist_z_joint" or a comma-separated list of those. Joints outside
 * `allowed` are rejected. Returns 0 on success, -1 with a message in err.
 */
int arms_parse_joints(const char *text, arms_mask allowed, arms_mask *out, char *err, size_t err_size);

#endif
