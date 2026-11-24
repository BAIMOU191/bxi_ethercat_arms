#ifndef ARMS_CONFIG_H
#define ARMS_CONFIG_H

#include "arms/arms.h"

#define ARMS_NAME_LEN 48
#define ARMS_PATH_LEN 512

typedef struct {
    bool present;
    char interface[32];
    char mac[24];
    int master; /* -1 = resolve from /sys/module/ec_master/parameters/main_devices */
} arms_arm_config;

typedef struct {
    bool present;
    int joint;
    char name[ARMS_NAME_LEN];
    int arm;
    int slave;
    char model[24];
    int direction;          /* +1 / -1 */
    double offset_rad;
    double pos_min, pos_max;
    double vel_max, tff_max;
    double kp_max, kd_max;
    double kp_default, kd_default;
    int pn082_expected;     /* -1 = not checked */
    char brake[16];         /* "normal" | "release" | "lock" */
} arms_joint_config;

typedef struct {
    char path[ARMS_PATH_LEN];
    int version;
    unsigned int cycle_hz;
    int rt_cpu;
    int irq_cpu;
    int rt_priority;
    unsigned int wkc_tolerance;
    unsigned int command_timeout_ms;
    char timeout_action[16];
    unsigned int release_max_joints;
    bool power_managed;
    unsigned int op_timeout_ms;
    uint32_t vendor_id;
    uint32_t product_code;
    arms_arm_config arm[ARMS_ARM_COUNT];
    arms_joint_config joint[ARMS_JOINT_LAST + 1]; /* indexed by joint number */
} arms_config;

/* Standard BXI joint names, indexed by joint number 0..30. */
extern const char *const arms_standard_joint_names[31];

bool arms_cycle_hz_supported(unsigned int hz);

/* Load and validate. On failure returns false and writes a message to error. */
bool arms_config_load(const char *path, arms_config *config, char *error, size_t error_size);

/* Default search: $ARMS_CONFIG, /etc/bxi/arms.json, ./config/arms.json,
 * <exe dir>/../config/arms.json. Returns false if none is readable. */
bool arms_config_default_path(char *out, size_t size);

#endif
