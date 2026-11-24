/*
 * arms.h - BXI dual-arm EtherCAT driver (KGU actuators, MIT mode)
 *
 * Joint numbering follows the BXI 31-joint standard order:
 *   15..21  left  shoulder_y/x/z, elbow_y, wrist_x/y/z
 *   22..28  right shoulder_y/x/z, elbow_y, wrist_x/y/z
 * Bus position ("slave") is the EtherLab ring position, starting at 0.
 */
#ifndef ARMS_ARMS_H
#define ARMS_ARMS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ARMS_JOINT_FIRST 15
#define ARMS_JOINT_LAST 28
#define ARMS_JOINT_COUNT 14
#define ARMS_ARM_COUNT 2
#define ARMS_ARM_LEFT 0
#define ARMS_ARM_RIGHT 1
#define ARMS_MAX_SLAVES_PER_ARM 16

typedef struct arms arms_t;

typedef enum {
    ARMS_OK = 0,
    ARMS_E_ARG,         /* invalid argument */
    ARMS_E_CONFIG,      /* configuration file invalid */
    ARMS_E_SYSTEM,      /* OS call failed (permissions, memory, threads) */
    ARMS_E_MASTER,      /* EtherCAT master unavailable or busy */
    ARMS_E_LINK,        /* no physical link */
    ARMS_E_TOPOLOGY,    /* slaves found do not match the configuration */
    ARMS_E_STATE,       /* slaves did not reach the requested state */
    ARMS_E_COMM,        /* cyclic communication failed */
    ARMS_E_TIMEOUT,     /* operation timed out */
    ARMS_E_UNSUPPORTED, /* not implemented in this phase / backend */
} arms_status;

const char *arms_strerror(arms_status status);
/* Detailed message for the last failure in the calling thread. */
const char *arms_last_error(void);

/* Bitmask of joints, bit n = joint n (only 15..28 are meaningful). */
typedef uint32_t arms_mask;
#define ARMS_MASK_JOINT(j) ((arms_mask)1u << (j))
#define ARMS_MASK_LEFT 0x003F8000u  /* 15..21 */
#define ARMS_MASK_RIGHT 0x1FC00000u /* 22..28 */
#define ARMS_MASK_ALL (ARMS_MASK_LEFT | ARMS_MASK_RIGHT)

typedef enum {
    ARMS_BACKEND_ETHERLAB = 0,
    ARMS_BACKEND_MOCK = 1,
} arms_backend;

typedef void (*arms_log_fn)(void *user, const char *line);

typedef struct {
    const char *config_path;   /* NULL = default search path */
    arms_backend backend;
    unsigned int cycle_hz;     /* 0 = use configuration */
    arms_mask joints;          /* joints to bring up; 0 = all configured */
    bool skip_selftest;
    arms_log_fn log;           /* progress/diagnostic lines (non-realtime) */
    void *log_user;
} arms_options;

void arms_options_init(arms_options *options);

typedef enum {
    ARMS_JS_ABSENT = 0, /* not selected / not configured */
    ARMS_JS_OFFLINE,    /* configured but not seen on the bus */
    ARMS_JS_PREOP,
    ARMS_JS_SAFEOP,
    ARMS_JS_LOCKED,     /* OP, servo off, brake per Pn075 */
    ARMS_JS_RELEASED,   /* phase 4 */
    ARMS_JS_ENABLED,    /* phase 3 */
    ARMS_JS_FAULT,
} arms_joint_mode;

const char *arms_joint_mode_name(arms_joint_mode mode);

typedef struct {
    int joint;
    const char *name;
    int arm;
    int slave;
    arms_joint_mode mode;
    uint8_t al_state;        /* raw EtherCAT AL state */
    uint16_t status_word;    /* CiA402 0x6041 */
    uint16_t error_code;     /* 0x603F */
    double position;         /* rad, output side, direction/offset applied */
    double velocity;         /* rad/s */
    double torque;           /* N.m */
    uint64_t sample_ns;      /* CLOCK_MONOTONIC time of the sample */
} arms_joint_state;

typedef struct {
    double min, mean, max, rms_jitter; /* microseconds */
} arms_interval_stats;

typedef struct {
    bool present;            /* arm selected and opened */
    char interface[32];
    int master_index;
    int slaves_expected;
    int slaves_found;
    int slaves_op;
    bool link_up;
    uint32_t wkc_expected;
    uint32_t wkc_last;
    uint64_t cycles;
    uint64_t wkc_bad;            /* cycles with WKC != expected */
    uint32_t wkc_bad_consecutive_max;
    uint64_t send_errors;
    bool comm_failed;            /* tolerance exceeded */
    double recv_us_max;          /* time spent in receive+process */
    double send_us_max;
} arms_arm_stats;

typedef struct {
    unsigned int cycle_hz;
    uint64_t cycles;
    uint64_t overruns;           /* wake-up later than 1.5 periods */
    uint64_t skipped;            /* whole periods skipped */
    arms_interval_stats period;  /* time between successive cycle starts */
    double wake_late_us_max;     /* wake-up latency vs. target */
    double work_us_max;          /* time spent in one cycle */
    arms_arm_stats arm[ARMS_ARM_COUNT];
} arms_bus_stats;

arms_status arms_open(const arms_options *options, arms_t **out);
void arms_close(arms_t *arms);

arms_status arms_read(arms_t *arms, arms_joint_state *states, size_t count, size_t *written);
arms_status arms_read_joint(arms_t *arms, int joint, arms_joint_state *state);
arms_status arms_get_stats(arms_t *arms, arms_bus_stats *stats);
void arms_reset_stats(arms_t *arms);

/* Configuration lookups (valid for the lifetime of the handle). */
unsigned int arms_cycle_hz(const arms_t *arms);
const char *arms_backend_name(const arms_t *arms);
const char *arms_config_path(const arms_t *arms);
const char *arms_joint_name(const arms_t *arms, int joint);
int arms_joint_by_name(const arms_t *arms, const char *name); /* -1 if unknown */
arms_mask arms_opened_joints(const arms_t *arms);
const char *arms_arm_name(int arm);

/* Phase 3+ (return ARMS_E_UNSUPPORTED in phase 1). */
arms_status arms_enable(arms_t *arms, arms_mask joints);
arms_status arms_disable(arms_t *arms, arms_mask joints);

const char *arms_version(void);

#ifdef __cplusplus
}
#endif

#endif
