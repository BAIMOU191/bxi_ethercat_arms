/*
 * Bus backend interface: one instance per arm (one EtherCAT master).
 *
 * Threading: open/scan/configure/activate/close run in the caller's thread
 * before or after the realtime loop. receive/send/inputs/outputs/
 * update_states run only in the realtime thread.
 */
#ifndef ARMS_BUS_H
#define ARMS_BUS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "kgu.h"

#define BUS_MAX_SLAVES 16

typedef void (*bus_log_fn)(void *user, const char *fmt, ...);

typedef struct {
    uint32_t vendor_id;
    uint32_t product_code;
    uint32_t revision;
    char name[64];
} bus_slave_info;

typedef struct {
    const char *interface;
    const char *mac;        /* may be empty */
    int master;             /* -1 = resolve from MAC */
    uint32_t cycle_ns;
    int32_t sync0_shift_ns;
    uint32_t vendor_id;
    uint32_t product_code;
    bus_log_fn log;
    void *log_user;
} bus_open_params;

typedef struct bus bus_t;

typedef struct {
    const char *name;
    int (*open)(bus_t **out, const bus_open_params *params, char *err, size_t err_size);
    /* Fill infos[0..*count-1]; wait up to timeout_ms for identities. */
    int (*scan)(bus_t *bus, bus_slave_info *infos, int max, int *count, unsigned int timeout_ms,
                char *err, size_t err_size);
    /* Configure MIT PDOs + DC Sync0 for slaves[0..n-1] (ring positions). */
    int (*configure)(bus_t *bus, const int *positions, int n, char *err, size_t err_size);
    int (*activate)(bus_t *bus, char *err, size_t err_size);
    /* Realtime: returns working counter of the last exchange, <0 on error. */
    int (*receive)(bus_t *bus);
    int (*send)(bus_t *bus, uint64_t app_time_ns);
    /* Realtime: process data pointers for the i-th configured slave. */
    const uint8_t *(*inputs)(bus_t *bus, int index);
    uint8_t *(*outputs)(bus_t *bus, int index);
    /* Realtime, low rate: refresh AL states into al[0..n-1]; link state. */
    void (*update_states)(bus_t *bus, uint8_t *al, bool *link_up);
    uint32_t (*expected_wkc)(bus_t *bus);
    int (*master_index)(bus_t *bus);
    void (*close)(bus_t *bus);
} bus_ops;

extern const bus_ops bus_mock_ops;
#ifdef ARMS_HAVE_ETHERLAB
extern const bus_ops bus_etherlab_ops;
#endif

#endif
