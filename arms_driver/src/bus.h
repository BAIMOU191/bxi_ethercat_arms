/**
 * @file bus.h
 * @brief 总线后端接口，每条臂一个实例（对应一个 EtherCAT 主站）
 *
 * open/scan/configure/activate/close 在周期开始前或结束后调用；
 * receive/send/inputs/outputs/update_states 只在周期上下文中调用。
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
    /* 等待从站身份就绪（最长 timeout_ms），写入 infos[0..*count-1] */
    int (*scan)(bus_t *bus, bus_slave_info *infos, int max, int *count, unsigned int timeout_ms,
                char *err, size_t err_size);
    /* 为链上位置 positions[0..n-1] 配置 MIT PDO 和 DC Sync0 */
    int (*configure)(bus_t *bus, const int *positions, int n, char *err, size_t err_size);
    int (*activate)(bus_t *bus, char *err, size_t err_size);
    /* 接收并处理过程数据，返回 WKC；失败返回负值 */
    int (*receive)(bus_t *bus);
    /* 写入 DC 应用时间并发送过程数据 */
    int (*send)(bus_t *bus, uint64_t app_time_ns);
    /* 第 index 个配置从站的过程数据 */
    const uint8_t *(*inputs)(bus_t *bus, int index);
    uint8_t *(*outputs)(bus_t *bus, int index);
    /* 刷新 AL 状态和链路状态（低频调用） */
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
