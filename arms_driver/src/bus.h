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
#define BUS_SETTLE_MS 300u /* 从站数保持不变该时长后认为总线稳定 */

typedef void (*bus_log_fn)(void *user, const char *fmt, ...);

typedef struct {
    uint32_t vendor_id;
    uint32_t product_code;
    uint32_t revision;
    char name[64];
} bus_slave_info;

typedef struct {
    const char *interface;
    const char *mac;        /* 可为空 */
    int master;             /* -1 表示按 MAC 解析 */
    uint32_t cycle_ns;
    int32_t sync0_shift_ns;
    uint32_t vendor_id;
    uint32_t product_code;
    bus_log_fn log;
    void *log_user;
} bus_open_params;

typedef struct bus bus_t;

/* 扫描失败原因；仅空链可由 --no-check 跳过 */
enum { BUS_SCAN_MASTER = -1, BUS_SCAN_LINK = -2, BUS_SCAN_EMPTY = -3, BUS_SCAN_UNSTABLE = -4 };

typedef struct {
    const char *name;
    int (*open)(bus_t **out, const bus_open_params *params, char *err, size_t err_size);
    /*
     * 等待总线稳定后返回从站身份，写入 infos[0..*count-1]。
     * 稳定：从站数不少于 expected，身份可读，且从站数持续 BUS_SETTLE_MS 不变。
     * 超时时可返回数量不足但已稳定的拓扑，由调用方判断缺失；不使用仍在变化的扫描结果。
     */
    int (*scan)(bus_t *bus, bus_slave_info *infos, int max, int expected, int *count, unsigned int timeout_ms,
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
    int (*sdo)(bus_t *, int slave, uint16_t index, uint8_t sub, uint8_t *, size_t *, bool write,
               uint32_t *abort);

    /*
     * 异步请求，只在周期上下文中调用，均不阻塞。index 为第几个配置从站。
     * 每个从站一个寄存器请求（ESC 错误计数）和一组监控量请求（写 Pn002、读 0x3000），
     * 在 configure 中创建。start 类函数在上一次请求仍为 BUS_REQ_BUSY 时返回负值。
     */
    int (*esc_counters_read)(bus_t *bus, int index);
    int (*esc_counters_clear)(bus_t *bus, int index);
    /* 返回 BUS_REQ_*；为 BUS_REQ_DONE 且上一次是读时把 ESC_ERR_BYTES 字节写入 out */
    int (*esc_counters_state)(bus_t *bus, int index, uint8_t *out);
    int (*monitor_select)(bus_t *bus, int index, uint16_t code);
    int (*monitor_read)(bus_t *bus, int index);
    /* 返回 BUS_REQ_*；为 BUS_REQ_DONE 且上一次是读时写入 value */
    int (*monitor_state)(bus_t *bus, int index, int32_t *value);
    /* 取出一条 CoE 紧急报文（8 字节），可在非周期上下文调用；1 取到，0 没有，负值失败 */
    int (*emerg_pop)(bus_t *bus, int index, uint8_t *msg);
    /*
     * 拓扑：链上位置 position 的从站，其 ESC 端口 port（0..3）连接的下一个从站的链上位置。
     * 端口链路断开或没有下一个从站时返回 -1，读取失败返回 -2。在 scan 之后、周期开始之前调用。
     */
    int (*port_next)(bus_t *bus, int position, int port);
} bus_ops;

enum { BUS_REQ_IDLE = 0, BUS_REQ_BUSY, BUS_REQ_DONE, BUS_REQ_ERROR };

extern const bus_ops bus_mock_ops;
#ifdef ARMS_HAVE_ETHERLAB
extern const bus_ops bus_etherlab_ops;
#endif

#endif
