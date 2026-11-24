/**
 * @file bus_mock.c
 * @brief 模拟总线后端，用于离线调试和单元测试
 *
 * 环境变量：
 *   ARMS_MOCK_SLAVES=<n>       每条臂的从站数（默认 7）
 *   ARMS_MOCK_DROP_PPM=<ppm>   每周期 WKC 为 0 的概率（故障注入）
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "bus.h"

struct bus {
    int slaves;
    int configured;
    int positions[BUS_MAX_SLAVES];
    uint8_t out[BUS_MAX_SLAVES][KGU_OUT_BYTES];
    uint8_t in[BUS_MAX_SLAVES][KGU_IN_BYTES];
    float pos[BUS_MAX_SLAVES];
    uint64_t activated_ns;
    uint64_t cycles;
    unsigned long drop_ppm;
    uint32_t rng;
    int master;
};

static uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static int mock_open(bus_t **out, const bus_open_params *p, char *err, size_t err_size)
{
    bus_t *b = calloc(1, sizeof(*b));
    const char *s;
    if (!b) {
        snprintf(err, err_size, "内存不足");
        return -1;
    }
    b->slaves = 7;
    s = getenv("ARMS_MOCK_SLAVES");
    if (s) {
        int n = atoi(s);
        if (n >= 0 && n <= BUS_MAX_SLAVES) b->slaves = n;
    }
    s = getenv("ARMS_MOCK_DROP_PPM");
    b->drop_ppm = s ? strtoul(s, NULL, 10) : 0;
    b->rng = 0x12345678u;
    b->master = p->master;
    if (b->master < 0) {
        size_t n = p->interface ? strlen(p->interface) : 0;
        b->master = (n > 0 && p->interface[n - 1] >= '0' && p->interface[n - 1] <= '9') ? p->interface[n - 1] - '0' : 0;
    }
    *out = b;
    return 0;
}

static int mock_scan(bus_t *b, bus_slave_info *infos, int max, int *count, unsigned int timeout_ms,
                     char *err, size_t err_size)
{
    int i;
    (void)timeout_ms;
    (void)err;
    (void)err_size;
    for (i = 0; i < b->slaves && i < max; ++i) {
        infos[i].vendor_id = KGU_VENDOR_ID;
        infos[i].product_code = KGU_PRODUCT_CODE;
        infos[i].revision = 0x00050012u;
        snprintf(infos[i].name, sizeof(infos[i].name), "MOCK KDE EtherCAT Drive");
    }
    *count = i;
    return 0;
}

static int mock_configure(bus_t *b, const int *positions, int n, char *err, size_t err_size)
{
    int i;
    for (i = 0; i < n; ++i) {
        if (positions[i] < 0 || positions[i] >= b->slaves) {
            snprintf(err, err_size, "mock: slave %d 不存在", positions[i]);
            return -1;
        }
        b->positions[i] = positions[i];
        b->pos[i] = 0.1f * (float)(positions[i] + 1) * ((positions[i] & 1) ? -1.0f : 1.0f);
    }
    b->configured = n;
    return 0;
}

static int mock_activate(bus_t *b, char *err, size_t err_size)
{
    (void)err;
    (void)err_size;
    b->activated_ns = now_ns();
    return 0;
}

static uint8_t mock_al(const bus_t *b)
{
    uint64_t dt = now_ns() - b->activated_ns;
    if (b->activated_ns == 0) return 0x02;
    if (dt < 300000000ull) return 0x02;
    if (dt < 600000000ull) return 0x04;
    return 0x08;
}

static int mock_receive(bus_t *b)
{
    uint8_t al = mock_al(b);
    int i;
    b->cycles++;
    for (i = 0; i < b->configured; ++i) {
        kgu_feedback f;
        kgu_command c;
        uint8_t tmp[KGU_IN_BYTES];
        c.controlword = kgu_get_u16(b->out[i], KGU_OUT_CONTROLWORD);
        f.statusword = (c.controlword & 0x000Fu) == 0x000Fu ? 0x0237u : 0x0250u;
        f.pos = b->pos[i];
        f.vel = 0.0f;
        f.torque = 0.0f;
        f.error = 0;
        memset(tmp, 0, sizeof(tmp));
        kgu_put_u16(tmp, KGU_IN_STATUSWORD, f.statusword);
        kgu_put_f32(tmp, KGU_IN_POS, f.pos);
        kgu_put_f32(tmp, KGU_IN_VEL, f.vel);
        kgu_put_f32(tmp, KGU_IN_TORQUE, f.torque);
        kgu_put_u16(tmp, KGU_IN_ERROR, f.error);
        memcpy(b->in[i], tmp, sizeof(tmp));
    }
    if (b->drop_ppm) {
        b->rng = b->rng * 1103515245u + 12345u;
        if ((b->rng >> 8) % 1000000u < b->drop_ppm) return 0;
    }
    if (al == 0x08) return 3 * b->configured;
    if (al == 0x04) return b->configured;
    return 0;
}

static int mock_send(bus_t *b, uint64_t app_time_ns)
{
    (void)b;
    (void)app_time_ns;
    return 0;
}

static const uint8_t *mock_inputs(bus_t *b, int i) { return b->in[i]; }
static uint8_t *mock_outputs(bus_t *b, int i) { return b->out[i]; }

static void mock_update_states(bus_t *b, uint8_t *al, bool *link_up)
{
    uint8_t s = mock_al(b);
    int i;
    for (i = 0; i < b->configured; ++i) al[i] = s;
    *link_up = true;
}

static uint32_t mock_expected_wkc(bus_t *b) { return (uint32_t)(3 * b->configured); }
static int mock_master_index(bus_t *b) { return b->master; }
static void mock_close(bus_t *b) { free(b); }

const bus_ops bus_mock_ops = {
    "mock", mock_open, mock_scan, mock_configure, mock_activate, mock_receive, mock_send,
    mock_inputs, mock_outputs, mock_update_states, mock_expected_wkc, mock_master_index, mock_close,
};
