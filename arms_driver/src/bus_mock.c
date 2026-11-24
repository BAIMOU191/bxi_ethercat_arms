/**
 * @file bus_mock.c
 * @brief 模拟总线后端，用于离线调试和单元测试
 *
 * 环境变量：
 *   ARMS_MOCK_SLAVES=<n>       每条臂的从站数（默认 7）
 *   ARMS_MOCK_DROP_PPM=<ppm>   每周期 WKC 为 0 的概率（故障注入）
 *   ARMS_MOCK_FAULT_CYCLE=<n>  第 n 周期后，主站 0 链上位置 0 的从站在使能状态下报故障
 *   ARMS_MOCK_OVERSPEED_CYCLE=<n> 第 n 周期后，主站 0 链上位置 0 的从站在使能状态下反馈速度 10 rad/s
 *   ARMS_MOCK_POWER_OFF=<s>,<n> 主站 0 从第 s 周期起掉电 n 个周期，恢复后从站报同步丢失故障
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "bus.h"

struct bus {
    uint16_t params[BUS_MAX_SLAVES][0x200];
    int slaves;
    int configured;
    int positions[BUS_MAX_SLAVES];
    uint8_t out[BUS_MAX_SLAVES][KGU_OUT_BYTES];
    uint8_t in[BUS_MAX_SLAVES][KGU_IN_BYTES];
    float pos[BUS_MAX_SLAVES];
    uint16_t sw[BUS_MAX_SLAVES];
    uint64_t fault_cycle;
    uint64_t overspeed_cycle;
    uint64_t off_start, off_len;
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
    s = getenv("ARMS_MOCK_POWER_OFF");
    if (s) sscanf(s, "%llu,%llu", (unsigned long long *)&b->off_start, (unsigned long long *)&b->off_len);
    s = getenv("ARMS_MOCK_FAULT_CYCLE");
    b->fault_cycle = s ? strtoull(s, NULL, 10) : 0;
    s = getenv("ARMS_MOCK_OVERSPEED_CYCLE");
    b->overspeed_cycle = s ? strtoull(s, NULL, 10) : 0;
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
        b->sw[i] = 0x0250u;
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
    if (b->off_len && b->master == 0 && b->cycles >= b->off_start) {
        uint64_t k = b->cycles - b->off_start;
        if (k < b->off_len) return 0x00;
        if (k < b->off_len + 300) return 0x02;
        if (k < b->off_len + 600) return 0x04;
        return 0x08;
    }
    if (dt < 300000000ull) return 0x02;
    if (dt < 600000000ull) return 0x04;
    return 0x08;
}

/* CiA402 状态机的简化模拟 */
static uint16_t mock_drive(bus_t *b, int i, uint16_t cw)
{
    uint16_t st = b->sw[i] & 0x006Fu;
    if (b->sw[i] & 0x0008u) {
        if (cw & 0x0080u) b->sw[i] = 0x0250u;
        return b->sw[i];
    }
    if (b->fault_cycle && b->master == 0 && b->positions[i] == 0 && b->cycles >= b->fault_cycle && st == 0x0027u) {
        b->fault_cycle = 0;
        b->sw[i] = 0x0218u;
        return b->sw[i];
    }
    switch (cw & 0x008Fu) {
    case 0x0006u: b->sw[i] = 0x0231u; break;
    case 0x0007u: if (st == 0x0021u || st == 0x0023u || st == 0x0027u) b->sw[i] = 0x0233u; break;
    case 0x000Fu: if (st == 0x0023u || st == 0x0027u) b->sw[i] = 0x0237u; break;
    default: b->sw[i] = 0x0250u; break;
    }
    return b->sw[i];
}

static int mock_receive(bus_t *b)
{
    uint8_t al = mock_al(b);
    int i;
    b->cycles++;
    if (b->off_len && b->master == 0 && b->cycles == b->off_start + b->off_len)
        for (i = 0; i < b->configured; ++i) b->sw[i] = 0x0218u; /* 上电后报同步丢失故障 */
    for (i = 0; i < b->configured; ++i) {
        kgu_feedback f;
        kgu_command c;
        uint8_t tmp[KGU_IN_BYTES];
        float prev = b->pos[i];
        c.controlword = kgu_get_u16(b->out[i], KGU_OUT_CONTROLWORD);
        c.pos = kgu_get_f32(b->out[i], KGU_OUT_TARGET_POS);
        c.kp = kgu_get_f32(b->out[i], KGU_OUT_KP);
        f.statusword = al == 0x08 ? mock_drive(b, i, c.controlword) : 0x0250u;
        if ((f.statusword & 0x006Fu) == 0x0027u && c.kp > 0.0f) b->pos[i] += (c.pos - b->pos[i]) * 0.02f;
        f.pos = b->pos[i];
        f.vel = (b->pos[i] - prev) * 1000.0f;
        if (b->overspeed_cycle && b->master == 0 && b->positions[i] == 0 && b->cycles >= b->overspeed_cycle &&
            (f.statusword & 0x006Fu) == 0x0027u)
            f.vel = 10.0f;
        f.torque = 0.0f;
        f.error = (f.statusword & 0x0008u) ? 0x7500u : 0;
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

static int mock_sdo(bus_t *b, int slave, uint16_t index, uint8_t sub, uint8_t *data, size_t *n, bool write,
                    uint32_t *abort)
{
    if (slave < 0 || slave >= BUS_MAX_SLAVES)
        return -1;
    if (index == 0x1018 && sub >= 1 && sub <= 4 && !write && *n >= 4) {
        uint32_t x = sub == 1   ? KGU_VENDOR_ID
                     : sub == 2 ? KGU_PRODUCT_CODE
                     : sub == 3 ? 1u
                                : (uint32_t)(1 + slave + 16 * b->master);
        for (unsigned k = 0; k < 4; k++)
            data[k] = (uint8_t)(x >> (8 * k));
        *n = 4;
        return 0;
    }
    if (sub || index < 0x2000 || index >= 0x2200 || *n < 2) {
        *abort = 0x06020000;
        return -1;
    }
    uint16_t *v = &b->params[slave][index - 0x2000];
    if (write)
        *v = kgu_get_u16(data, 0);
    else
        kgu_put_u16(data, 0, *v);
    *n = 2;
    return 0;
}

const bus_ops bus_mock_ops = {
    "mock",     mock_open,   mock_scan,    mock_configure,     mock_activate,     mock_receive,
    mock_send,  mock_inputs, mock_outputs, mock_update_states, mock_expected_wkc, mock_master_index,
    mock_close, mock_sdo,
};
