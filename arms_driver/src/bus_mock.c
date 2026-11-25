/**
 * @file bus_mock.c
 * @brief 模拟总线后端，用于离线调试和单元测试
 *
 * 环境变量：
 *   ARMS_MOCK_SLAVES=<n>       每条臂的从站数（默认 7）
 *   ARMS_MOCK_DROP_PPM=<ppm>   每周期 WKC 为 0 的概率（故障注入）
 *   ARMS_MOCK_FAULT_CYCLE=<n>  第 n 周期后，主站 0 链上位置 0 的从站在使能状态下报故障
 *   ARMS_MOCK_OVERSPEED_CYCLE=<n> 第 n 周期后，主站 0 链上位置 0 的从站在使能状态下反馈速度 10 rad/s
 *   ARMS_MOCK_STUCK_ENABLED=1    模拟已使能的驱动器不响应失能请求
 *   ARMS_MOCK_PN075_UNREADABLE=1  模拟抱闸参数读取失败
 *   ARMS_MOCK_POWER_OFF=<s>,<n> 主站 0 从第 s 周期起掉电 n 个周期，恢复后从站报同步丢失故障
 *   ARMS_MOCK_ESC_ERR=<n>      第 n 周期起，主站 0 链上位置 3 的从站端口 1 错误计数每 200 周期加 1
 *   ARMS_MOCK_ESC_PROC=<n>     第 n 周期起，主站 0 每 2000 周期有一个帧被位置 0 的从站拒绝：
 *                              各从站处理单元错误加 1，下游端口 0、上游端口 1 转发错误加 1
 *   ARMS_MOCK_TEMP=<C>         全部从站执行器温度（默认 35 + 链上位置）
 *   ARMS_MOCK_DRIVE_LOSS=<n>   第 n 周期起，主站 0 各从站自报的 EtherCAT 丢帧次数每 500 周期加 1
 *   ARMS_MOCK_EMCY=<n>         第 n 周期，主站 0 链上位置 0 的从站发出一条紧急报文（错误码 0x7500）
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "bus.h"
#include "arms/pn.h"

struct bus {
    /* 模拟设备的对象存储，读写权限由模拟对象字典决定 */
    uint16_t params[BUS_MAX_SLAVES][200], extra_pn[BUS_MAX_SLAVES];
    uint8_t test_record[BUS_MAX_SLAVES][2][8];
    bool pn075_unreadable, stuck_enabled;
    uint32_t revision;
    int slaves;
    int configured;
    float cycle_hz;
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
    /* 诊断模拟：ESC 错误计数、监控量和紧急报文 */
    uint8_t esc[BUS_MAX_SLAVES][ESC_ERR_BYTES];
    uint64_t esc_err_cycle, esc_proc_cycle, esc_done[BUS_MAX_SLAVES], mon_done[BUS_MAX_SLAVES];
    bool esc_reading[BUS_MAX_SLAVES], mon_reading[BUS_MAX_SLAVES];
    int temp_c;
    uint64_t loss_cycle, emcy_cycle;
    uint32_t drive_loss[BUS_MAX_SLAVES];
    int emcy_pending;
};

#define MOCK_REQ_CYCLES 3u /* 模拟异步请求的处理时间 */

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
    s = getenv("ARMS_MOCK_STUCK_ENABLED");
    b->stuck_enabled = s && !strcmp(s, "1");
    s = getenv("ARMS_MOCK_PN075_UNREADABLE");
    b->pn075_unreadable = s && !strcmp(s, "1");
    b->cycle_hz = 1e9f / (float)p->cycle_ns;
    b->slaves = 7;
    s = getenv("ARMS_MOCK_REVISION");
    b->revision = s ? (uint32_t)strtoul(s, NULL, 0) : KGU_PN_REVISION;
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
    s = getenv("ARMS_MOCK_ESC_ERR");
    b->esc_err_cycle = s ? strtoull(s, NULL, 10) : 0;
    s = getenv("ARMS_MOCK_ESC_PROC");
    b->esc_proc_cycle = s ? strtoull(s, NULL, 10) : 0;
    s = getenv("ARMS_MOCK_TEMP");
    b->temp_c = s ? atoi(s) : -1;
    s = getenv("ARMS_MOCK_DRIVE_LOSS");
    b->loss_cycle = s ? strtoull(s, NULL, 10) : 0;
    s = getenv("ARMS_MOCK_EMCY");
    b->emcy_cycle = s ? strtoull(s, NULL, 10) : 0;
    b->rng = 0x12345678u;
    b->master = p->master;
    if (b->master < 0) {
        size_t n = p->interface ? strlen(p->interface) : 0;
        b->master = (n > 0 && p->interface[n - 1] >= '0' && p->interface[n - 1] <= '9') ? p->interface[n - 1] - '0' : 0;
    }
    *out = b;
    return 0;
}

static int mock_scan(bus_t *b, bus_slave_info *infos, int max, int expected, int *count, unsigned int timeout_ms,
                     char *err, size_t err_size)
{
    int i;
    (void)expected;
    (void)timeout_ms;
    (void)err;
    (void)err_size;
    for (i = 0; i < b->slaves && i < max; ++i) {
        infos[i].vendor_id = KGU_VENDOR_ID;
        infos[i].product_code = KGU_PRODUCT_CODE;
        infos[i].revision = b->revision;
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
    if (b->stuck_enabled && st == 0x0027u && cw == KGU_CW_DISABLE) return b->sw[i];
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
    if (b->master == 0) {
        if (b->esc_err_cycle && b->cycles >= b->esc_err_cycle && (b->cycles - b->esc_err_cycle) % 200u == 0) {
            for (i = 0; i < b->configured; ++i) {
                if (b->positions[i] != 3) continue;
                if (b->esc[i][2] < 0xFF) b->esc[i][2]++; /* 端口 1 无效帧 */
                if (b->esc[i][3] < 0xFF) b->esc[i][3]++; /* 端口 1 物理层错误 */
            }
        }
        if (b->esc_proc_cycle && b->cycles >= b->esc_proc_cycle && (b->cycles - b->esc_proc_cycle) % 2000u == 0) {
            for (i = 0; i < b->configured; ++i) {
                uint8_t *e = b->esc[i];
                if (e[12] < 0xFF) e[12]++;                                       /* 处理单元错误 */
                if (b->positions[i] > 0 && e[8] < 0xFF) e[8]++;                  /* 端口 0 转发错误 */
                if (b->positions[i] < b->slaves - 1 && e[9] < 0xFF) e[9]++;      /* 端口 1 转发错误 */
            }
        }
        if (b->loss_cycle && b->cycles >= b->loss_cycle && (b->cycles - b->loss_cycle) % 500u == 0)
            for (i = 0; i < b->configured; ++i) b->drive_loss[i]++;
        if (b->emcy_cycle && b->cycles == b->emcy_cycle) b->emcy_pending = 1;
    }
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
        f.vel = (b->pos[i] - prev) * b->cycle_hz;
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

/* 0x3000 的值：Pn002 选择的监控量。slave 为链上位置（阻塞 SDO 使用链上位置寻址）。 */
static int32_t mock_monitor_value(const bus_t *b, int slave)
{
    int i;
    switch (b->params[slave][2]) {
    case KGU_MON_TEMPERATURE: return b->temp_c >= 0 ? b->temp_c : 35 + slave;
    case KGU_MON_ECAT_LOSS:
        for (i = 0; i < b->configured; ++i)
            if (b->positions[i] == slave) return (int32_t)b->drive_loss[i];
        return 0;
    default: return 0;
    }
}

static int mock_req_state(const bus_t *b, uint64_t done)
{
    if (!done) return BUS_REQ_IDLE;
    return b->cycles >= done ? BUS_REQ_DONE : BUS_REQ_BUSY;
}

static int mock_esc_read(bus_t *b, int i)
{
    if (mock_req_state(b, b->esc_done[i]) == BUS_REQ_BUSY) return -1;
    b->esc_reading[i] = true;
    b->esc_done[i] = b->cycles + MOCK_REQ_CYCLES;
    return 0;
}

static int mock_esc_clear(bus_t *b, int i)
{
    if (mock_req_state(b, b->esc_done[i]) == BUS_REQ_BUSY) return -1;
    memset(b->esc[i], 0, ESC_ERR_BYTES);
    b->esc_reading[i] = false;
    b->esc_done[i] = b->cycles + MOCK_REQ_CYCLES;
    return 0;
}

static int mock_esc_state(bus_t *b, int i, uint8_t *out)
{
    int s = mock_req_state(b, b->esc_done[i]);
    if (s == BUS_REQ_DONE && b->esc_reading[i] && out) memcpy(out, b->esc[i], ESC_ERR_BYTES);
    return s;
}

static int mock_monitor_select(bus_t *b, int i, uint16_t code)
{
    if (mock_req_state(b, b->mon_done[i]) == BUS_REQ_BUSY) return -1;
    b->params[b->positions[i]][2] = code;
    b->mon_reading[i] = false;
    b->mon_done[i] = b->cycles + MOCK_REQ_CYCLES;
    return 0;
}

static int mock_monitor_read(bus_t *b, int i)
{
    if (mock_req_state(b, b->mon_done[i]) == BUS_REQ_BUSY) return -1;
    b->mon_reading[i] = true;
    b->mon_done[i] = b->cycles + MOCK_REQ_CYCLES;
    return 0;
}

static int mock_monitor_state(bus_t *b, int i, int32_t *value)
{
    int s = mock_req_state(b, b->mon_done[i]);
    if (s == BUS_REQ_DONE && b->mon_reading[i] && value) *value = mock_monitor_value(b, b->positions[i]);
    return s;
}

static int mock_emerg_pop(bus_t *b, int i, uint8_t *msg)
{
    static const uint8_t emcy[8] = {0x00, 0x75, 0x10, 0x01, 0x02, 0x03, 0x04, 0x05};
    if (!b->emcy_pending || b->master != 0 || b->positions[i] != 0) return 0;
    memcpy(msg, emcy, sizeof(emcy));
    b->emcy_pending = 0;
    return 1;
}

static int mock_sdo(bus_t *b, int slave, uint16_t index, uint8_t sub, uint8_t *data, size_t *n, bool write,
                    uint32_t *abort)
{
    if (slave < 0 || slave >= BUS_MAX_SLAVES)
        return -1;
    if (index == 0x1018 && sub >= 1 && sub <= 4) {
        if (write) { *abort = 0x06010002; return -1; } /* 只读 */
        if (*n < 4) { *abort = 0x06070010; return -1; }
        uint32_t x = sub == 1 ? KGU_VENDOR_ID : sub == 2 ? KGU_PRODUCT_CODE :
                     sub == 3 ? b->revision : (uint32_t)(1 + slave + 16 * b->master);
        for (unsigned k = 0; k < 4; k++) data[k] = (uint8_t)(x >> (8 * k));
        *n = 4;
        return 0;
    }
    /* 模拟专用的非 Pn 对象：子索引 1 为 32 位读写，2 为 8 位只写，3 为 64 位读写 */
    if (index == 0x5ffe) {
        if (sub < 1 || sub > 3) { *abort = 0x06090011; return -1; }
        if (sub == 2 && !write) { *abort = 0x06010001; return -1; }
        size_t width = sub == 1 ? 4 : sub == 2 ? 1 : 8;
        if (write ? *n != width : *n < width) { *abort = 0x06070010; return -1; }
        if (sub != 2) {
            uint8_t *slot = b->test_record[slave][sub == 1 ? 0 : 1];
            if (write) memcpy(slot, data, width); else memcpy(data, slot, width);
        }
        *n = width; return 0;
    }
    if (index == KGU_OBJ_MONITOR_VALUE && sub == 0) {
        int32_t v;
        if (write) { *abort = 0x06010002; return -1; }
        if (*n < 4) { *abort = 0x06070010; return -1; }
        v = mock_monitor_value(b, slave);
        for (unsigned k = 0; k < 4; k++) data[k] = (uint8_t)((uint32_t)v >> (8 * k));
        *n = 4;
        return 0;
    }
    int number = arms_pn_from_index(index);
    if (number < 0 || (number >= 200 && number != 345)) { *abort = 0x06020000; return -1; }
    if (sub) { *abort = 0x06090011; return -1; }
    if (write ? *n != 2 : *n < 2) { *abort = 0x06070010; return -1; }
    if (!write && number == 75 && b->pn075_unreadable) { *abort = 0x08000000; return -1; }
    uint16_t *v = number == 345 ? &b->extra_pn[slave] : &b->params[slave][number];
    if (write) {
        uint16_t val = kgu_get_u16(data, 0);
        /* 由模拟设备返回 SDO abort */
        if (number == 26 && val > 24) { *abort = 0x06090030; return -1; }
        *v = val;
    } else kgu_put_u16(data, 0, *v);
    *n = 2;
    return 0;
}

const bus_ops bus_mock_ops = {
    "mock",          mock_open,          mock_scan,          mock_configure,      mock_activate,
    mock_receive,    mock_send,          mock_inputs,        mock_outputs,        mock_update_states,
    mock_expected_wkc, mock_master_index, mock_close,        mock_sdo,            mock_esc_read,
    mock_esc_clear,  mock_esc_state,     mock_monitor_select, mock_monitor_read,  mock_monitor_state,
    mock_emerg_pop,
};
