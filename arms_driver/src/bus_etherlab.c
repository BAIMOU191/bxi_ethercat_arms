/**
 * @file bus_etherlab.c
 * @brief EtherLab 总线后端
 *
 * 每条臂对应一个主站。所有配置的从站使用 KGU MIT PDO 映射并启用 DC Sync0，
 * 链上位置最小的从站作为 DC 参考时钟。
 */
#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <unistd.h>

#include <ecrt.h>

#include "bus.h"

static const ec_pdo_entry_info_t kgu_rx_entries[] = {
    {0x6040, 0, 16}, {0x4010, 0, 32}, {0x4011, 0, 32}, {0x4012, 0, 32},
    {0x4013, 0, 32}, {0x4014, 0, 32}, {0x6060, 0, 8},
};
static const ec_pdo_entry_info_t kgu_tx_entries[] = {
    {0x6041, 0, 16}, {0x4000, 0, 32}, {0x4001, 0, 32}, {0x4002, 0, 32}, {0x603f, 0, 16},
};
static const ec_pdo_info_t kgu_pdos[] = {
    {0x1601, 7, (ec_pdo_entry_info_t *)kgu_rx_entries},
    {0x1a01, 5, (ec_pdo_entry_info_t *)kgu_tx_entries},
};
static const ec_sync_info_t kgu_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, (ec_pdo_info_t *)&kgu_pdos[0], EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, (ec_pdo_info_t *)&kgu_pdos[1], EC_WD_DISABLE},
    {0xff, EC_DIR_INVALID, 0, NULL, EC_WD_DEFAULT},
};

struct bus {
    ec_master_t *master;
    ec_domain_t *domain;
    uint8_t *pd;
    int master_index;
    char interface[32];
    uint32_t cycle_ns;
    int32_t shift_ns;
    uint32_t vendor_id, product_code;
    int count;
    ec_slave_config_t *sc[BUS_MAX_SLAVES];
    unsigned int out_off[BUS_MAX_SLAVES];
    unsigned int in_off[BUS_MAX_SLAVES];
    ec_domain_state_t domain_state;
    /* 异步请求：ESC 错误计数寄存器、监控量选择（Pn002）和读取（0x3000） */
    ec_reg_request_t *esc_req[BUS_MAX_SLAVES];
    bool esc_reading[BUS_MAX_SLAVES];
    ec_sdo_request_t *mon_sel[BUS_MAX_SLAVES], *mon_val[BUS_MAX_SLAVES];
    bool mon_reading[BUS_MAX_SLAVES];
};

#define MONITOR_SDO_TIMEOUT_MS 500u
#define EMERGENCY_RING 8u

static uint64_t mono_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static bool read_line(const char *path, char *buf, size_t size)
{
    FILE *f = fopen(path, "r");
    size_t n;
    if (!f) return false;
    if (!fgets(buf, (int)size, f)) {
        fclose(f);
        return false;
    }
    fclose(f);
    n = strlen(buf);
    while (n > 0 && isspace((unsigned char)buf[n - 1])) buf[--n] = '\0';
    return true;
}

/* 主站号 = 网口 MAC 在 ec_master main_devices 中的序号 */
static int resolve_master(const bus_open_params *p, char *err, size_t err_size)
{
    char path[128], mac[32], devices[512];
    char *save = NULL, *tok;
    int index = 0, found = -1;

    snprintf(path, sizeof(path), "/sys/class/net/%s/address", p->interface);
    if (!read_line(path, mac, sizeof(mac))) {
        snprintf(err, err_size, "网口 %s 不存在（读取 %s 失败）", p->interface, path);
        return -1;
    }
    if (p->mac && p->mac[0] && strcasecmp(p->mac, mac) != 0) {
        snprintf(err, err_size, "网口 %s 的 MAC 是 %s，与配置的 %s 不一致（左右臂可能接反或配置写错）",
                 p->interface, mac, p->mac);
        return -1;
    }
    if (!read_line("/sys/module/ec_master/parameters/main_devices", devices, sizeof(devices))) {
        snprintf(err, err_size, "EtherLab 主站模块未加载（/sys/module/ec_master 不存在）");
        return -1;
    }
    for (tok = strtok_r(devices, ", \t", &save); tok; tok = strtok_r(NULL, ", \t", &save), ++index)
        if (strcasecmp(tok, mac) == 0) found = index;
    if (found < 0) {
        snprintf(err, err_size, "网口 %s (MAC %s) 没有绑定到任何 EtherLab 主站；检查 /etc/sysconfig/ethercat",
                 p->interface, mac);
        return -1;
    }
    if (p->master >= 0 && p->master != found) {
        snprintf(err, err_size, "配置要求 %s 对应主站 %d，但实际绑定的是主站 %d", p->interface, p->master, found);
        return -1;
    }
    return found;
}

static int el_open(bus_t **out, const bus_open_params *p, char *err, size_t err_size)
{
    bus_t *b;
    int index = resolve_master(p, err, err_size);
    if (index < 0) return -1;
    b = calloc(1, sizeof(*b));
    if (!b) {
        snprintf(err, err_size, "内存不足");
        return -1;
    }
    /*
     * 只打开、不占用：主站保持空闲阶段，后台线程持续扫描，上电后陆续启动的从站才能被发现。
     * 占用（ecrt_master_reserve）推迟到 scan 确认总线稳定之后。
     */
    b->master = ecrt_open_master((unsigned int)index);
    if (!b->master) {
        snprintf(err, err_size, "无法打开主站 %d（%s）：EtherLab 未启动或权限不足（需要 sudo）", index, p->interface);
        free(b);
        return -1;
    }
    b->master_index = index;
    snprintf(b->interface, sizeof(b->interface), "%s", p->interface);
    b->cycle_ns = p->cycle_ns;
    b->shift_ns = p->sync0_shift_ns;
    b->vendor_id = p->vendor_id;
    b->product_code = p->product_code;
    *out = b;
    return 0;
}

/* 总线稳定后占用主站；占用后主站进入运行阶段，不再自动扫描 */
static int reserve(bus_t *b, char *err, size_t err_size)
{
    if (ecrt_master_reserve(b->master) != 0) {
        snprintf(err, err_size,
                 "无法占用主站 %d（%s）：可能已被 motor_console、hw_lib_test_* 或 ROS 节点占用",
                 b->master_index, b->interface);
        return BUS_SCAN_MASTER;
    }
    return 0;
}

static int el_scan(bus_t *b, bus_slave_info *infos, int max, int expected, int *count, unsigned int timeout_ms,
                   char *err, size_t err_size)
{
    uint64_t deadline = mono_ns() + (uint64_t)timeout_ms * 1000000ull;
    uint64_t stable_since = 0;
    int last = -1;
    ec_master_info_t mi;
    bool have_info;
    unsigned int i;
    int bad = -1;

    do {
        uint64_t now = mono_ns();
        memset(&mi, 0, sizeof(mi));
        have_info = ecrt_master(b->master, &mi) == 0;
        if (have_info && mi.link_up && mi.slave_count > 0 && mi.scan_busy == 0) {
            bool ready = true;
            int n = (int)(mi.slave_count < (unsigned int)max ? mi.slave_count : (unsigned int)max);
            for (i = 0; (int)i < n; ++i) {
                ec_slave_info_t si;
                if (ecrt_master_get_slave(b->master, (uint16_t)i, &si) != 0 || si.vendor_id == 0 ||
                    si.product_code == 0) {
                    ready = false;
                    bad = (int)i;
                    break;
                }
                infos[i].vendor_id = si.vendor_id;
                infos[i].product_code = si.product_code;
                infos[i].revision = si.revision_number;
                snprintf(infos[i].name, sizeof(infos[i].name), "%s", si.name);
            }
            if (ready) {
                /* 上电后驱动器陆续启动，从站数会逐步增加；数量达到预期并保持不变才算稳定 */
                if (n != last) {
                    last = n;
                    stable_since = now;
                }
                if (n >= expected && now - stable_since >= (uint64_t)BUS_SETTLE_MS * 1000000ull) {
                    *count = n;
                    return reserve(b, err, err_size);
                }
            } else {
                last = -1;
            }
        } else {
            last = -1;
        }
        usleep(20000);
    } while (mono_ns() < deadline);
    if (last > 0 && mono_ns() - stable_since >= (uint64_t)BUS_SETTLE_MS * 1000000ull) {
        /* 超时仍不足预期：返回当前结果，由调用方的启动检查给出缺失的关节 */
        *count = last;
        return reserve(b, err, err_size);
    }
    snprintf(err, err_size, "%s 在 %u ms 内未完成稳定扫描：从站数=%u 链路=%s 扫描中=%u 首个未就绪位置=%d",
             b->interface, timeout_ms, mi.slave_count, !have_info ? "UNKNOWN" : mi.link_up ? "UP" : "DOWN",
             mi.scan_busy, bad);
    if (!have_info) return BUS_SCAN_MASTER;
    if (!mi.link_up) return BUS_SCAN_LINK;
    return !mi.scan_busy && !mi.slave_count ? BUS_SCAN_EMPTY : BUS_SCAN_UNSTABLE;
}

static int el_configure(bus_t *b, const int *positions, int n, char *err, size_t err_size)
{
    int i, ref = -1, ret;
    if (n <= 0 || n > BUS_MAX_SLAVES) {
        snprintf(err, err_size, "配置的从站数 %d 无效", n);
        return -1;
    }
    b->domain = ecrt_master_create_domain(b->master);
    if (!b->domain) {
        snprintf(err, err_size, "创建 domain 失败");
        return -1;
    }
    for (i = 0; i < n; ++i) {
        int off_out, off_in;
        b->sc[i] = ecrt_master_slave_config(b->master, 0, (uint16_t)positions[i], b->vendor_id, b->product_code);
        if (!b->sc[i]) {
            snprintf(err, err_size, "slave %d 配置失败（厂商/产品号不匹配？）", positions[i]);
            return -1;
        }
        ret = ecrt_slave_config_pdos(b->sc[i], EC_END, kgu_syncs);
        if (ret != 0) {
            snprintf(err, err_size, "slave %d PDO 映射失败: %s", positions[i], strerror(-ret));
            return -1;
        }
        off_out = ecrt_slave_config_reg_pdo_entry(b->sc[i], 0x6040, 0, b->domain, NULL);
        off_in = ecrt_slave_config_reg_pdo_entry(b->sc[i], 0x6041, 0, b->domain, NULL);
        if (off_out < 0 || off_in < 0) {
            snprintf(err, err_size, "slave %d PDO 条目注册失败", positions[i]);
            return -1;
        }
        b->out_off[i] = (unsigned int)off_out;
        b->in_off[i] = (unsigned int)off_in;
        ret = ecrt_slave_config_dc(b->sc[i], 0x0300, b->cycle_ns, b->shift_ns, 0, 0);
        if (ret != 0) {
            snprintf(err, err_size, "slave %d DC Sync0 配置失败: %s", positions[i], strerror(-ret));
            return -1;
        }
        /* 诊断用的异步请求和紧急报文缓冲；创建失败时仅停用诊断 */
        b->esc_req[i] = ecrt_slave_config_create_reg_request(b->sc[i], ESC_ERR_BYTES);
        b->mon_sel[i] = ecrt_slave_config_create_sdo_request(b->sc[i], KGU_OBJ_MONITOR_SELECT, 0, 2);
        b->mon_val[i] = ecrt_slave_config_create_sdo_request(b->sc[i], KGU_OBJ_MONITOR_VALUE, 0, 4);
        if (b->mon_sel[i]) ecrt_sdo_request_timeout(b->mon_sel[i], MONITOR_SDO_TIMEOUT_MS);
        if (b->mon_val[i]) ecrt_sdo_request_timeout(b->mon_val[i], MONITOR_SDO_TIMEOUT_MS);
        ecrt_slave_config_emerg_size(b->sc[i], EMERGENCY_RING);
        if (ref < 0 || positions[i] < positions[ref]) ref = i;
    }
    ret = ecrt_master_select_reference_clock(b->master, b->sc[ref]);
    if (ret != 0) {
        snprintf(err, err_size, "选择 DC 参考时钟失败: %s", strerror(-ret));
        return -1;
    }
    b->count = n;
    return 0;
}

static int el_activate(bus_t *b, char *err, size_t err_size)
{
    int ret;
    ecrt_master_application_time(b->master, mono_ns());
    ret = ecrt_master_activate(b->master);
    if (ret != 0) {
        snprintf(err, err_size, "主站激活失败: %s", strerror(-ret));
        return -1;
    }
    b->pd = ecrt_domain_data(b->domain);
    if (!b->pd) {
        snprintf(err, err_size, "获取过程数据区失败");
        return -1;
    }
    return 0;
}

static int el_receive(bus_t *b)
{
    if (ecrt_master_receive(b->master) < 0) return -1;
    ecrt_domain_process(b->domain);
    ecrt_domain_state(b->domain, &b->domain_state);
    return (int)b->domain_state.working_counter;
}

static int el_send(bus_t *b, uint64_t app_time_ns)
{
    ecrt_master_application_time(b->master, app_time_ns);
    ecrt_master_sync_reference_clock(b->master);
    ecrt_master_sync_slave_clocks(b->master);
    ecrt_domain_queue(b->domain);
    return ecrt_master_send(b->master) < 0 ? -1 : 0;
}

static const uint8_t *el_inputs(bus_t *b, int i) { return b->pd + b->in_off[i]; }
static uint8_t *el_outputs(bus_t *b, int i) { return b->pd + b->out_off[i]; }

static void el_update_states(bus_t *b, uint8_t *al, bool *link_up)
{
    ec_master_state_t ms;
    int i;
    for (i = 0; i < b->count; ++i) {
        ec_slave_config_state_t st;
        memset(&st, 0, sizeof(st));
        if (ecrt_slave_config_state(b->sc[i], &st) == 0)
            al[i] = st.online ? (uint8_t)st.al_state : 0;
    }
    memset(&ms, 0, sizeof(ms));
    ecrt_master_state(b->master, &ms);
    *link_up = ms.link_up != 0;
}

static uint32_t el_expected_wkc(bus_t *b) { return (uint32_t)(3 * b->count); }
static int el_master_index(bus_t *b) { return b->master_index; }

static void el_close(bus_t *b)
{
    if (!b) return;
    if (b->master) ecrt_release_master(b->master);
    free(b);
}

static int el_sdo(bus_t *b, int slave, uint16_t index, uint8_t sub, uint8_t *data, size_t *n, bool write,
                  uint32_t *abort)
{
    if (write)
        return ecrt_master_sdo_download(b->master, slave, index, sub, data, *n, abort);
    size_t cap = *n;
    return ecrt_master_sdo_upload(b->master, slave, index, sub, data, cap, n, abort);
}

static int req_state(ec_request_state_t s)
{
    switch (s) {
    case EC_REQUEST_BUSY: return BUS_REQ_BUSY;
    case EC_REQUEST_SUCCESS: return BUS_REQ_DONE;
    case EC_REQUEST_ERROR: return BUS_REQ_ERROR;
    default: return BUS_REQ_IDLE;
    }
}

static int el_esc_read(bus_t *b, int i)
{
    if (!b->esc_req[i] || ecrt_reg_request_state(b->esc_req[i]) == EC_REQUEST_BUSY) return -1;
    b->esc_reading[i] = true;
    return ecrt_reg_request_read(b->esc_req[i], ESC_ERR_ADDR, ESC_ERR_BYTES);
}

static int el_esc_clear(bus_t *b, int i)
{
    if (!b->esc_req[i] || ecrt_reg_request_state(b->esc_req[i]) == EC_REQUEST_BUSY) return -1;
    memset(ecrt_reg_request_data(b->esc_req[i]), 0, ESC_ERR_BYTES);
    b->esc_reading[i] = false;
    return ecrt_reg_request_write(b->esc_req[i], ESC_ERR_ADDR, ESC_ERR_BYTES);
}

static int el_esc_state(bus_t *b, int i, uint8_t *out)
{
    int s;
    if (!b->esc_req[i]) return BUS_REQ_ERROR;
    s = req_state(ecrt_reg_request_state(b->esc_req[i]));
    if (s == BUS_REQ_DONE && b->esc_reading[i] && out) memcpy(out, ecrt_reg_request_data(b->esc_req[i]), ESC_ERR_BYTES);
    return s;
}

static int el_monitor_select(bus_t *b, int i, uint16_t code)
{
    if (!b->mon_sel[i] || ecrt_sdo_request_state(b->mon_sel[i]) == EC_REQUEST_BUSY) return -1;
    kgu_put_u16(ecrt_sdo_request_data(b->mon_sel[i]), 0, code);
    b->mon_reading[i] = false;
    return ecrt_sdo_request_write(b->mon_sel[i]);
}

static int el_monitor_read(bus_t *b, int i)
{
    if (!b->mon_val[i] || ecrt_sdo_request_state(b->mon_val[i]) == EC_REQUEST_BUSY) return -1;
    b->mon_reading[i] = true;
    return ecrt_sdo_request_read(b->mon_val[i]);
}

static int el_monitor_state(bus_t *b, int i, int32_t *value)
{
    ec_sdo_request_t *r = b->mon_reading[i] ? b->mon_val[i] : b->mon_sel[i];
    int s;
    if (!r) return BUS_REQ_ERROR;
    s = req_state(ecrt_sdo_request_state(r));
    if (s == BUS_REQ_DONE && b->mon_reading[i] && value) {
        const uint8_t *d = ecrt_sdo_request_data(r);
        if (ecrt_sdo_request_data_size(r) < 4) return BUS_REQ_ERROR;
        *value = (int32_t)((uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24));
    }
    return s;
}

static int el_emerg_pop(bus_t *b, int i, uint8_t *msg)
{
    int ret = ecrt_slave_config_emerg_pop(b->sc[i], msg);
    if (ret == 0) return 1;
    return ret == -ENOENT ? 0 : ret;
}

const bus_ops bus_etherlab_ops = {
    "etherlab",       el_open,         el_scan,          el_configure,     el_activate,        el_receive,
    el_send,          el_inputs,       el_outputs,       el_update_states, el_expected_wkc,    el_master_index,
    el_close,         el_sdo,          el_esc_read,      el_esc_clear,     el_esc_state,       el_monitor_select,
    el_monitor_read,  el_monitor_state, el_emerg_pop,
};
