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
};

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
        snprintf(err, err_size, "EtherLab 主站模块未加载（/sys/module/ec_master 不存在）；请先启动 ethercat 服务");
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
    b->master = ecrt_request_master((unsigned int)index);
    if (!b->master) {
        snprintf(err, err_size,
                 "无法获取主站 %d（%s）：可能已被 motor_console、hw_lib_test_* 或 ROS 节点占用，或权限不足（需要 sudo）",
                 index, p->interface);
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

static int el_scan(bus_t *b, bus_slave_info *infos, int max, int expected, int *count, unsigned int timeout_ms,
                   char *err, size_t err_size)
{
    uint64_t deadline = mono_ns() + (uint64_t)timeout_ms * 1000000ull;
    uint64_t stable_since = 0;
    int last = -1;
    ec_master_info_t mi;
    ec_master_state_t ms;
    unsigned int i;
    int bad = -1;

    do {
        uint64_t now = mono_ns();
        memset(&mi, 0, sizeof(mi));
        if (ecrt_master(b->master, &mi) == 0 && mi.slave_count > 0 && mi.scan_busy == 0) {
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
                    return 0;
                }
            } else {
                last = -1;
            }
        } else {
            last = -1;
        }
        usleep(20000);
    } while (mono_ns() < deadline);
    if (last > 0) {
        /* 超时仍不足预期：返回当前结果，由调用方的启动检查给出缺失的关节 */
        *count = last;
        return 0;
    }
    memset(&ms, 0, sizeof(ms));
    ecrt_master_state(b->master, &ms);
    snprintf(err, err_size, "%s 在 %u ms 内未完成扫描：从站数=%u 应答=%u 链路=%s 扫描中=%u 首个未就绪位置=%d",
             b->interface, timeout_ms, mi.slave_count, ms.slaves_responding, ms.link_up ? "UP" : "DOWN",
             mi.scan_busy, bad);
    return -1;
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

const bus_ops bus_etherlab_ops = {
    "etherlab", el_open,    el_scan,          el_configure,    el_activate,     el_receive, el_send,
    el_inputs,  el_outputs, el_update_states, el_expected_wkc, el_master_index, el_close,   el_sdo,
};
