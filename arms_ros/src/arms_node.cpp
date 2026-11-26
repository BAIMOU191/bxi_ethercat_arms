#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <communication/msg/actuator_cmds.hpp>
#include <communication/msg/actuator_states.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <arms_ros/srv/joint_operation.hpp>
#include <arms_ros/srv/read_param.hpp>
#include "arms/arms.h"
#include "arms/lease.h"
#include "arms/pn.h"
extern "C" {
#include "power.h"
#include "runtime_env.h"
}
using namespace std::chrono_literals;
using Cmd = communication::msg::ActuatorCmds;
using State = communication::msg::ActuatorStates;
using Joint = sensor_msgs::msg::JointState;
using Diag = diagnostic_msgs::msg::DiagnosticArray;
using Op = arms_ros::srv::JointOperation;
using Trigger = std_srvs::srv::Trigger;
using ReadParam = arms_ros::srv::ReadParam;

static std::string format(const char *fmt, ...)
{
    char line[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    return line;
}

// 节点在启动检查中失败、尚未完整构造时同样执行
struct Device {
    arms_t *driver = nullptr;
    int lease = -1;
    std::atomic<bool> owns_power{false}, stop{false}, cut{false};
    std::thread watchdog;
    ~Device()
    {
        stop = true;
        if (watchdog.joinable())
            watchdog.join();
        if (driver) {
            arms_disable(driver, arms_opened_joints(driver));
            std::this_thread::sleep_for(50ms);
            arms_log_events(driver);
            arms_close(driver);
        }
        if (owns_power.exchange(false))
            arms_power_off();
        if (lease >= 0) {
            arms_env_rt_revert();
            close(lease);
        }
    }
};
class ArmsNode final : public rclcpp::Node
{
    Device dev;
    std::string config, event_log;
    rclcpp::Publisher<Joint>::SharedPtr joint_pub;
    rclcpp::Publisher<State>::SharedPtr state_pub;
    rclcpp::Publisher<Diag>::SharedPtr diag_pub;
    rclcpp::Subscription<Cmd>::SharedPtr cmd_sub;
    std::vector<rclcpp::ServiceBase::SharedPtr> services;
    rclcpp::TimerBase::SharedPtr tick, report;
    int64_t max_age_ms;
    unsigned rejected = 0;
    bool manage_power = false;
    /* 处于 hold 的关节：忽略话题目标，直到执行 enable、free、disable 或 reset；仅在执行器线程访问 */
    arms_mask held = 0;
    void reject(const char *why)
    {
        ++rejected;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "command rejected: %s", why);
    }
    void receive(const Cmd &m)
    {
        if (dev.cut)
            return;
        const auto n = m.actuators_name.size();
        if (!n || n > 31 || m.pos.size() != n || m.vel.size() != n || m.kp.size() != n || m.kd.size() != n ||
            m.torque.size() != n) {
            reject("array lengths");
            return;
        }
        if (m.header.stamp.sec < 0 || m.header.stamp.nanosec >= 1000000000u) {
            reject("invalid timestamp");
            return;
        }
        const int64_t age = now().nanoseconds() - (static_cast<int64_t>(m.header.stamp.sec) * 1000000000LL +
                                                   m.header.stamp.nanosec);
        if (age < -5000000LL || age > max_age_ms * 1000000LL) {
            reject("timestamp");
            return;
        }
        std::array<arms_command, 29> cmds{};
        arms_mask mask = 0;
        for (size_t i = 0; i < n; i++) {
            int j = arms_joint_by_name(dev.driver, m.actuators_name[i].c_str());
            if (j < 15 || j > 28)
                continue; // full-body frames also carry leg and trunk commands
            arms_mask bit = ARMS_MASK_JOINT(j);
            if (!(arms_opened_joints(dev.driver) & bit))
                continue;
            arms_joint_state s{};
            arms_read_joint(dev.driver, j, &s);
            arms_command c{m.pos[i], m.vel[i], m.torque[i], m.kp[i], m.kd[i]};
            if (mask & bit || !std::isfinite(c.position) || !std::isfinite(c.velocity) ||
                !std::isfinite(c.torque)) {
                reject("duplicate or invalid value");
                return;
            }
            mask |= bit;
            cmds[j] = c;
        }
        for (int j = 15; j <= 28; j++)
            if ((mask & ~held) & ARMS_MASK_JOINT(j)) {
                arms_joint_state s{};
                arms_read_joint(dev.driver, j, &s);
                if (s.mode == ARMS_JS_ENABLED && arms_set_command(dev.driver, j, &cmds[j]) != ARMS_OK) {
                    arms_disable(dev.driver, mask);
                    reject("driver rejected command");
                    return;
                }
            }
    }
    bool healthy()
    {
        if (dev.cut || arms_check_online(dev.driver, nullptr) != ARMS_OK)
            return false;
        arms_bus_stats b{};
        arms_get_stats(dev.driver, &b);
        if (b.tripped || b.protect.reason != ARMS_PROTECT_NONE)
            return false;
        arms_joint_state s[14];
        size_t n = 0;
        arms_read(dev.driver, s, 14, &n);
        for (size_t i = 0; i < n; i++)
            if (s[i].mode == ARMS_JS_FAULT || s[i].error_code)
                return false;
        return n > 0;
    }
    void operation(const std::string &name, const Op::Request &q, Op::Response &r)
    {
        if (name == "selftest") {
            r.success = healthy();
            r.message = r.success ? "online; no drive fault" : "offline, drive fault or trip";
            return;
        }
        arms_mask m = q.joints;
        auto all = arms_opened_joints(dev.driver);
        if (!m || (m & ~all)) {
            r.message = "select opened joint bits 15..28";
            return;
        }
        if (name == "hold") {
            /* 停在当前指令位置：速度与前馈清零，增益不变；不需要确认 */
            arms_mask enabled = 0;
            for (int j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; j++) {
                arms_joint_state s{};
                if ((m & ARMS_MASK_JOINT(j)) && arms_read_joint(dev.driver, j, &s) == ARMS_OK &&
                    s.mode == ARMS_JS_ENABLED)
                    enabled |= ARMS_MASK_JOINT(j);
            }
            if (!enabled) {
                r.message = "no selected joint is enabled";
                return;
            }
            held |= enabled;
            r.success = arms_hold(dev.driver, enabled) == ARMS_OK;
            r.message = r.success ? format("holding joints 0x%08x; topic targets for them are ignored until "
                                           "enable, free, disable or reset",
                                           enabled)
                                  : std::string("hold failed: ") + arms_last_error();
            return;
        }
        if (name != "disable" && (!q.confirm || dev.cut)) {
            r.message = "confirmation required or protection latched";
            return;
        }
        arms_status st = ARMS_E_ARG;
        if (name == "enable") {
            st = arms_enable(dev.driver, m, 0);
        } else if (name == "disable")
            st = arms_disable(dev.driver, m);
        else if (name == "reset")
            st = arms_reset(dev.driver, m, 0);
        else if (name == "free")
            st = arms_free(dev.driver, m, 0.0);
        else {
            /* brake_* 与 zero：逐个关节执行，汇总失败原因 */
            std::string failed;
            for (int j = ARMS_JOINT_FIRST; j <= ARMS_JOINT_LAST; j++) {
                if (!(m & ARMS_MASK_JOINT(j)))
                    continue;
                arms_status one;
                if (name == "brake_normal")
                    one = arms_set_brake(dev.driver, j, ARMS_BRAKE_NORMAL, q.confirm);
                else if (name == "brake_release")
                    one = arms_set_brake(dev.driver, j, ARMS_BRAKE_RELEASE, q.confirm);
                else if (name == "brake_hold")
                    one = arms_set_brake(dev.driver, j, ARMS_BRAKE_HOLD, q.confirm);
                else
                    one = arms_service(dev.driver, j, q.soft_zero ? ARMS_ZERO_SOFT : ARMS_ZERO_HW, q.confirm);
                if (one != ARMS_OK)
                    failed += std::to_string(j) + ": " + arms_last_error() + "; ";
            }
            r.success = failed.empty();
            r.message = r.success ? "OK" : failed;
            return;
        }
        if (name == "disable" || st == ARMS_OK)
            held &= ~m;
        r.success = st == ARMS_OK;
        r.message = r.success ? "OK" : std::string(arms_strerror(st)) + ": " + arms_last_error();
    }
    /* 一次性可读状态：总线、保护、每个关节的模式和反馈 */
    void status(Trigger::Response &r)
    {
        static const char *protect_names[] = {"none", "overspeed", "overtorque", "overtemp"};
        arms_bus_stats b{};
        arms_get_stats(dev.driver, &b);
        std::string t;
        const char *protect = b.protect.reason <= ARMS_PROTECT_OVERTEMP ? protect_names[b.protect.reason] : "?";
        t += format("session %s, trip %s (joints 0x%08x), protect %s", dev.cut ? "LATCHED" : "ok",
                    b.tripped ? "LATCHED" : "none", b.trip_joints, protect);
        if (b.protect.reason != ARMS_PROTECT_NONE)
            t += format(" joint %d value %.3f limit %.3f", b.protect.joint, b.protect.value, b.protect.limit);
        t += format("\nignored 0x%08x  missing 0x%08x  held 0x%08x  zero_pending 0x%08x  rejected_commands %u\n",
                    arms_ignored_joints(dev.driver), arms_missing_joints(dev.driver), held,
                    arms_zero_pending(dev.driver), rejected);
        for (int i = 0; i < ARMS_ARM_COUNT; i++) {
            const auto &a = b.arm[i];
            if (!a.present)
                continue;
            t += format("%-5s %s link %s  OP %d/%d  WKC %u/%u  comm %s  drops %u\n", arms_link_name(dev.driver, i), a.interface,
                        a.link_up ? "up" : "DOWN", a.slaves_op, a.slaves_expected, a.wkc_last, a.wkc_expected,
                        a.comm_failed ? "FAILED" : "ok", a.comm_drops);
        }
        t += "joint name                 mode         pos(rad)  vel(rad/s) torque(Nm)  SW     ERR    cmd_pos  kp      "
             "kd     temp(C) drv_loss flags\n";
        arms_joint_state s[ARMS_JOINT_COUNT];
        size_t n = 0;
        arms_read(dev.driver, s, ARMS_JOINT_COUNT, &n);
        for (size_t i = 0; i < n; i++) {
            std::string flags;
            if (held & ARMS_MASK_JOINT(s[i].joint)) flags += 'H';
            if (s[i].cmd_pos_blocked) flags += 'P';
            if (s[i].cmd_timeout) flags += 'T';
            if (flags.empty()) flags = "-";
            const std::string temp = std::isnan(s[i].temperature) ? "-" : format("%.0f", s[i].temperature);
            const std::string loss = s[i].drive_frame_loss < 0 ? "-" : std::to_string(s[i].drive_frame_loss);
            t += format("%-5d %-20s %-10s %9.4f %10.4f %10.4f  0x%04x 0x%04x %8.4f %7.2f %6.2f %7s %8s %s\n",
                        s[i].joint, s[i].name ? s[i].name : "-", arms_joint_mode_name(s[i].mode), s[i].position,
                        s[i].velocity, s[i].torque, s[i].status_word, s[i].error_code, s[i].cmd_position, s[i].cmd_kp,
                        s[i].cmd_kd, temp.c_str(), loss.c_str(), flags.c_str());
        }
        t += "flags: H=hold (topic targets ignored), P=soft position limit blocking outward vel/torque, T=command timeout; drv_loss = drive-reported "
             "EtherCAT frame losses since power-up";
        r.success = healthy();
        r.message = t;
    }
    /* 等待两条臂通信恢复：链路正常、从站全部 OP、WKC 正确 */
    bool wait_online(unsigned timeout_ms)
    {
        for (unsigned waited = 0; waited <= timeout_ms; waited += 100) {
            arms_bus_stats b{};
            arms_get_stats(dev.driver, &b);
            bool online = true;
            for (int i = 0; i < ARMS_ARM_COUNT; i++) {
                const auto &a = b.arm[i];
                if (a.present && (a.comm_failed || a.slaves_op != a.slaves_expected || a.wkc_last != a.wkc_expected))
                    online = false;
            }
            if (online)
                return true;
            std::this_thread::sleep_for(100ms);
        }
        return false;
    }
    /*
     * 解除掉线保护锁定。节点管理电源时先重新上电；等待通信恢复后要求全部已打开关节在线，
     * 通过则清除锁定，关节保持失能。检查失败时，本次上的电会再次切断。
     */
    void clear_trip(Trigger::Response &r)
    {
        arms_bus_stats b{};
        arms_get_stats(dev.driver, &b);
        if (!b.tripped && !dev.cut) {
            r.success = true;
            r.message = "no protection trip latched";
            return;
        }
        bool powered_here = false;
        if (manage_power && !dev.owns_power) {
            if (arms_power_on(0)) {
                r.message = "power-on request failed";
                return;
            }
            dev.owns_power = true;
            powered_here = true;
        }
        std::string why;
        if (!wait_online(15000)) {
            arms_mask off = 0;
            why = arms_check_online(dev.driver, &off) != ARMS_OK ? arms_last_error()
                                                                 : "communication did not recover within 15 s";
        } else if (arms_check_online(dev.driver, nullptr) != ARMS_OK || arms_clear_trip(dev.driver) != ARMS_OK) {
            why = arms_last_error();
        }
        if (!why.empty()) {
            if (powered_here && dev.owns_power.exchange(false) && arms_power_off())
                why += "; power-off request FAILED, switch motor power off manually";
            else if (powered_here)
                why += "; motor power switched off again";
            r.message = "trip not cleared: " + why;
            return;
        }
        held = 0;
        dev.cut = false;
        r.success = true;
        r.message = "trip cleared; all opened joints online and disabled";
        RCLCPP_INFO(get_logger(), "protection trip cleared");
    }
    /* 只读：按 Pn 编号或对象字典地址读取单个关节的参数（SDO 上传），不修改驱动器 */
    void read_param(const ReadParam::Request &q, ReadParam::Response &r)
    {
        const int j = q.joint;
        if (j < ARMS_JOINT_FIRST || j > ARMS_JOINT_LAST || !(arms_opened_joints(dev.driver) & ARMS_MASK_JOINT(j))) {
            r.message = "joint must be an opened joint 15..28";
            return;
        }
        uint16_t index = q.index;
        uint8_t sub = q.subindex;
        const arms_pn_info *info = nullptr;
        if (q.pn >= 0) {
            const int idx = q.pn <= 999 ? arms_pn_index(static_cast<unsigned>(q.pn)) : -1;
            if (idx < 0) {
                r.message = "pn must be a decimal Pn number 0..999, or -1 to use index/subindex";
                return;
            }
            index = static_cast<uint16_t>(idx);
            sub = 0;
            info = arms_pn_lookup(static_cast<unsigned>(q.pn));
        }
        uint8_t data[64];
        size_t n = 0;
        if (arms_sdo_read(dev.driver, j, index, sub, data, sizeof(data), &n) != ARMS_OK) {
            r.message = arms_last_error();
            return;
        }
        r.raw.assign(data, data + n);
        int64_t value = 0;
        for (size_t i = 0; i < n && i < 8; i++)
            value |= static_cast<int64_t>(data[i]) << (8 * i);
        if (q.pn >= 0 && n == 2)
            value = static_cast<int16_t>(data[0] | (data[1] << 8)); /* Pn 为 INT16 */
        std::string bytes;
        for (size_t i = 0; i < n; i++)
            bytes += format(" %02x", data[i]);
        r.value = value;
        r.success = true;
        if (q.pn >= 0)
            r.message = format("joint %d Pn%03d (0x%04x:%02x%s%s) = %lld, raw%s", j, q.pn, index, sub,
                               info ? ", " : "", info ? info->name : "", static_cast<long long>(value), bytes.c_str());
        else
            r.message = format("joint %d 0x%04x:%02x = %lld, raw%s", j, index, sub, static_cast<long long>(value),
                               bytes.c_str());
    }
    void feedback()
    {
        arms_joint_state s[14];
        size_t n = 0;
        if (arms_read(dev.driver, s, 14, &n) != ARMS_OK)
            return;
        Joint j;
        State a;
        j.header.stamp = now();
        a.header = j.header;
        for (size_t i = 0; i < n; i++) {
            j.name.emplace_back(s[i].name);
            j.position.push_back(s[i].position);
            j.velocity.push_back(s[i].velocity);
            j.effort.push_back(s[i].torque);
        }
        a.name = j.name;
        a.position = j.position;
        a.velocity = j.velocity;
        a.effort = j.effort;
        /* 驱动器只提供执行器温度（Pn002=77），填入 driver_temperature；电机温度未知 */
        const double unknown = std::numeric_limits<double>::quiet_NaN();
        for (size_t i = 0; i < n; i++)
            a.driver_temperature.push_back(s[i].temperature);
        a.motor_temperature.assign(n, unknown);
        joint_pub->publish(j);
        state_pub->publish(a);
    }
    void diagnostics()
    {
        Diag msg;
        msg.header.stamp = now();
        arms_bus_stats b{};
        arms_get_stats(dev.driver, &b);
        arms_joint_state states[ARMS_JOINT_COUNT];
        size_t n = 0;
        arms_mask faults[ARMS_ARM_COUNT]{};
        double max_temp[ARMS_ARM_COUNT];
        int64_t drive_loss[ARMS_ARM_COUNT]{}, esc_line[ARMS_ARM_COUNT]{}, esc_proc[ARMS_ARM_COUNT]{};
        for (auto &t : max_temp)
            t = std::numeric_limits<double>::quiet_NaN();
        arms_read(dev.driver, states, ARMS_JOINT_COUNT, &n);
        for (size_t k = 0; k < n; ++k) {
            const auto &s = states[k];
            if (s.mode == ARMS_JS_FAULT || s.error_code) faults[s.link] |= ARMS_MASK_JOINT(s.joint);
            if (!std::isnan(s.temperature) && (std::isnan(max_temp[s.link]) || s.temperature > max_temp[s.link]))
                max_temp[s.link] = s.temperature;
            if (s.drive_frame_loss > 0)
                drive_loss[s.link] += s.drive_frame_loss;
            /* 线路类：无效帧、物理层错误、链路丢失；处理类：转发、处理单元、PDI 错误 */
            for (int e = 0; e < 20; ++e) {
                if (e == 14 || e == 15)
                    continue;
                if (e < 8 || e >= 16)
                    esc_line[s.link] += s.esc_errors[e];
                else
                    esc_proc[s.link] += s.esc_errors[e];
            }
        }
        for (int i = 0; i < ARMS_ARM_COUNT; i++) {
            auto &a = b.arm[i];
            if (!a.present)
                continue;
            diagnostic_msgs::msg::DiagnosticStatus d;
            d.name = std::string("arms/") + arms_link_name(dev.driver, i);
            d.hardware_id = a.interface;
            const bool prot = b.protect.reason != ARMS_PROTECT_NONE;
            d.level = (dev.cut || b.tripped || prot || faults[i] || a.slaves_op != a.slaves_expected || a.comm_failed || !a.link_up ||
                       a.wkc_last != a.wkc_expected)
                          ? 2
                          : 0;
            d.message = prot ? (b.protect.reason == ARMS_PROTECT_OVERSPEED    ? "overspeed protection latched"
                                : b.protect.reason == ARMS_PROTECT_OVERTORQUE ? "overtorque protection latched"
                                                                               : "overtemperature protection latched")
                        : faults[i] ? "drive fault"
                        : d.level ? "communication fault or trip"
                                  : "online";
            auto add = [&](const char *k, auto value) {
                diagnostic_msgs::msg::KeyValue kv;
                kv.key = k;
                kv.value = std::to_string(value);
                d.values.push_back(kv);
            };
            add("fault_joints", faults[i]);
            add("wkc", a.wkc_last);
            add("expected", a.wkc_expected);
            add("trip_joints", b.trip_joints);
            add("ignored", arms_ignored_joints(dev.driver));
            add("missing", arms_missing_joints(dev.driver));
            add("protect_joint", b.protect.joint);
            add("protect_value", b.protect.value);
            add("rejected_commands", rejected);
            add("held_joints", held);
            add("max_temp_c", std::isnan(max_temp[i]) ? -1.0 : max_temp[i]);
            add("drive_frame_loss", drive_loss[i]);
            add("esc_line_errors", esc_line[i]);
            add("esc_processing_errors", esc_proc[i]);
            msg.status.push_back(d);
        }
        diag_pub->publish(msg);
    }

  public:
    ArmsNode() : Node("arms_node")
    {
        arms_options opt;
        arms_options_init(&opt);
        opt.mode = ARMS_MODE_STANDALONE;
        config = declare_parameter<std::string>("config", "");
        if (!config.empty())
            opt.config_path = config.c_str();
        bool mock = declare_parameter("mock", false), power = declare_parameter("manage_power", false);
        bool rt_setup = declare_parameter("setup_rt", false);
        auto check = declare_parameter<int64_t>("check_motor", -1);
        opt.ignore_mask = declare_parameter<int64_t>("motor_disable", -1);
        opt.log = [](void *, const char *line) { RCLCPP_INFO(rclcpp::get_logger("arms_node"), "%s", line); };
        event_log = declare_parameter<std::string>("event_log", "");
        if (!event_log.empty())
            opt.event_log = event_log.c_str();
        max_age_ms = declare_parameter<int64_t>("command_max_age_ms", 100);
        if (check < -1 || check > 1 || opt.ignore_mask < -1 || opt.ignore_mask > 0x7fffffff ||
            max_age_ms < 1 || max_age_ms > 1000)
            throw std::invalid_argument("invalid startup parameters");
        opt.check_motors = (int)check;
        manage_power = power && !mock;
        if (mock)
            opt.backend = ARMS_BACKEND_MOCK;
        else {
            dev.lease = arms_acquire_lease();
            if (dev.lease < 0)
                throw std::runtime_error("hardware session busy or permission denied");
            if (rt_setup && (arms_env_start_master() != 0 || arms_env_rt_apply(opt.config_path, -1) != 0))
                throw std::runtime_error("real-time environment setup failed");
            if (power) {
                dev.owns_power = true;
                opt.scan_timeout_ms = 15000; /* 扫描等待总线稳定，不固定延时 */
                if (arms_power_on(0))
                    throw std::runtime_error("power-on request failed");
            }
        }
        if (arms_open(&opt, &dev.driver) != ARMS_OK)
            throw std::runtime_error(arms_last_error());
        {
            /* 上电后驱动器残留的故障先复位一次 */
            arms_joint_state s[14];
            size_t n = 0;
            arms_mask faulted = 0;
            arms_read(dev.driver, s, 14, &n);
            for (size_t i = 0; i < n; i++)
                if (s[i].mode == ARMS_JS_FAULT || s[i].error_code)
                    faulted |= ARMS_MASK_JOINT(s[i].joint);
            if (faulted && arms_reset(dev.driver, faulted, 0) != ARMS_OK)
                RCLCPP_WARN(get_logger(), "startup fault reset incomplete: %s", arms_last_error());
        }
        if (!healthy())
            throw std::runtime_error("startup selftest failed");
        {
            arms_mask released = 0, held = 0, unknown = 0;
            arms_brake_scan(dev.driver, &released, &held, &unknown);
            if (released | held)
                RCLCPP_WARN(get_logger(),
                            "brake not in normal mode (Pn075=1 released 0x%08x, Pn075=2 held 0x%08x); "
                            "enable/free switch the selected joints back to normal (Pn075=0)",
                            released, held);
            if (unknown)
                RCLCPP_WARN(get_logger(), "brake mode (Pn075) unreadable on joint mask 0x%08x", unknown);
        }
        auto qos = rclcpp::SensorDataQoS().keep_last(1);
        joint_pub = create_publisher<Joint>("hardware/joint_states", qos);
        state_pub = create_publisher<State>("hardware/actuator_states", qos);
        diag_pub = create_publisher<Diag>("hardware/arms/diagnostics", 10);
        cmd_sub = create_subscription<Cmd>("hardware/actuators_cmds", qos,
                                           [this](Cmd::ConstSharedPtr msg) { receive(*msg); });
        for (std::string name :
             {"enable", "disable", "reset", "selftest", "free", "hold", "brake_normal", "brake_release", "brake_hold",
              "zero"}) {
            services.push_back(create_service<Op>(
                "arms/" + name, [this, name](const std::shared_ptr<Op::Request> q,
                                             std::shared_ptr<Op::Response> r) { operation(name, *q, *r); }));
        }
        services.push_back(create_service<Trigger>(
            "arms/status",
            [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> r) { status(*r); }));
        services.push_back(create_service<Trigger>(
            "arms/clear_trip",
            [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> r) { clear_trip(*r); }));
        services.push_back(create_service<ReadParam>(
            "arms/read_param", [this](const std::shared_ptr<ReadParam::Request> q,
                                      std::shared_ptr<ReadParam::Response> r) { read_param(*q, *r); }));
        tick = create_wall_timer(10ms, [this] { feedback(); });
        report = create_wall_timer(500ms, [this] { diagnostics(); });
        RCLCPP_INFO(get_logger(), "communication event log: %s", arms_event_log_path(dev.driver));
        dev.watchdog = std::thread([this] {
            unsigned tick = 0;
            bool log_warned = false;
            while (!dev.stop) {
                arms_bus_stats s{};
                if (++tick % 10 == 0) {
                    /* 每 100 ms 把通信事件写入日志文件 */
                    int n = arms_log_events(dev.driver);
                    if (n > 0)
                        RCLCPP_WARN(get_logger(), "%d communication event(s) logged to %s", n,
                                    arms_event_log_path(dev.driver));
                    else if (n < 0 && !log_warned) {
                        RCLCPP_ERROR(get_logger(), "communication event log failed: %s", arms_last_error());
                        log_warned = true;
                    }
                }
                arms_get_stats(dev.driver, &s);
                if (s.tripped && !dev.cut.exchange(true)) {
                    arms_disable(dev.driver, arms_opened_joints(dev.driver));
                    if (dev.owns_power.exchange(false)) {
                        if (arms_power_off())
                            RCLCPP_ERROR(get_logger(), "power-off request FAILED");
                    } else
                        RCLCPP_ERROR(get_logger(),
                                     "trip latched; external power owner must verify power off");
                }
                std::this_thread::sleep_for(10ms);
            }
        });
        RCLCPP_INFO(get_logger(), "Startup checks passed; servos disabled.");
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int rc = 0;
    try {
        rclcpp::spin(std::make_shared<ArmsNode>());
    } catch (const std::exception &e) {
        fprintf(stderr, "arms_node: %s\n", e.what());
        rc = 1;
    }
    rclcpp::shutdown();
    return rc;
}
