#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <communication/msg/actuator_cmds.hpp>
#include <communication/msg/actuator_states.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <arms_ros/srv/joint_operation.hpp>
#include "arms/arms.h"
#include "arms/lease.h"
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

// Also runs when a partially constructed node fails its startup checks.
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
    std::string config;
    rclcpp::Publisher<Joint>::SharedPtr joint_pub;
    rclcpp::Publisher<State>::SharedPtr state_pub;
    rclcpp::Publisher<Diag>::SharedPtr diag_pub;
    rclcpp::Subscription<Cmd>::SharedPtr cmd_sub;
    std::vector<rclcpp::Service<Op>::SharedPtr> services;
    rclcpp::TimerBase::SharedPtr tick, report;
    int64_t max_age_ms;
    unsigned rejected = 0;
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
                !std::isfinite(c.torque) || !std::isfinite(c.kp) || !std::isfinite(c.kd) || c.kp < 0 ||
                c.kd < 0) {
                reject("duplicate or invalid value");
                return;
            }
            mask |= bit;
            cmds[j] = c;
            if (s.mode != ARMS_JS_ENABLED)
                continue;
            arms_joint_limits l{};
            arms_get_limits(dev.driver, j, &l);
            if (c.position < l.pos_min || c.position > l.pos_max || std::abs(c.velocity) > l.vel_max ||
                std::abs(c.torque) > l.tff_max || c.kp > l.kp_max || c.kd > l.kd_max) {
                reject("configured joint limit");
                return;
            }
        }
        for (int j = 15; j <= 28; j++)
            if (mask & ARMS_MASK_JOINT(j)) {
                arms_joint_state s{};
                arms_read_joint(dev.driver, j, &s);
                if (s.mode == ARMS_JS_ENABLED && arms_set_command(dev.driver, j, &cmds[j], 0) != ARMS_OK) {
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
                    one = arms_service(dev.driver, j, q.hardware_zero ? ARMS_ZERO_HW : ARMS_ZERO_SOFT, q.confirm);
                if (one != ARMS_OK)
                    failed += std::to_string(j) + ": " + arms_last_error() + "; ";
            }
            r.success = failed.empty();
            r.message = r.success ? "OK" : failed;
            return;
        }
        r.success = st == ARMS_OK;
        r.message = r.success ? "OK" : std::string(arms_strerror(st)) + ": " + arms_last_error();
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
        const double unknown = std::numeric_limits<double>::quiet_NaN();
        a.driver_temperature.assign(n, unknown);
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
        arms_read(dev.driver, states, ARMS_JOINT_COUNT, &n);
        for (size_t k = 0; k < n; ++k) {
            const auto &s = states[k];
            if (s.mode == ARMS_JS_FAULT || s.error_code) faults[s.arm] |= ARMS_MASK_JOINT(s.joint);
        }
        for (int i = 0; i < ARMS_ARM_COUNT; i++) {
            auto &a = b.arm[i];
            if (!a.present)
                continue;
            diagnostic_msgs::msg::DiagnosticStatus d;
            d.name = std::string("arms/") + arms_arm_name(i);
            d.hardware_id = a.interface;
            const bool prot = b.protect.reason != ARMS_PROTECT_NONE;
            d.level = (dev.cut || b.tripped || prot || faults[i] || a.slaves_op != a.slaves_expected || a.comm_failed || !a.link_up ||
                       a.wkc_last != a.wkc_expected)
                          ? 2
                          : 0;
            d.message = prot ? (b.protect.reason == ARMS_PROTECT_OVERSPEED ? "overspeed protection latched"
                                                                           : "overtorque protection latched")
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
        max_age_ms = declare_parameter<int64_t>("command_max_age_ms", 100);
        if (check < -1 || check > 1 || opt.ignore_mask < -1 || opt.ignore_mask > 0x7fffffff ||
            max_age_ms < 1 || max_age_ms > 1000)
            throw std::invalid_argument("invalid startup parameters");
        opt.check_motors = (int)check;
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
                            "brake not in normal mode (released 0x%08x, held 0x%08x); enable refused until brake_normal",
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
             {"enable", "disable", "reset", "selftest", "free", "brake_normal", "brake_release", "brake_hold", "zero"}) {
            services.push_back(create_service<Op>(
                "arms/" + name, [this, name](const std::shared_ptr<Op::Request> q,
                                             std::shared_ptr<Op::Response> r) { operation(name, *q, *r); }));
        }
        tick = create_wall_timer(10ms, [this] { feedback(); });
        report = create_wall_timer(500ms, [this] { diagnostics(); });
        dev.watchdog = std::thread([this] {
            while (!dev.stop) {
                arms_bus_stats s{};
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
