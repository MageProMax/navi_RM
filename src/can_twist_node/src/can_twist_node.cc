#include "can_twist/usb_can_v2.hpp"
#include "can_twist/game_state.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <rm_interfaces/msg/game_para.hpp>
#include <rm_interfaces/msg/own_para.hpp>
#include <rm_interfaces/msg/ally_para.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>

class CanTwistNode : public rclcpp::Node {
public:
    CanTwistNode() : Node("can_twist_node") {
        use_sim_time_ = get_parameter("use_sim_time").as_bool();
        dry_run_ = declare_parameter<bool>("dry_run", false);
        allow_rotation_ = declare_parameter<bool>("allow_rotation", false);
        spin_enabled_ = declare_parameter<bool>("spin_enabled", true);
        require_permission_ = declare_parameter<bool>("require_spin_permission", false);
        spin_duration_ = declare_parameter<double>("spin_duration", 10.0);
        spin_speed_ = declare_parameter<double>("spin_speed", 1.0);
        if (!std::isfinite(spin_duration_) || spin_duration_ <= 0 || spin_duration_ > 3600.0 ||
            !std::isfinite(spin_speed_) || std::abs(spin_speed_) > 1.0 || spin_speed_ == 0)
            throw std::invalid_argument("Invalid spin duration/speed");
        timeout_ = declare_parameter<double>("cmd_vel_timeout", 0.25);
        linear_limit_ = declare_parameter<double>("linear_limit", 0.5);
        angular_limit_ = declare_parameter<double>("angular_limit", 1.0);
        if (!std::isfinite(timeout_) || timeout_ <= 0 ||
            !std::isfinite(linear_limit_) || linear_limit_ <= 0 || linear_limit_ > 1 ||
            !std::isfinite(angular_limit_) || angular_limit_ <= 0 || angular_limit_ > 1)
            throw std::invalid_argument("Timeout must be positive; CAN limits must be in (0, 1]");
        game_pub_ = create_publisher<rm_interfaces::msg::GamePara>("game_para", 10);
        own_pub_ = create_publisher<rm_interfaces::msg::OwnPara>("own_para", 10);
        ally_pub_ = create_publisher<rm_interfaces::msg::AllyPara>("ally_para", 10);
        frame_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("can_twist/command_frame", 1);
        command_pub_ = create_publisher<geometry_msgs::msg::Twist>("can_twist/commanded_velocity", 10);
        if (!dry_run_) {
            usb_can_ = std::make_unique<usb_can_v2>();
            usb_can_->listen(0x153, [this](uint16_t, uint8_t *data, int length) {
                if (length >= 8) handleGameState(data);
            });
            usb_can_->listen(0x155, [this](uint16_t, uint8_t *data, int length) {
                if (length >= 8) handleRobotState(data);
            });
            if (!usb_can_->ready()) RCLCPP_ERROR(get_logger(), "USB-CAN unavailable; motion output inhibited");
        }
        twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 1, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
                if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
                    !std::isfinite(msg->angular.z)) {
                    have_command_ = false;
                    command_ = geometry_msgs::msg::Twist();
                    RCLCPP_WARN(get_logger(), "Invalid cmd_vel; clearing motion command");
                    return;
                }
                // A command while stopped is discarded, never cached for restart.
                if (stopped_) return;
                command_ = *msg;
                have_command_ = true;
                last_command_ = controlNow();
            });
        permission_sub_ = create_subscription<std_msgs::msg::Bool>(
            "spin_motion_permitted", 1, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
                spin_permitted_ = msg->data;
                permission_receipt_ = controlNow();
            });
        // Process telemetry on the ROS executor, avoiding USB/ROS state races.
        own_sub_ = create_subscription<rm_interfaces::msg::OwnPara>(
            "own_para", 10, [this](rm_interfaces::msg::OwnPara::ConstSharedPtr msg) {
                if (msg->maximum_hp == 0 || msg->current_hp > msg->maximum_hp) return;
                const auto now = controlNow();
                if (spin_enabled_ && !stopped_ && have_hp_ && msg->current_hp < previous_hp_ &&
                    msg->hp_deduction_reason == static_cast<uint8_t>(HPDeductionReason::ARMOR_ATTACKED) &&
                    now >= spin_until_) {
                    spin_until_ = now + static_cast<int64_t>(spin_duration_ * 1e9);
                    RCLCPP_WARN(get_logger(), "Hit-triggered spin requested: %.2f rad/s for %.1fs",
                                spin_speed_, spin_duration_);
                }
                previous_hp_ = msg->current_hp;
                have_hp_ = true;
            });
        spin_pub_ = create_publisher<std_msgs::msg::Bool>("spin_active", 10);
        stop_sub_ = create_subscription<std_msgs::msg::Bool>(
            "emergency_stop", 1, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
                stopped_ = msg->data;
                // Both stop and release require a NEW command before movement.
                have_command_ = false;
                spin_until_ = int64_t{0};
                command_ = geometry_msgs::msg::Twist();
                sendCommand(geometry_msgs::msg::Twist());
            });
        timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { controlTick(); });
        RCLCPP_INFO(get_logger(), "CAN command arbiter ready: dry_run=%d rotation=%d timeout=%.2fs",
                    dry_run_, allow_rotation_, timeout_);
    }
    ~CanTwistNode() override {
        if (usb_can_) {
            const std::array<uint8_t, 8> stop{{0, 0, 0, 0, 0, 0, 100, 0}};
            usb_can_->transmit(0x120, stop.data(), stop.size());
            usb_can_->close(); // All callbacks finish while their owner is alive.
        }
    }
private:
    int64_t controlNow() {
        return use_sim_time_ ? get_clock()->now().nanoseconds() :
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    void controlTick() {
        geometry_msgs::msg::Twist effective;
        const int64_t now = controlNow();
        if (now < previous_tick_) {
            have_command_ = false;
            spin_until_ = 0;
            spin_permitted_ = false;
        }
        previous_tick_ = now;
        const double age = (now - last_command_) * 1e-9;
        if (!stopped_ && have_command_ && 0 <= age && age < timeout_ &&
            (dry_run_ || (usb_can_ && usb_can_->ready()))) {
            effective.linear.x = std::clamp(command_.linear.x, -linear_limit_, linear_limit_);
            effective.linear.y = std::clamp(command_.linear.y, -linear_limit_, linear_limit_);
            effective.angular.z = allow_rotation_ ?
                std::clamp(command_.angular.z, -angular_limit_, angular_limit_) : 0.0;
        }
        const bool permission_valid = !require_permission_ || (spin_permitted_ &&
            0 <= now-permission_receipt_ && (now-permission_receipt_) * 1e-9 < timeout_);
        const bool spin_active = spin_enabled_ && !stopped_ && have_command_ &&
            0 <= age && age < timeout_ && permission_valid && now < spin_until_ &&
            (dry_run_ || (usb_can_ && usb_can_->ready()));
        if (spin_active)
            effective.angular.z = std::clamp(spin_speed_,
                                             -angular_limit_, angular_limit_);
        if (stopped_ || !permission_valid || age >= timeout_)
            spin_until_ = int64_t{0};
        std_msgs::msg::Bool status;
        status.data = spin_active;
        spin_pub_->publish(status);
        sendCommand(effective);
    }
    void sendCommand(const geometry_msgs::msg::Twist &effective) {
        command_pub_->publish(effective); // Intended ROS-frame command, not feedback.
        // Measured chassis convention: first field forward, second right, positive yaw left.
        const std::array<int16_t, 4> values{{
            static_cast<int16_t>(std::lround(effective.linear.x * 500)),
            static_cast<int16_t>(std::lround(-effective.linear.y * 500)),
            static_cast<int16_t>(std::lround(effective.angular.z * 500)), 100}};
        std::array<uint8_t, 8> bytes{};
        for (std::size_t i = 0; i < values.size(); ++i) {
            const auto value = static_cast<uint16_t>(values[i]);
            bytes[2*i] = value & 0xFF;
            bytes[2*i+1] = (value >> 8) & 0xFF;
        }
        std_msgs::msg::UInt8MultiArray wire;
        wire.data.assign(bytes.begin(), bytes.end());
        frame_pub_->publish(wire);
        if (!usb_can_) return;
        const int ret = usb_can_->transmit(0x120, bytes.data(), bytes.size());
        if (ret != USB_CAN_OK)
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "CAN send failed (%d); command was not delivered", ret);
    }
    // These two callbacks execute sequentially on the receiver thread. They
    // publish telemetry only and do not share motion state with ROS callbacks.
    void handleGameState(const uint8_t *data) {
        game_.parse_0x153(data);
        rm_interfaces::msg::GamePara msg;
        msg.game_progress = game_.game_progress;
        msg.stage_remain_time = game_.stage_remain_time;
        msg.recovery_occupy = game_.recovery_occupy;
        msg.mid_occupy = game_.mid_occupy;
        msg.recovery_rfid = game_.recovery_rfid;
        msg.mid_rfid = game_.mid_rfid;
        game_pub_->publish(msg);
    }
    void handleRobotState(const uint8_t *data) {
        own_.parse_0x155(data);
        ally_.parse_0x155_extended(data);
        rm_interfaces::msg::OwnPara own;
        own.current_hp = own_.current_HP;
        own.maximum_hp = own_.maximum_HP;
        own.hp_deduction_reason = game_.HP_deduction_reason;
        own.recovery_rfid = own_.recovery_rfid;
        own.mid_rfid = own_.mid_rfid;
        own.power_management_shooter_output = game_.power_management_shooter_output;
        own.recovery_buff = game_.recovery_buff;
        own.shooter_remain = own_.shooter_remain;
        own_pub_->publish(own);
        rm_interfaces::msg::AllyPara ally;
        ally.ally_1_robot_hp = ally_.ally_1_robot_HP;
        ally.ally_3_robot_hp = ally_.ally_3_robot_HP;
        ally.level = ally_.level;
        ally.offending_robot_id = ally_.offending_robot_id;
        ally_pub_->publish(ally);
    }
    bool dry_run_ = false, allow_rotation_ = false;
    bool spin_enabled_ = true, require_permission_ = false, spin_permitted_ = false;
    bool have_hp_ = false;
    uint16_t previous_hp_ = 0;
    double spin_duration_ = 10.0, spin_speed_ = 1.0;
    bool use_sim_time_ = false;
    int64_t spin_until_ = 0, permission_receipt_ = 0, previous_tick_ = 0;
    bool stopped_ = false, have_command_ = false;
    double timeout_ = 0.25, linear_limit_ = 0.5, angular_limit_ = 1.0;
    geometry_msgs::msg::Twist command_;
    int64_t last_command_ = 0;
    std::unique_ptr<usb_can_v2> usb_can_;
    GamePara game_{};
    OwnPara own_{};
    AllyPara ally_{};
    rclcpp::Publisher<rm_interfaces::msg::GamePara>::SharedPtr game_pub_;
    rclcpp::Publisher<rm_interfaces::msg::OwnPara>::SharedPtr own_pub_;
    rclcpp::Publisher<rm_interfaces::msg::AllyPara>::SharedPtr ally_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr frame_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_, permission_sub_;
    rclcpp::Subscription<rm_interfaces::msg::OwnPara>::SharedPtr own_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spin_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanTwistNode>();
    rclcpp::spin(node);
    node.reset(); // Send final stop while middleware and callback owners exist.
    rclcpp::shutdown();
    return 0;
}
