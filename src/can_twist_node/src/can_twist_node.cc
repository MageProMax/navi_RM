#include "can_twist/usb_can_v2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

using namespace std;
using namespace rclcpp;

class CanTwistNode : public Node {
public:
    CanTwistNode() : Node("can_twist_node") {
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&CanTwistNode::twist_callback, this, std::placeholders::_1));
        
        usb_can_ = new usb_can_v2();

        RCLCPP_INFO(this->get_logger(), "CAN Twist Node Started");
    }

    ~CanTwistNode() {
        delete usb_can_; 
    }

private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {

        float vx = -msg->linear.y;
        float vy = msg->linear.x;
        float wz = msg->angular.z;

        // 假设线速度和角速度的映射比例不同，在此处统一定义
        const float LINEAR_SCALE = 500.0f;  // 线速度缩放系数
        const float ANGULAR_SCALE = 500.0f;   // 角速度缩放系数 (根据实际协议修改)

        int16_t vx_int = static_cast<int16_t>(std::round(vx * LINEAR_SCALE));
        int16_t vy_int = static_cast<int16_t>(std::round(vy * LINEAR_SCALE));
        int16_t wz_int = static_cast<int16_t>(std::round(wz * ANGULAR_SCALE));
        int16_t power_int = 100; // 功率示例值

        vx_int = std::clamp(vx_int, static_cast<int16_t>(-LINEAR_SCALE), static_cast<int16_t>(LINEAR_SCALE));
        vy_int = std::clamp(vy_int, static_cast<int16_t>(-LINEAR_SCALE), static_cast<int16_t>(LINEAR_SCALE));
        wz_int = std::clamp(wz_int, static_cast<int16_t>(-ANGULAR_SCALE), static_cast<int16_t>(ANGULAR_SCALE));

        uint8_t send_data[8] = {0};
        send_data[1] = (vx_int >> 8) & 0xFF; 
        send_data[0] = vx_int & 0xFF;        
        send_data[3] = (vy_int >> 8) & 0xFF; 
        send_data[2] = vy_int & 0xFF;        
        send_data[5] = (wz_int >> 8) & 0xFF; 
        send_data[4] = wz_int & 0xFF;        
        send_data[7] = (power_int >> 8) & 0xFF; 
        send_data[6] = power_int & 0xFF;        
        usb_can_->transmit(0x120, send_data, 8);
        RCLCPP_INFO(this->get_logger(), "Sent 0x120: vx=%d, vy=%d, wz=%d", vx_int, vy_int, wz_int);
    }
    Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    usb_can_v2* usb_can_; 
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanTwistNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
