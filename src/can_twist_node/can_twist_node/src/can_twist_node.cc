#include "can_twist/usb_can_v2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <cmath>
#include <cstring>

using namespace std;
using namespace rclcpp;

class CanTwistNode;

CanTwistNode* g_node_ptr = nullptr;

class CanTwistNode : public Node {
public:
    CanTwistNode() : Node("can_twist_node") {
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>( "cmd_vel", 10, std::bind(&CanTwistNode::twist_callback, this, std::placeholders::_1));
        usb_can_ = new usb_can_v2();
        g_node_ptr = this;
        usb_can_->listen(0x124, static_can_callback);
        vx_ = 0.0f;
        vy_ = 0.0f;
        yaw_angle_ = 0.0f;
    }

    ~CanTwistNode() {
        delete usb_can_; 
    }

private:
    void can_callback(uint16_t id, uint8_t* data, int length) {
        if (id == 0x124 && length >= 8) {
            float yaw_rad;
            memcpy(&yaw_rad, data, 4); 
            yaw_angle_ = yaw_rad;
            float vx_new = vx_ * cos(yaw_angle_) - vy_ * sin(yaw_angle_);
            float vy_new = vx_ * sin(yaw_angle_) + vy_ * cos(yaw_angle_);

            float magnitude = sqrt(vx_new * vx_new + vy_new * vy_new);

            float vx_unit, vy_unit;
            const float EPS = 1e-6;  
            if (magnitude < EPS) {
                vx_unit = 0.0f;
                vy_unit = 0.0f;
            } else {
                vx_unit = vx_new / magnitude;
                vy_unit = vy_new / magnitude;
            }

            const int16_t INT16_MAX_SCALE = 5; 
            int16_t vx_int = static_cast<int16_t>(std::round(vx_unit * INT16_MAX_SCALE));
            int16_t vy_int = static_cast<int16_t>(std::round(vy_unit * INT16_MAX_SCALE));
            int16_t vz_int = 0; 
            int16_t power_int = 100; 

            vx_int = std::clamp(vx_int, static_cast<int16_t>(-1500), INT16_MAX_SCALE);
            vy_int = std::clamp(vy_int, static_cast<int16_t>(-1500), INT16_MAX_SCALE);

            uint8_t send_data[8] = {0};
            send_data[1] = vx_int & 0xFF;        
            send_data[0] = (vx_int >> 8) & 0xFF; 
            send_data[3] = vy_int & 0xFF;        
            send_data[2] = (vy_int >> 8) & 0xFF; 
            send_data[5] = vz_int & 0xFF;        
            send_data[4] = (vz_int >> 8) & 0xFF; 
            send_data[7] = power_int & 0xFF;     
            send_data[6] = (power_int >> 8) & 0xFF; 

            usb_can_->transmit(0x111, send_data, 8);
            RCLCPP_INFO(this->get_logger(), "下发0x111: vx=%d , vy=%d , 功率=%d%%", 
                        vx_int, vy_int, power_int );
        }
    }


    static void static_can_callback(uint16_t id, uint8_t* data, int length) {
        if (g_node_ptr != nullptr) {
            g_node_ptr->can_callback(id, data, length);
        }
    }

    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        vx_ = msg->linear.x; 
        vy_ = msg->linear.y; 
    }

    Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    usb_can_v2* usb_can_; 
    float vx_;            
    float vy_;        
    float yaw_angle_;   
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanTwistNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    g_node_ptr = nullptr;
    return 0;
}
