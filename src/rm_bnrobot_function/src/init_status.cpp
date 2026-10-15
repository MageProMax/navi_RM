#include "rclcpp/rclcpp.hpp"
#include "interface/srv/is_valid.hpp"
#include "rm_interfaces/msg/goal.hpp"

using namespace std::chrono_literals;

class InitStatusNode : public rclcpp::Node
{
public:
    InitStatusNode() : Node("init_status_node")
    {
        // 1. 创建发布者 (话题: /init_status)
        goal_pub_ = this->create_publisher<rm_interfaces::msg::Goal>("/init_status", 10);
        
        // 2. 创建服务客户端 (服务名: relocalize_check)
        check_client_ = this->create_client<interface::srv::IsValid>("relocalize_check");
        
        // 3. 创建定时器，每 1 秒去查询一次定位状态
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&InitStatusNode::timerCallback, this));
            
        RCLCPP_INFO(this->get_logger(), "Init Status Node started! Waiting for relocalization...");
    }

private:
    void timerCallback()
    {
        // 如果服务没上线，直接跳过等下一秒，不干等
        if (!check_client_->wait_for_service(0s)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "Service 'relocalize_check' is not available yet...");
            return;
        }

        // 构造请求
        auto request = std::make_shared<interface::srv::IsValid::Request>();
        // 注意：根据你之前的代码，code=1 会被强制设为 true，所以咱们这里传 0，获取真实的 m_state.localize_success
        request->code = 0; 

        // 异步发送请求，拿到结果后触发 Lambda 回调函数
        check_client_->async_send_request(
            request,
            [this](rclcpp::Client<interface::srv::IsValid>::SharedFuture future) {
                auto response = future.get();
                if (response->valid) {
                    publishGoal();
                    
                    // 【关键细节】：如果你只要发一次 (999,999,999)，发完就可以把定时器关了，避免一直发。
                    // 如果你需要一直周期性发，把下面这行注释掉就行！
                    timer_->cancel(); 
                }
            }
        );
    }

    void publishGoal()
    {
        rm_interfaces::msg::Goal msg;
        msg.goal_x = 999.0;
        msg.goal_y = 999.0;
        msg.goal_z = 999.0;
        
        goal_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "⚡ Relocalization SUCCESS! Published Goal(999.0, 999.0, 999.0)");
    }

    rclcpp::Publisher<rm_interfaces::msg::Goal>::SharedPtr goal_pub_;
    rclcpp::Client<interface::srv::IsValid>::SharedPtr check_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InitStatusNode>());
    rclcpp::shutdown();
    return 0;
}