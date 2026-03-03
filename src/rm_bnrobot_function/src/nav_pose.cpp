#include <memory>
#include <mutex>
#include "nav2_msgs/action/navigate_to_pose.hpp"  
#include "rclcpp/rclcpp.hpp"  
#include "rclcpp_action/rclcpp_action.hpp"  
#include "rm_interfaces/msg/goal.hpp"  

using NavigationAction = nav2_msgs::action::NavigateToPose;  
using NavActGoalHandle = rclcpp_action::ClientGoalHandle<NavigationAction>;  

class NavToPoseClient : public rclcpp::Node {
public: 
  rclcpp_action::Client<NavigationAction>::SharedPtr action_client_;
  rclcpp::Subscription<rm_interfaces::msg::Goal>::SharedPtr target_sub_;
  
  std::mutex goal_mutex_;
  rm_interfaces::msg::Goal current_goal_;
  bool has_new_goal_;
  bool action_server_ready_;
  NavActGoalHandle::SharedPtr current_goal_handle_;

  NavToPoseClient() : Node("nav_to_pose_client"), 
                      has_new_goal_(false),
                      action_server_ready_(false),
                      current_goal_handle_(nullptr) {
    // 创建动作客户端
    action_client_ = rclcpp_action::create_client<NavigationAction>(
        this, "navigate_to_pose");
    
    // 订阅targetPose话题
    target_sub_ = this->create_subscription<rm_interfaces::msg::Goal>(
      "targetPose", 10,
      std::bind(&NavToPoseClient::targetCallback, this, std::placeholders::_1));
    
    // 初始化参数
    this->declare_parameter("goal.frame_id", "map");
    
    RCLCPP_INFO(get_logger(), "导航客户端已启动，等待targetPose消息...");
    
    // 启动动作服务器检查线程
    std::thread(&NavToPoseClient::checkActionServer, this).detach();
  }

  void targetCallback(const rm_interfaces::msg::Goal::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    
    // 检查目标是否改变
    if (current_goal_.goal_x != msg->goal_x || 
        current_goal_.goal_y != msg->goal_y || 
        current_goal_.goal_z != msg->goal_z) {
      
      current_goal_ = *msg;
      has_new_goal_ = true;
      
      RCLCPP_INFO(get_logger(), 
                  "收到新目标: x=%.2f, y=%.2f, z=%.2f", 
                  msg->goal_x, msg->goal_y, msg->goal_z);
      
      // 如果动作服务器已就绪，立即发送目标
      if (action_server_ready_) {
        sendNavigationGoal(current_goal_);
        has_new_goal_ = false;
      }
    }
  }

  void checkActionServer() {
    // 等待动作服务器上线
    while (rclcpp::ok()) {
      if (action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        action_server_ready_ = true;
        RCLCPP_INFO(get_logger(), "导航动作服务器已连接");
        
        // 服务器就绪后，发送缓存的目
        std::lock_guard<std::mutex> lock(goal_mutex_);
        if (has_new_goal_) {
          sendNavigationGoal(current_goal_);
          has_new_goal_ = false;
        }
        break;
      }
      RCLCPP_INFO(get_logger(), "等待导航动作服务器上线...");
    }
  }

  void sendNavigationGoal(const rm_interfaces::msg::Goal& goal_msg) {
    // 检查是否有正在执行的任务，如果有则取消
    if (current_goal_handle_) {
      RCLCPP_INFO(get_logger(), "取消当前导航任务");
      action_client_->async_cancel_goal(current_goal_handle_);
      current_goal_handle_ = nullptr;
    }

    //创建导航目标
    auto nav_goal_msg = NavigationAction::Goal();
    
    //设置header
    nav_goal_msg.pose.header.stamp = this->now();
    this->get_parameter("goal.frame_id", nav_goal_msg.pose.header.frame_id);
    
    //设置位置
    nav_goal_msg.pose.pose.position.x = goal_msg.goal_x;
    nav_goal_msg.pose.pose.position.y = goal_msg.goal_y;
    nav_goal_msg.pose.pose.position.z = goal_msg.goal_z;
    
    //设置方向
    nav_goal_msg.pose.pose.orientation.x = 0.0;
    nav_goal_msg.pose.pose.orientation.y = 0.0;
    nav_goal_msg.pose.pose.orientation.z = 0.0;
    nav_goal_msg.pose.pose.orientation.w = 1.0;

    //设置动作选项
    auto send_goal_options = rclcpp_action::Client<NavigationAction>::SendGoalOptions();
    
    //目标响应回调
    send_goal_options.goal_response_callback =
        [this](NavActGoalHandle::SharedPtr goal_handle) {
          if (goal_handle) {
            current_goal_handle_ = goal_handle;
            RCLCPP_INFO(get_logger(), "导航目标已接受，开始导航");
          } else {
            RCLCPP_ERROR(get_logger(), "导航目标被拒绝");
          }
        };
    
    send_goal_options.feedback_callback =
        [this](
            NavActGoalHandle::SharedPtr goal_handle,
            const std::shared_ptr<const NavigationAction::Feedback> feedback) {
          (void)goal_handle;  
          RCLCPP_INFO(this->get_logger(), 
                      "剩余距离: %.2f 米，导航时间: %.2f 秒",
                      feedback->distance_remaining,
                      feedback->navigation_time.sec + feedback->navigation_time.nanosec * 1e-9);
        };
    
    send_goal_options.result_callback =
        [this](const NavActGoalHandle::WrappedResult& result) {
          current_goal_handle_ = nullptr;
          
          switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
              RCLCPP_INFO(this->get_logger(), "成功到达目的地");
              break;
            case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_WARN(this->get_logger(), "导航被取消");
              break;
            case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_ERROR(this->get_logger(), "导航失败");
              break;
            default:
              RCLCPP_ERROR(this->get_logger(), "未知结果");
              break;
          }
        };

    //发送导航目标
    RCLCPP_INFO(get_logger(), 
                "发送导航目标到: (%.2f, %.2f, %.2f)", 
                goal_msg.goal_x, goal_msg.goal_y, goal_msg.goal_z);
    
    action_client_->async_send_goal(nav_goal_msg, send_goal_options);
  }


};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavToPoseClient>();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}