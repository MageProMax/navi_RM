#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>
#include <stdexcept>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rm_interfaces/msg/goal.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rm_interfaces/srv/configure_patrol.hpp"
#include "rm_interfaces/msg/navigation_status.hpp"
#include <set>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using NavigationAction = nav2_msgs::action::NavigateToPose;
using NavActGoalHandle = rclcpp_action::ClientGoalHandle<NavigationAction>;

// All state is handled by the single-threaded executor. No detached worker can
// outlive this node. Nav2 accepts a new goal as a preemption of the old goal.
class NavToPoseClient : public rclcpp::Node {
public:
  NavToPoseClient() : Node("nav_to_pose_client") {
    require_mapping_ready_ = declare_parameter<bool>("require_mapping_ready", false);
    if (require_mapping_ready_) {
      mapping_sub_ = create_subscription<std_msgs::msg::Bool>("/pgo/map_ready", rclcpp::QoS(10).reliable().transient_local(),
        [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
          if (!msg->data && mapping_ready_) {
            cancelCurrent(); ++revision_;
            if (mode_=="explore" || mode_=="explore_goal") { have_goal_=false;pending_=false; }
            else pending_=have_goal_;
            state_="waiting_mapping";
          }
          if (msg->data && !mapping_ready_) mapping_ready_since_=now().seconds();
          mapping_ready_=msg->data;
        });
    }
    require_motion_ready_ = declare_parameter<bool>("require_motion_ready", false);
    if (require_motion_ready_) {
      motion_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/can_twist/command_frame", 1, [this](std_msgs::msg::UInt8MultiArray::ConstSharedPtr msg) {
          if (msg->data.size() == 8) { have_motion_frame_ = true; last_motion_frame_ = now().seconds(); }
        });
    }
    require_localization_ = declare_parameter<bool>("require_localization", false);
    if (require_localization_) {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    }
    frame_ = declare_parameter<std::string>("goal.frame_id", "map");
    mode_ = declare_parameter<std::string>("patrol.mode", "formal");
    if (mode_ != "formal" && mode_ != "test" && mode_ != "explore")
      throw std::invalid_argument("patrol.mode must be formal, test or explore");
    if (mode_ == "test") {
      loop_ = declare_parameter<bool>("patrol.loop", true);
      wait_seconds_ = declare_parameter<double>("patrol.wait_seconds", 1.0);
      if (!std::isfinite(wait_seconds_) || wait_seconds_ < 0 || wait_seconds_ > 3600)
        throw std::invalid_argument("patrol.wait_seconds must be in [0,3600]");
      const auto names = declare_parameter<std::vector<std::string>>("patrol.point_names", std::vector<std::string>{});
      if (names.empty()) throw std::invalid_argument("Test patrol requires patrol.point_names");
      for (const auto& name : names) {
        const auto xyz = declare_parameter<std::vector<double>>("patrol.points." + name, std::vector<double>{});
        if (xyz.size() != 3 || !std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2]))
          throw std::invalid_argument("patrol.points." + name + " must contain finite [x,y,z]");
        rm_interfaces::msg::Goal point;
        point.goal_frame_id = frame_;
        point.goal_x = xyz[0];point.goal_y = xyz[1];point.goal_z = xyz[2];
        points_.push_back(point);
        point_names_.push_back(name);
      }
      schedulePoint();
    }
    action_client_ = rclcpp_action::create_client<NavigationAction>(this, "navigate_to_pose");
    auto subscribe_target = [this](const std::string& topic, const std::string& source_mode) {
      return create_subscription<rm_interfaces::msg::Goal>(topic,10,
[this,source_mode](rm_interfaces::msg::Goal::ConstSharedPtr msg) {
        if (mode_ != source_mode && !(source_mode=="explore" && mode_=="explore_goal")) return;
        if (!std::isfinite(msg->goal_x) || !std::isfinite(msg->goal_y) ||
            !std::isfinite(msg->goal_z)) {
          RCLCPP_ERROR(get_logger(), "Ignoring non-finite navigation target");
          return;
        }
        auto goal = *msg;
        if (goal.goal_frame_id.empty()) goal.goal_frame_id = frame_;
        if (have_goal_ && goal.goal_frame_id == desired_.goal_frame_id &&
            goal.goal_x == desired_.goal_x && goal.goal_y == desired_.goal_y &&
            goal.goal_z == desired_.goal_z) return;
        desired_ = goal;
        have_goal_ = true;
        ++revision_;
        pending_ = true;
      });
    };
    target_sub_=subscribe_target("targetPose","formal");
    explore_sub_=subscribe_target("/navigation/exploration_goal","explore");
    intent_pub_ = create_publisher<rm_interfaces::msg::Goal>("/navigation/exploration_target", rclcpp::QoS(1).reliable().transient_local());
    status_pub_ = create_publisher<rm_interfaces::msg::NavigationStatus>("/navigation/client/status", 10);
    configure_srv_ = create_service<rm_interfaces::srv::ConfigurePatrol>("/navigation/client/configure",
      [this](rm_interfaces::srv::ConfigurePatrol::Request::SharedPtr req,
             rm_interfaces::srv::ConfigurePatrol::Response::SharedPtr res) { configure(*req, *res); });
    start_srv_ = create_service<std_srvs::srv::Trigger>("/navigation/client/start",
      [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
        if (emergency_) { res->message="请先解除急停";return; }
        paused_=false;next_goal_time_=0;
        if (mode_=="test" && point_index_>=points_.size()) { point_index_=0;schedulePoint(); }
        if (have_goal_) { pending_=true;++revision_; }
        state_="waiting";dispatch();publishStatus();res->success=true;res->message=statusMessage();
      });
    pause_srv_ = create_service<std_srvs::srv::Trigger>("/navigation/client/pause",
      [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
        cancelCurrent();paused_=true;pending_=have_goal_;state_="paused";
        res->success=true;res->message="已请求暂停；继续时重试当前点";
      });
    stop_srv_ = create_service<std_srvs::srv::Trigger>("/navigation/client/stop",
      [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
        cancelCurrent();paused_=true;pending_=false;state_="stopped";
        res->success=true;res->message="已请求取消导航";
      });
    clear_explore_srv_ = create_service<std_srvs::srv::Trigger>("/navigation/client/clear_exploration_goal",
      [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
        if(mode_!="explore" && mode_!="explore_goal") { res->message="当前不是探索模式";return; }
        cancelCurrent();have_goal_=false;pending_=false;state_="waiting";
        res->success=true;res->message="已取消失效探索目标，等待新的前沿";
      });
    emergency_sub_ = create_subscription<std_msgs::msg::Bool>("/emergency_stop",10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        emergency_=msg->data;
        if (emergency_) { cancelCurrent();paused_=true;pending_=false;state_="emergency"; }
        else if(state_=="emergency") state_="paused";
      });
    status_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() { publishStatus(); });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { dispatch(); });
    RCLCPP_INFO(get_logger(), "Navigation client ready; awaiting targetPose / NavigateToPose server");
  }

private:
  void dispatch() {
    if (paused_ || emergency_) return;
    if (require_mapping_ready_ && (!mapping_ready_ || now().seconds()-mapping_ready_since_<1.0)) {state_="waiting_mapping";return;}
    if (now().nanoseconds() < next_goal_time_) return;
    if (!pending_ || (request_in_flight_ && request_revision_ == revision_)) return;
    if (!action_client_->action_server_is_ready()) { state_="waiting_server";return; }
    if (require_motion_ready_ && (!have_motion_frame_ || now().seconds() < last_motion_frame_ ||
        now().seconds() - last_motion_frame_ > .25)) { state_="waiting_motion";return; }
    if (require_localization_) {
      state_="waiting_localization";
      if (!tf_buffer_->canTransform(desired_.goal_frame_id, "nav_base_link", tf2::TimePointZero)) return;
      try {
        const auto pose = tf_buffer_->lookupTransform(desired_.goal_frame_id, "nav_base_link", tf2::TimePointZero);
        const double age = now().seconds() - pose.header.stamp.sec - pose.header.stamp.nanosec * 1e-9;
        if (age < 0 || age > .5) return;
      } catch (const tf2::TransformException&) { return; }
    }
    NavigationAction::Goal goal;
    goal.pose.header.stamp = now();
    goal.pose.header.frame_id = desired_.goal_frame_id;
    goal.pose.pose.position.x = desired_.goal_x;
    goal.pose.pose.position.y = desired_.goal_y;
    goal.pose.pose.position.z = desired_.goal_z;
    goal.pose.pose.orientation.w = 1.0;
    const auto revision = revision_;
    pending_ = false;
    request_in_flight_ = true;request_revision_=revision;
    state_="sending";
    distance_remaining_=-1.;
    auto options = rclcpp_action::Client<NavigationAction>::SendGoalOptions();
    options.goal_response_callback = [this, revision](NavActGoalHandle::SharedPtr handle) {
      if (request_revision_ == revision) request_in_flight_ = false;
      if (revision != revision_) {
        // A newer decision arrived while this request was being accepted.
        if (handle) action_client_->async_cancel_goal(handle);
        return;
      }
      if (handle) {
        current_handle_=handle;state_="executing";
        RCLCPP_INFO(get_logger(), "Navigation target accepted");
      } else {
        state_="rejected";
        RCLCPP_ERROR(get_logger(), "Navigation target rejected; waiting for a changed target");
      }
    };
    options.feedback_callback = [this,revision](NavActGoalHandle::SharedPtr,
        const std::shared_ptr<const NavigationAction::Feedback> feedback) {
      if (revision==revision_) distance_remaining_=feedback->distance_remaining;
    };
    options.result_callback = [this, revision](const NavActGoalHandle::WrappedResult& result) {
      if (revision != revision_) return; // Old results cannot clear a newer goal.
      current_handle_.reset();
      if (mode_=="explore" || mode_=="explore_goal") have_goal_=false;
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        state_="complete";
        RCLCPP_INFO(get_logger(), "Navigation target reached");
        if (mode_ == "test") {
          ++point_index_;
          if (point_index_ == points_.size()) {
            if (!loop_) {
              RCLCPP_INFO(get_logger(), "Test patrol complete");
              return;
            }
            point_index_ = 0;
          }
          next_goal_time_ = now().nanoseconds() + static_cast<int64_t>(wait_seconds_ * 1e9);
          schedulePoint();
        }
      } else {
        state_="failed";
        RCLCPP_WARN(get_logger(), "Navigation ended with result %d; waiting for a changed target",
                    static_cast<int>(result.code));
      }
    };
    RCLCPP_INFO(get_logger(), "Sending target %s: (%.3f, %.3f, %.3f)",
                desired_.goal_frame_id.c_str(), desired_.goal_x, desired_.goal_y, desired_.goal_z);
    action_client_->async_send_goal(goal, options);
  }

  void schedulePoint() {
    desired_ = points_.at(point_index_);
    have_goal_ = true;
    pending_ = true;
    state_="waiting";
    ++revision_; // Also permits revisiting the same coordinate on another lap.
  }

  void cancelCurrent() {
    ++revision_;
    if (current_handle_) action_client_->async_cancel_goal(current_handle_);
    current_handle_.reset();distance_remaining_=-1.;
  }
  void configure(const rm_interfaces::srv::ConfigurePatrol::Request& req,
                 rm_interfaces::srv::ConfigurePatrol::Response& res) {
    if ((req.mode!="formal" && req.mode!="test" && req.mode!="manual" && req.mode!="explore" && req.mode!="explore_goal") || req.frame_id.empty() ||
        !std::isfinite(req.wait_seconds) || req.wait_seconds<0 || req.wait_seconds>3600 ||
        req.points.size()!=req.point_names.size() || ((req.mode=="test" || req.mode=="manual") && req.points.empty())) {
      res.message="模式、坐标系或路线格式不正确";return;
    }
    if(req.mode=="explore_goal" && (req.points.size()!=1 || req.frame_id!="map")) { res.message="目标探索需要一个 map 坐标系目的地";return; }
    std::set<std::string> names;
    for(size_t i=0;i<req.points.size();++i) {
      const auto& p=req.points[i];
      if(req.point_names[i].empty() || !names.insert(req.point_names[i]).second ||
         !std::isfinite(p.goal_x) || !std::isfinite(p.goal_y) || !std::isfinite(p.goal_z)) {
        res.message="点名重复或坐标无效";return;
      }
    }
    cancelCurrent();paused_=true;pending_=false;have_goal_=false;
    mode_=req.mode;frame_=req.frame_id;loop_=req.loop;wait_seconds_=req.wait_seconds;
    point_index_=0;point_names_=req.point_names;points_=req.points;next_goal_time_=0;
    for(auto& p:points_) p.goal_frame_id=frame_;
    if(mode_=="test" || mode_=="manual") { desired_=points_.front();have_goal_=true; }
    rm_interfaces::msg::Goal intent;
    if(mode_=="explore_goal") intent=points_.front();
    intent_pub_->publish(intent);
    state_="ready";res.success=true;res.message="配置已应用，点击开始后执行";
  }
  std::string statusMessage() const {
    if(state_=="waiting_server") return "等待 NavigateToPose 动作服务就绪";
    if(state_=="waiting_motion") return "等待底盘仲裁心跳；目标尚未发送";
    if(state_=="waiting_mapping") return "等待回环地图重建及代价地图更新；旧路径已取消";
    if(state_=="waiting_localization") return "等待有效的 map 到 nav_base_link 定位 TF；目标尚未发送";
    if(state_=="sending") return "正在发送目标，等待 Nav2 接受";
    if(state_=="executing") return "Nav2 已接受目标，正在执行导航任务";
    if(state_=="waiting" && !have_goal_ && mode_=="explore_goal") return "目的地探索已启用，等待可达前沿逐段接近目标";
    if(state_=="waiting" && !have_goal_) return mode_=="explore" ?
        "自动探索已启用，等待探索器选择可达前沿；表格标点不参与自动探索" : "已启用正式决策，等待上层目标";
    if(state_=="waiting") return "目标等待执行或巡检停留中";
    if(state_=="failed") return "Nav2 导航失败，请检查规划或控制日志";
    if(state_=="rejected") return "Nav2 拒绝了目标";
    if(state_=="paused") return "任务已暂停";
    if(state_=="stopped") return "任务已取消";
    if(state_=="complete") return "目标已完成";
    if(state_=="emergency") return "急停中，先解除急停再开始";
    return "配置就绪，点击开始执行";
  }
  void publishStatus() {
    rm_interfaces::msg::NavigationStatus status;
    status.mode=mode_;status.state=state_;status.message=statusMessage();status.paused=paused_;
    status.active=static_cast<bool>(current_handle_);status.emergency_stop=emergency_;
    status.point_index=point_index_;status.point_count=points_.size();status.target=desired_;
    if(mode_=="formal") {
      for(size_t i=0;i<points_.size();++i) {
        if(std::abs(points_[i].goal_x-desired_.goal_x)<1e-6 &&
           std::abs(points_[i].goal_y-desired_.goal_y)<1e-6 &&
           std::abs(points_[i].goal_z-desired_.goal_z)<1e-6) {
          status.point_name=point_names_[i];status.point_index=i;break;
        }
      }
    } else if(mode_=="explore" || mode_=="explore_goal") status.point_name="exploration";
    else if(point_index_<point_names_.size()) status.point_name=point_names_[point_index_];
    status.distance_remaining=distance_remaining_;status_pub_->publish(status);
  }
  std::string state_{"waiting"};
  bool paused_{false}, emergency_{false};
  bool require_mapping_ready_{false}, mapping_ready_{false};
  double mapping_ready_since_{0};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mapping_sub_;
  double distance_remaining_{-1};
  std::vector<std::string> point_names_;
  NavActGoalHandle::SharedPtr current_handle_;
  rclcpp::Publisher<rm_interfaces::msg::NavigationStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<rm_interfaces::msg::Goal>::SharedPtr intent_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;
  rclcpp::Service<rm_interfaces::srv::ConfigurePatrol>::SharedPtr configure_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_,pause_srv_,stop_srv_,clear_explore_srv_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  bool require_motion_ready_{false}, have_motion_frame_{false};
  double last_motion_frame_{0};
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr motion_sub_;
  bool require_localization_{false};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string mode_;
  bool loop_{true};
  double wait_seconds_{1.0};
  size_t point_index_{0};
  int64_t next_goal_time_{0};
  std::vector<rm_interfaces::msg::Goal> points_;
  std::string frame_;
  rm_interfaces::msg::Goal desired_;
  bool have_goal_{false}, pending_{false}, request_in_flight_{false};
  uint64_t request_revision_{0};
  uint64_t revision_{0};
  rclcpp_action::Client<NavigationAction>::SharedPtr action_client_;
  rclcpp::Subscription<rm_interfaces::msg::Goal>::SharedPtr target_sub_,explore_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavToPoseClient>());
  rclcpp::shutdown();
  return 0;
}
