#include <QApplication>
#include <QLabel>
#include <QString>
#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/goal.hpp"


using pose_goal=rm_interfaces::msg::Goal;

class GoalPoseDisplay : public rclcpp::Node {
 public:
  GoalPoseDisplay() : Node("goal_pose_display") {
    subscription_ = this->create_subscription<pose_goal>(
        "goalPose", 10, [&](const pose_goal::SharedPtr msg) -> void {
          label_->setText(get_qstr_from_msg(msg));
        }); 

    label_ = new QLabel(get_qstr_from_msg(std::make_shared<pose_goal>()));
    label_->show();
  }
QString get_qstr_from_msg(const pose_goal::SharedPtr msg) {
    std::stringstream show_str;
    show_str
        << "===========当前机器人坐标============\n"
        << "x:\t" << msg->goal_x << "\t\n"
        << "y:\t" << msg->goal_y << "\t\n"
        << "z:\t" << msg->goal_z << "\t\n"
        << "==========================================";

    return QString::fromStdString(show_str.str());
  }


 private:
  rclcpp::Subscription<pose_goal>::SharedPtr subscription_;
  QLabel* label_;
};


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  auto node = std::make_shared<GoalPoseDisplay>();
  std::thread spin_thread([&]() -> void { rclcpp::spin(node); });
  spin_thread.detach();
  app.exec();
  rclcpp::shutdown();
  return 0;
}