#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/polygon.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/layer.hpp"

namespace real_bringup
{

// ROS callbacks only queue the newest polygon. The map update thread applies it
// under the same costmap mutex used by MPPI and the global planner.
class FootprintUpdateLayer : public nav2_costmap_2d::Layer
{
public:
  void queueFootprint(const geometry_msgs::msg::Polygon::SharedPtr polygon);
  void updateBounds(double, double, double, double *, double *, double *, double *) override;
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override {}
  void reset() override {current_ = true;}
  bool isClearable() override {return false;}

protected:
  void onInitialize() override;

private:
  std::weak_ptr<nav2_costmap_2d::Costmap2DROS> owner_;
  std::mutex pending_mutex_;
  std::optional<std::vector<geometry_msgs::msg::Point>> pending_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr subscription_;
};

}  // namespace real_bringup
