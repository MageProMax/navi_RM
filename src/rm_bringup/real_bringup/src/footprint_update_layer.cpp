#include "real_bringup/footprint_update_layer.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "pluginlib/class_list_macros.hpp"

namespace real_bringup
{

void FootprintUpdateLayer::onInitialize()
{
  auto node = node_.lock();
  auto owner = std::dynamic_pointer_cast<nav2_costmap_2d::Costmap2DROS>(node);
  if (!owner) {
    throw std::runtime_error("FootprintUpdateLayer requires a Costmap2DROS owner");
  }
  owner_ = owner;
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  subscription_ = node->create_subscription<geometry_msgs::msg::Polygon>(
    "footprint_request", rclcpp::QoS(1),
    std::bind(&FootprintUpdateLayer::queueFootprint, this, std::placeholders::_1), options);
  current_ = true;
  enabled_ = true;
}

void FootprintUpdateLayer::queueFootprint(const geometry_msgs::msg::Polygon::SharedPtr polygon)
{
  if (polygon->points.size() < 3 || polygon->points.size() > 128) {
    return;
  }
  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(polygon->points.size());
  double area = 0.;
  for (std::size_t i = 0; i < polygon->points.size(); ++i) {
    const auto & p = polygon->points[i];
    const auto & next = polygon->points[(i + 1) % polygon->points.size()];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      return;
    }
    geometry_msgs::msg::Point point;
    point.x = p.x; point.y = p.y;
    points.push_back(point);
    area += static_cast<double>(p.x) * next.y - static_cast<double>(next.x) * p.y;
  }
  if (std::abs(area) < 1e-9) {
    return;
  }
  std::lock_guard<std::mutex> lock(pending_mutex_);
  pending_ = std::move(points);
}

void FootprintUpdateLayer::updateBounds(
  double, double, double, double *, double *, double *, double *)
{
  std::optional<std::vector<geometry_msgs::msg::Point>> next;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    next.swap(pending_);
  }
  if (next) {
    if (auto owner = owner_.lock()) {
      // LayeredCostmap::updateMap already holds this recursive mutex. Taking it
      // explicitly also protects direct calls and documents the reader contract.
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
        *owner->getCostmap()->getMutex());
      owner->setRobotFootprint(*next);
    }
  }
}

}  // namespace real_bringup

PLUGINLIB_EXPORT_CLASS(real_bringup::FootprintUpdateLayer, nav2_costmap_2d::Layer)
