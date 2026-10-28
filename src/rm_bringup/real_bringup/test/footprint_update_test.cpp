#include <atomic>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

#include "real_bringup/footprint_update_layer.hpp"
#include "nav2_util/node_utils.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using Polygon = geometry_msgs::msg::Polygon;
using real_bringup::FootprintUpdateLayer;

void require(bool value, const char * message)
{
  if (!value) {throw std::runtime_error(message);}
}

int main(int argc, char ** argv)
{
  const char * domain = std::getenv("ROS_DOMAIN_ID");
  require(domain && std::stoi(domain) >= 100, "Use an isolated ROS_DOMAIN_ID >= 100");
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<nav2_costmap_2d::Costmap2DROS>("guarded_costmap");
    node->set_parameter(rclcpp::Parameter("plugins",
      std::vector<std::string>{"footprint_update", "inflation_layer"}));
    node->declare_parameter("footprint_update.plugin", "real_bringup::FootprintUpdateLayer");
    node->declare_parameter("inflation_layer.plugin", "nav2_costmap_2d::InflationLayer");
    node->set_parameter(rclcpp::Parameter("footprint",
      "[[-0.3365,-0.283],[0.3365,-0.283],[0.3365,0.283],[-0.3365,0.283]]"));
    node->set_parameter(rclcpp::Parameter("footprint_padding", 0.0));
    require(node->configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      "Costmap plugin configuration failed");
    auto layer = std::dynamic_pointer_cast<FootprintUpdateLayer>(
      node->getLayeredCostmap()->getPlugins()->at(0));
    require(static_cast<bool>(layer), "Guard plugin was not loaded");
    auto rectangle = std::make_shared<Polygon>();
    for (const auto & xy : std::vector<std::pair<float, float>>{
      {-.3365f, -.283f}, {.3365f, -.283f}, {.3365f, .283f}, {-.3365f, .283f}})
    {
      geometry_msgs::msg::Point32 point;
      point.x = xy.first; point.y = xy.second;
      rectangle->points.push_back(point);
    }
    auto circle = std::make_shared<Polygon>();
    for (int i = 0; i < 32; ++i) {
      geometry_msgs::msg::Point32 point;
      point.x = .473 * std::cos(i * 2 * M_PI / 32);
      point.y = .473 * std::sin(i * 2 * M_PI / 32);
      circle->points.push_back(point);
    }
    auto update = [&]() {node->getLayeredCostmap()->updateMap(0., 0., 0.);};
    layer->queueFootprint(circle);
    require(node->getRobotFootprint().size() == 4,
      "A receive callback changed the live footprint before map update");
    update();
    require(node->getRobotFootprint().size() == 32, "Queued circle did not apply");
    layer->queueFootprint(rectangle); update();
    require(node->getRobotFootprint().size() == 4, "Queued rectangle did not apply");
    auto invalid = std::make_shared<Polygon>(*circle);
    invalid->points[0].x = std::numeric_limits<float>::quiet_NaN();
    layer->queueFootprint(invalid); update();
    require(node->getRobotFootprint().size() == 4, "Invalid polygon replaced the footprint");
    std::atomic<bool> start{false}, failed{false};
    std::thread writer([&]() {
      while (!start) {std::this_thread::yield();}
      for (int i = 0; i < 20000; ++i) {
        layer->queueFootprint(i % 2 ? circle : rectangle);
        if (i % 20 == 0) {std::this_thread::yield();}
      }
    });
    std::thread reader([&]() {
      while (!start) {std::this_thread::yield();}
      for (int i = 0; i < 50000; ++i) {
        // Same lock contract as MPPI::computeVelocityCommands.
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
          *node->getCostmap()->getMutex());
        auto footprint = node->getRobotFootprint();
        if (footprint.size() != 4 && footprint.size() != 32) {failed = true; break;}
        for (const auto & point : footprint) {
          const bool valid = footprint.size() == 4 ?
            std::abs(std::abs(point.x) - .3365) < 1e-6 &&
            std::abs(std::abs(point.y) - .283) < 1e-6 :
            std::abs(std::hypot(point.x, point.y) - .473) < 1e-6;
          if (!valid) {failed = true;}
        }
      }
    });
    start = true;
    for (int i = 0; i < 10000; ++i) {update();}
    writer.join(); reader.join();
    require(!failed, "Controller observed a torn footprint during concurrent updates");
    node->cleanup();
    std::cout << "PASS queued-only callback, polygon validation, 20000 requests, "
      "10000 map updates and 50000 coherent controller snapshots\n";
  }
  rclcpp::shutdown();
}
