cmake_minimum_required(VERSION 3.8)
project(real_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(bondcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)
add_executable(lifecycle_manager_resilient src/lifecycle_manager_resilient.cpp)
target_include_directories(lifecycle_manager_resilient PRIVATE ${nav2_lifecycle_manager_INCLUDE_DIRS})
ament_target_dependencies(lifecycle_manager_resilient rclcpp nav2_util nav2_msgs std_srvs bondcpp diagnostic_updater)
install(TARGETS lifecycle_manager_resilient DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
install(DIRECTORY behavior_trees config launch rviz
DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/navigation_recovery.py scripts/accumulated_scan.py scripts/spin_navigation_adapter.py scripts/sim_can_bridge.py scripts/frontier_explorer.py scripts/frontier_planner.py DESTINATION lib/${PROJECT_NAME})

ament_package()
