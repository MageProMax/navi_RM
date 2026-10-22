cmake_minimum_required(VERSION 3.8)
project(pgo)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(interface REQUIRED)
find_package(yaml-cpp REQUIRED)



include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

set(SRC_LIST src/pgos/commons.cpp
             src/pgos/simple_pgo.cpp
             src/pgos/static_map_filter.cpp)

add_library(pgo_backend STATIC ${SRC_LIST})
target_link_libraries(pgo_backend ${PCL_LIBRARIES} gtsam)
add_executable(pgo_node src/pgo_node.cpp)
ament_target_dependencies(pgo_node rclcpp std_msgs sensor_msgs nav_msgs message_filters pcl_conversions tf2_ros geometry_msgs visualization_msgs interface GTSAM)
target_link_libraries(pgo_node pgo_backend ${PCL_LIBRARIES} gtsam yaml-cpp)


add_executable(online_pgo_node src/online_pgo_node.cpp)
ament_target_dependencies(online_pgo_node rclcpp std_msgs sensor_msgs nav_msgs message_filters pcl_conversions tf2_ros geometry_msgs interface)
target_link_libraries(online_pgo_node pgo_backend ${PCL_LIBRARIES} gtsam yaml-cpp)

add_executable(static_map_filter src/static_map_filter_main.cpp)
ament_target_dependencies(static_map_filter nav_msgs)
target_link_libraries(static_map_filter pgo_backend ${PCL_LIBRARIES} gtsam yaml-cpp)

install(TARGETS static_map_filter online_pgo_node pgo_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  enable_testing()
  add_executable(online_mapping_test test/online_mapping_test.cpp)
  target_include_directories(online_mapping_test PRIVATE src)
  ament_target_dependencies(online_mapping_test nav_msgs)
  target_link_libraries(online_mapping_test pgo_backend ${PCL_LIBRARIES} gtsam)
  add_test(NAME online_mapping_test COMMAND online_mapping_test)
  add_executable(static_map_filter_test test/static_map_filter_test.cpp)
  target_include_directories(static_map_filter_test PRIVATE src)
  ament_target_dependencies(static_map_filter_test nav_msgs)
  target_link_libraries(static_map_filter_test pgo_backend ${PCL_LIBRARIES} gtsam yaml-cpp)
  add_test(NAME static_map_filter_test COMMAND static_map_filter_test)
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

ament_package()
