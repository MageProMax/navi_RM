cmake_minimum_required(VERSION 3.8)
project(rm_bnrobot_function)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake_python REQUIRED)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs)
find_package(lifecycle_msgs)
find_package(nav2_util REQUIRED) 
find_package(rm_interfaces REQUIRED) 
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(OpenCV)
find_package(interface REQUIRED)

add_executable(nav_pose src/nav_pose.cpp)
ament_target_dependencies(nav_pose rm_interfaces rclcpp)
ament_target_dependencies(nav_pose rclcpp nav2_msgs rclcpp_action tf2_ros std_msgs std_srvs)

add_executable(displayPose src/displayPose.cpp)
ament_target_dependencies(displayPose rm_interfaces rclcpp)
target_link_libraries(displayPose Qt5::Widgets)

add_executable(init_status src/init_status.cpp)
ament_target_dependencies(init_status rm_interfaces interface rclcpp)


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

install(TARGETS nav_pose displayPose init_status
DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  src/nav_client_ui.py src/init_pose.py src/get_pose.py src/pcd2map.py src/add_normal.py
  DESTINATION lib/${PROJECT_NAME} )


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
