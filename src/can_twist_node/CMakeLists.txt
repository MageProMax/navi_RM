cmake_minimum_required(VERSION 3.8)
project(can_twist)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

include_directories(
  include 
  ${LIBUSB_INCLUDE_DIRS}
)

add_executable(can_twist_node 
  src/can_twist_node.cc 
  src/usb_can_v2.cc
)

target_compile_features(can_twist_node PRIVATE cxx_std_20)

ament_target_dependencies(can_twist_node 
  rclcpp 
  geometry_msgs
)
target_link_libraries(can_twist_node 
  ${LIBUSB_LIBRARIES}
)

install(TARGETS
  can_twist_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
