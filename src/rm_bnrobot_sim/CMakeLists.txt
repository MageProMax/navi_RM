cmake_minimum_required(VERSION 3.8)
project(rm_bnrobot_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

install(DIRECTORY urdf meshes world scan_mode launch config
DESTINATION share/${PROJECT_NAME})

# Only complete model directories belong in Gazebo's model browser path.
install(DIRECTORY meshes/cafe_table meshes/RMUC2024_world
                  world/room world/room2 world/RMUL2024_world
  DESTINATION share/${PROJECT_NAME}/models)
install(DIRECTORY meshes/RMUL2024_world/meshes
  DESTINATION share/${PROJECT_NAME}/models/RMUL2024_world)
foreach(obstacle_number RANGE 1 12)
  install(DIRECTORY meshes/obstacles/obstacle${obstacle_number}
    DESTINATION share/${PROJECT_NAME}/models)
endforeach()

ament_package()
