cmake_minimum_required(VERSION 3.8)
project(ackermann_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  config
  meshes
  worlds
  rviz
  maps
  models
  xacro
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
