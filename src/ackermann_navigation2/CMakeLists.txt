cmake_minimum_required(VERSION 3.8)
project(ackermann_navigation2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------------------------------------------------
# 1. 查找依赖包
# -------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# -------------------------------------------------------------------
# 2. 编译设置
# -------------------------------------------------------------------
include_directories(${PCL_INCLUDE_DIRS})
add_executable(icp_relocalizer src/icp_relocalizer.cpp)

# 链接 PCL 库和 ament 依赖
target_link_libraries(icp_relocalizer ${PCL_LIBRARIES})

ament_target_dependencies(icp_relocalizer
  rclcpp 
  sensor_msgs 
  nav_msgs 
  geometry_msgs 
  tf2 
  tf2_geometry_msgs 
  pcl_conversions 
  pcl_ros
)

# -------------------------------------------------------------------
# 3. 安装指令
# -------------------------------------------------------------------
# 安装二进制文件到 lib 目录下，这样才能通过 ros2 run 启动
install(TARGETS
  icp_relocalizer
  DESTINATION lib/${PROJECT_NAME}
)

# 安装其他资源目录 
install(DIRECTORY
  launch
  config
  rviz
  maps
  params
  DESTINATION share/${PROJECT_NAME}
)

# -------------------------------------------------------------------
# 4. 测试与导出
# -------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
