cmake_minimum_required(VERSION 3.8)
project(sensor_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 包含目录
include_directories(include)

# 创建库（接口头文件）
install(
  DIRECTORY include/
  DESTINATION include
)

# 可执行文件：激光雷达传感器节点
add_executable(laser_sensor_node
  src/laser_sensor_node.cpp
)

ament_target_dependencies(laser_sensor_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

# 安装可执行文件
install(TARGETS
  laser_sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

# 导出
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


