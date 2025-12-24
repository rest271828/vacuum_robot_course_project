cmake_minimum_required(VERSION 3.8)
project(vacuum_robot_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖（仅需要ament_cmake用于安装文件）
find_package(ament_cmake REQUIRED)

# 安装URDF文件
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# 安装世界文件
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# 安装RViz配置
install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# 安装配置文件
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 安装launch文件
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装Python脚本到lib目录（作为可执行文件）
install(
  PROGRAMS scripts/new_nav2_controller.py
            scripts/coverage_controller_test01.py
            scripts/coverage_controller_test02.py
  DESTINATION lib/${PROJECT_NAME}
)

# 暂时禁用测试以简化编译
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()

