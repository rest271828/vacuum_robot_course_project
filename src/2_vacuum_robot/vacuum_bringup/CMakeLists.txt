cmake_minimum_required(VERSION 3.8)
project(vacuum_bringup)

# Default to C99 / C++14 (not strictly needed, but common)
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

# Install launch and config files
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Optional: if you later add rviz config folder, this will install it too
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rviz")
  install(
    DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
