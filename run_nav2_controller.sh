#!/bin/bash
# 直接运行Nav2控制节点的脚本
# 不依赖ros2 run，避免环境问题

# 设置ROS2环境（尽量清理conda干扰）
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/usr/lib
export PATH=/usr/bin:/usr/local/bin:$(echo $PATH | tr ':' '\n' | grep -v miniconda3 | tr '\n' ':' | sed 's/:$//')

# Source ROS2
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/vacuum_robot/install/setup.bash 2>/dev/null

# 直接运行Python脚本
python3 ~/vacuum_robot/install/vacuum_robot_sim/lib/vacuum_robot_sim/nav2_controller.py "$@"



