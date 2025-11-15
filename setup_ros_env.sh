#!/bin/bash
# ROS2环境设置脚本 - 解决conda库冲突问题

# 退出conda环境（如果已激活）
if [ -n "$CONDA_DEFAULT_ENV" ]; then
    echo "正在退出conda环境: $CONDA_DEFAULT_ENV"
    conda deactivate
fi

# 设置LD_LIBRARY_PATH，让系统库优先于conda库
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/usr/lib:$LD_LIBRARY_PATH

# 移除conda库路径
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | tr ':' '\n' | grep -v miniconda3 | tr '\n' ':' | sed 's/:$//')

# 设置PATH，让系统Python优先于conda Python
export PATH=/usr/bin:/usr/local/bin:$(echo $PATH | tr ':' '\n' | grep -v miniconda3 | tr '\n' ':' | sed 's/:$//')

# 设置Python可执行文件路径（用于CMake）
export PYTHON_EXECUTABLE=/usr/bin/python3

# Source ROS2环境
source /opt/ros/humble/setup.bash

# Source工作空间（如果已编译）
if [ -f install/setup.bash ]; then
    source install/setup.bash
fi

echo "ROS2环境已设置完成！"
echo "现在可以运行: ros2 launch vacuum_robot_sim complete_simulation.launch.py"

