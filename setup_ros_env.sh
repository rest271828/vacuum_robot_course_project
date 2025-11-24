#!/bin/bash
# ROS2环境设置脚本 - 解决conda库冲突问题
#
# 注意：如果看到 "/bin/bash: .../libtinfo.so.6: no version information available" 警告
# 这是无害的，因为conda在~/.bashrc中初始化时设置了库路径。
# 脚本会清理这些路径，确保ROS2使用系统库。

# 退出conda环境（如果已激活）
if [ -n "$CONDA_DEFAULT_ENV" ]; then
    echo "正在退出conda环境: $CONDA_DEFAULT_ENV"
    conda deactivate 2>/dev/null || true
fi

# 首先完全移除conda库路径，避免库冲突
# 使用临时变量避免在清理过程中引用自身
CLEANED_LD_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v miniconda3 | grep -v conda | tr '\n' ':' | sed 's/:$//' | sed 's/^://')
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/usr/lib
if [ -n "$CLEANED_LD_PATH" ]; then
    export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$CLEANED_LD_PATH"
fi
unset CLEANED_LD_PATH

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

