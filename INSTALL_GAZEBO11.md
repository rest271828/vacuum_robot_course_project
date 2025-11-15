# 安装Gazebo 11指南

## 问题说明

在添加Gazebo仓库密钥时，`apt-key` 命令已被弃用。本指南使用新的密钥管理方式。

## 快速安装（推荐）

运行提供的安装脚本：

```bash
cd /home/rest1/junior_ws
./install_gazebo11.sh
```

## 手动安装步骤

### 1. 安装必要的工具

```bash
sudo apt-get update
sudo apt-get install -y wget gnupg
```

### 2. 添加Gazebo仓库（使用新方式）

**重要：不再使用 `apt-key`，改用新的密钥管理方式**

```bash
# 创建密钥目录
sudo mkdir -p /etc/apt/keyrings

# 下载并添加密钥到新位置
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo tee /etc/apt/keyrings/gazebo.gpg > /dev/null

# 设置密钥权限
sudo chmod a+r /etc/apt/keyrings/gazebo.gpg

# 获取系统版本代号（通常是jammy）
OS_VERSION=$(lsb_release -cs)

# 添加仓库源（使用新的signed-by格式）
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/gazebo.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $OS_VERSION main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
```

### 3. 更新软件包列表

```bash
sudo apt-get update
```

### 4. 安装Gazebo 11

```bash
sudo apt-get install -y \
    gazebo11 \
    libgazebo11-dev \
    gazebo11-plugin-base \
    gazebo11-plugins \
    libgazebo11
```

### 5. 安装ROS2 Gazebo包

```bash
# 确保ROS2环境已设置
source /opt/ros/humble/setup.bash

# 安装ROS2 Gazebo包
sudo apt-get install -y \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-msgs \
    ros-humble-gazebo-plugins
```

## 验证安装

```bash
# 检查gazebo命令
gazebo --version

# 检查ROS2包
source /opt/ros/humble/setup.bash
ros2 pkg list | grep gazebo
```

## 故障排除

### 如果gazebo命令找不到

```bash
# 刷新命令缓存
hash -r

# 或重新打开终端
```

### 如果遇到密钥错误

确保使用了新的密钥位置（`/etc/apt/keyrings/`）而不是旧的 `apt-key` 方式。

### 如果仓库无法访问

检查网络连接，或尝试使用镜像源。

## 新密钥方式 vs 旧方式

### ❌ 旧方式（已弃用）
```bash
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

### ✅ 新方式（推荐）
```bash
sudo mkdir -p /etc/apt/keyrings
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo tee /etc/apt/keyrings/gazebo.gpg > /dev/null
sudo chmod a+r /etc/apt/keyrings/gazebo.gpg
# 然后在sources.list中使用 signed-by=/etc/apt/keyrings/gazebo.gpg
```

## 安装完成后

```bash
# 设置ROS2环境
source /home/rest1/junior_ws/setup_ros_env.sh

# 启动仿真
ros2 launch vacuum_robot_sim complete_simulation.launch.py
```

