# 扫地机器人仿真导航项目

这是一个基于ROS2的扫地机器人导航仿真项目，支持Gazebo仿真环境和实机部署，具有模块化的传感器接口设计。

## 项目结构

```
junior_ws/
├── src/
│   ├── vacuum_robot_sim/          # 机器人仿真包
│   │   ├── urdf/                   # 机器人URDF/Xacro模型
│   │   ├── worlds/                 # Gazebo世界文件
│   │   ├── launch/                 # Launch启动文件
│   │   ├── rviz/                   # RViz2配置文件
│   │   └── config/                 # Nav2和slam_toolbox配置文件
│   └── sensor_interface/           # 模块化传感器接口包
│       ├── include/                # 头文件
│       └── src/                    # 源代码文件
└── README.md
```

## 功能特性

- ✅ Gazebo仿真环境搭建
- ✅ 二维激光雷达传感器（仿真）
- ✅ RViz2数据可视化
- ✅ Nav2导航栈集成
- ✅ SLAM_Toolbox建图
- ✅ 模块化传感器接口（支持仿真/实机切换）

## 依赖要求

### ROS2包依赖
- `ros-humble-desktop` 或更高版本
- `gazebo-ros-pkgs`
- `nav2-bringup`
- `slam-toolbox`
- `robot-state-publisher`
- `joint-state-publisher`
- `xacro`

### 安装依赖

```bash
# Ubuntu/Debian系统
sudo apt update
sudo apt install -y \
  gazebo11 \
  libgazebo11-dev \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-msgs \
  ros-humble-gazebo-plugins \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-rviz2
```

**注意**：如果遇到 `FileNotFoundError: 'gazebo'` 错误，说明缺少Gazebo Classic。可以使用提供的安装脚本：
```bash
./install_gazebo.sh
```
或参考 `INSTALL_GAZEBO.md` 进行手动安装。

## 环境设置（重要！）

**如果系统安装了conda/miniconda，需要先设置环境以避免库冲突：**

```bash
# 使用提供的环境设置脚本（推荐）
source /home/rest1/junior_ws/setup_ros_env.sh
```

或者手动设置：
```bash
# 退出conda环境
conda deactivate

# 设置库路径，让系统库优先
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/usr/lib:$LD_LIBRARY_PATH
export PATH=/usr/bin:/usr/local/bin:$(echo $PATH | tr ':' '\n' | grep -v miniconda3 | tr '\n' ':' | sed 's/:$//')
export PYTHON_EXECUTABLE=/usr/bin/python3

# Source ROS2
source /opt/ros/humble/setup.bash
```

## 编译项目

```bash
cd /home/rest1/junior_ws
source setup_ros_env.sh  # 或手动设置环境
colcon build --symlink-install
source install/setup.bash
```

## 使用方法

### 1. 启动完整仿真环境

**重要：先设置ROS2环境！**

```bash
# 设置环境（解决conda冲突）
source /home/rest1/junior_ws/setup_ros_env.sh

# 启动完整仿真
ros2 launch vacuum_robot_sim complete_simulation.launch.py
```

这将启动Gazebo、机器人模型、导航栈和RViz2。

### 2. 分别启动各个组件

#### 启动Gazebo仿真环境
```bash
ros2 launch vacuum_robot_sim gazebo.launch.py
```

#### 发布机器人描述
```bash
ros2 launch vacuum_robot_sim robot_description.launch.py
```

#### 在Gazebo中生成机器人
```bash
ros2 launch vacuum_robot_sim spawn_robot.launch.py
```

#### 启动导航栈
```bash
ros2 launch vacuum_robot_sim navigation.launch.py
```

#### 启动RViz2可视化
```bash
ros2 launch vacuum_robot_sim rviz.launch.py
```

### 3. 传感器接口切换

传感器接口支持仿真和实机模式切换：

#### 仿真模式（默认）
```bash
ros2 run sensor_interface laser_sensor_node --ros-args -p interface_type:=sim
```

#### 实机模式
```bash
ros2 run sensor_interface laser_sensor_node --ros-args -p interface_type:=real
```

## 主要话题

- `/scan` - 激光雷达扫描数据
- `/odom` - 里程计数据
- `/cmd_vel` - 速度控制命令
- `/map` - 地图数据（SLAM生成）
- `/tf` / `/tf_static` - 坐标变换

## 导航操作

### 建图模式
1. 启动完整仿真：`ros2 launch vacuum_robot_sim complete_simulation.launch.py`
2. 在RViz2中使用"2D Nav Goal"工具设置目标点
3. 机器人将开始导航并同时建图
4. 保存地图：
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

### 导航模式
1. 使用已保存的地图启动导航：
```bash
ros2 launch nav2_bringup navigation_launch.py params_file:=/path/to/nav2_params.yaml map:=/path/to/map.yaml
```

## 模块化传感器接口

项目采用模块化设计，传感器接口可以在仿真和实机之间轻松切换：

### 接口设计
- `LaserInterface` - 抽象基类
- `SimLaserInterface` - 仿真接口实现（从Gazebo订阅数据）
- `RealLaserInterface` - 实机接口实现（预留接口，需根据硬件实现）

### 扩展传感器接口

要添加新的传感器或实现实机接口，请：

1. 实现`LaserInterface`接口类
2. 在`sensor_interface`包中注册新接口
3. 通过参数选择使用哪个接口

## 配置说明

### Nav2配置
配置文件位于：`vacuum_robot_sim/config/nav2_params.yaml`

主要配置项：
- 局部/全局代价地图
- 路径规划器
- 控制器
- 恢复行为

### SLAM_Toolbox配置
配置文件位于：`vacuum_robot_sim/config/slam_toolbox.yaml`

主要配置项：
- 扫描匹配参数
- 闭环检测
- 地图分辨率

## 故障排除

### Gazebo无法启动
- 检查Gazebo是否已安装：`gazebo --version`
- 检查DISPLAY环境变量（GUI模式）

### 机器人模型未显示
- 检查URDF文件是否正确：`check_urdf $(rospack find vacuum_robot_sim)/urdf/robot_gazebo.xacro`
- 检查Gazebo插件是否正确加载

### 导航不工作
- 检查TF树：`ros2 run tf2_tools view_frames`
- 检查话题是否正常发布：`ros2 topic list`
- 检查地图是否发布：`ros2 topic echo /map`

## 许可证

MIT License

## 贡献

欢迎提交问题和拉取请求！

