"""
启动 Gazebo 仿真环境的 launch 文件

该文件负责启动 Gazebo 物理仿真引擎，包括：
- Gazebo 服务器（gazebo server）：执行物理仿真计算
- Gazebo 客户端（gazebo client）：提供图形界面（可选）
- ROS 2 集成插件：libgazebo_ros_init.so 和 libgazebo_ros_factory.so

@file gazebo.launch.py
@brief Gazebo 仿真环境启动文件
@author vacuum_robot_sim
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    生成 Gazebo 启动描述
    
    配置并启动 Gazebo 仿真环境，支持自定义世界文件和 GUI 选项。
    在启动前会尝试停用 conda 环境，避免环境冲突。
    
    @returns {LaunchDescription} 包含 Gazebo 启动配置的 LaunchDescription 对象
    """
    
    # ========== 声明 Launch 参数 ==========
    
    # Gazebo 世界文件路径参数
    # @param {string} world - Gazebo .world 文件的完整路径，定义仿真环境
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('vacuum_robot_sim'),
            'worlds',
            'simple_world.world'
        ]),
        description='Gazebo世界文件路径'
    )
    
    # Gazebo GUI 参数
    # @param {string} gui - 'true' 启动图形界面，'false' 无头模式运行
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='是否启动Gazebo GUI'
    )
    
    # ========== 获取 Launch 配置参数 ==========
    world_file = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    
    # ========== 启动 Gazebo 进程 ==========
    # 使用 ExecuteProcess 启动 Gazebo 可执行文件
    # 
    # 命令说明：
    # - 'conda deactivate 2>/dev/null || true': 尝试停用 conda 环境，避免环境冲突
    # - 'gazebo --verbose': 启动 Gazebo，verbose 模式输出详细日志
    # - '-s libgazebo_ros_init.so': 加载 ROS 2 初始化插件，建立 ROS 2 通信
    # - '-s libgazebo_ros_factory.so': 加载 ROS 2 工厂插件，支持通过服务生成模型
    # - world_file: 加载指定的世界文件
    #
    # 注意：cmd 列表中的第三个元素使用列表格式，ROS 2 会自动拼接字符串和 LaunchConfiguration
    gazebo_process = ExecuteProcess(
        cmd=[
            'bash', '-c',
            [
                #'conda deactivate 2>/dev/null || true; exec gazebo --verbose -s libgazebo_ros_factory.so ',
                'conda deactivate 2>/dev/null || true; exec gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so ',
                world_file
            ]
        ],
        output='screen'  # 输出到终端屏幕
    )
    
    return LaunchDescription([
        world_file_arg,
        gui_arg,
        gazebo_process,
    ])