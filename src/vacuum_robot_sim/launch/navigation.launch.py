"""
Nav2 导航栈启动文件

该文件启动完整的 Nav2 导航系统，包括：
- SLAM Toolbox：同步 SLAM 节点，负责实时建图和定位
- Nav2 导航栈：路径规划、路径跟踪、恢复行为等导航功能

在 SLAM 模式下，地图从话题动态订阅，而不是从静态文件加载。

@file navigation.launch.py
@brief Nav2 导航栈启动文件
@author vacuum_robot_sim
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    """
    生成 Nav2 导航栈启动描述
    
    配置并启动 SLAM Toolbox 和 Nav2 导航系统。
    在 SLAM 模式下，使用同步 SLAM 节点进行实时建图，
    同时启动 Nav2 导航栈进行路径规划和跟踪。
    
    @returns {LaunchDescription} 包含导航系统配置的 LaunchDescription 对象
    """
    
    # ========== 声明 Launch 参数 ==========
    
    # 仿真时间参数
    # @param {string} use_sim_time - 'true' 使用仿真时间，'false' 使用系统时间
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间'
    )
    
    # Nav2 参数文件路径
    # @param {string} params_file - Nav2 导航栈的 YAML 配置文件路径
    # 包含路径规划器、控制器、恢复行为等所有 Nav2 组件的参数
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('vacuum_robot_sim'),
            'config',
            'nav2_params.yaml'
        ]),
        description='Nav2参数文件路径'
    )
    
    # SLAM Toolbox 参数文件路径
    # @param {string} slam_params_file - SLAM Toolbox 的 YAML 配置文件路径
    # 包含建图算法、扫描匹配、回环检测等 SLAM 相关参数
    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('vacuum_robot_sim'),
            'config',
            'slam_toolbox.yaml'
        ]),
        description='Slam Toolbox参数文件路径'
    )
    
    # ========== 获取 Launch 配置参数 ==========
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    # ========== 1. SLAM Toolbox 节点 ==========
    # 同步 SLAM 节点，负责实时建图和定位
    # 
    # 功能：
    # - 订阅激光扫描数据（/scan）和里程计数据（/odom）
    # - 执行同步 SLAM 算法，构建地图并估计机器人位姿
    # - 发布地图到 /map 话题
    # - 发布 map->odom 的坐标变换（通过 tf2）
    #
    # 注意：在 SLAM 模式下，map->odom 变换由 SLAM 节点提供，
    # 而不是由 AMCL（自适应蒙特卡洛定位）提供
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # ========== 2. Nav2 导航栈 ==========
    # 启动 Nav2 导航系统，负责路径规划和控制
    # 
    # 注意：
    # - 使用 navigation_launch.py 而不是 bringup_launch.py
    # - bringup_launch.py 通常包含 AMCL（定位），但我们已经用 SLAM 定位了
    # - 因此只需要导航部分：路径规划、路径跟踪、恢复行为等
    #
    # 参数说明：
    # - 'map': 空字符串表示 SLAM 模式，从 /map 话题订阅地图，不加载静态地图文件
    # - 'autostart': 'true' 自动激活所有生命周期节点
    # - 'use_composition': 'False' 简单模式，不使用组件容器，每个节点独立运行
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',  # 自动激活生命周期节点
            'use_composition': 'False',  # 简单模式，不使用组件容器
            'map': '',  # SLAM模式：空字符串表示从话题订阅地图，不加载静态地图文件
        }.items()
    )

    # ========== 返回 Launch 描述 ==========
    # 将所有参数声明和节点配置组合成完整的 LaunchDescription
    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        slam_params_file_arg,
        slam_toolbox,
        nav2_navigation_launch,  # 必须包含 Nav2 导航栈启动配置
    ])