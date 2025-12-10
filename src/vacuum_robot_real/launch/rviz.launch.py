"""
启动 RViz2 可视化的 launch 文件

该文件负责启动 RViz2 可视化工具，用于：
- 可视化机器人模型和 TF 变换
- 显示传感器数据（激光扫描、点云等）
- 显示地图和导航路径
- 显示机器人状态和调试信息

@file rviz.launch.py
@brief RViz2 可视化启动文件
@author vacuum_robot_real
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    生成 RViz2 启动描述
    
    配置并启动 RViz2 可视化工具，加载预定义的配置文件。
    RViz2 是 ROS 2 的主要可视化工具，用于调试和监控机器人系统。
    
    @returns {LaunchDescription} 包含 RViz2 配置的 LaunchDescription 对象
    """
    
    # ========== 声明 Launch 参数 ==========
    
    # 仿真时间参数
    # @param {string} use_sim_time - 'true' 使用仿真时间，'false' 使用系统时间
    # 实机环境默认使用系统时间
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间'
    )
    
    # RViz 配置文件路径参数
    # @param {string} rviz_config - RViz2 配置文件的完整路径（.rviz 文件）
    # 配置文件定义了 RViz2 的显示设置、话题订阅、TF 显示等
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('vacuum_robot_real'),
            'rviz',
            'robot_navigation.rviz'
        ]),
        description='RViz配置文件路径'
    )
    
    # ========== 获取 Launch 配置参数 ==========
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # ========== 启动 RViz2 节点 ==========
    # RViz2 是 ROS 2 的 3D 可视化工具
    # 
    # 功能：
    # - 可视化机器人模型（从 /robot_description 话题）
    # - 显示 TF 坐标系变换树
    # - 显示传感器数据（激光扫描、点云、图像等）
    # - 显示地图（占用栅格地图、代价地图等）
    # - 显示导航路径和规划结果
    # - 提供交互式标记和工具
    #
    # 参数说明：
    # - '-d': 指定配置文件路径，RViz2 将加载该配置
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],  # 加载配置文件
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        rviz_config_arg,
        rviz_node,
    ])


