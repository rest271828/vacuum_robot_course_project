"""
Nav2 控制节点启动文件 - 真机版本

该文件启动自定义的 Nav2 控制节点，提供以下功能：
- 读取机器人当前位置（从里程计话题）
- 接收导航目标点（通过 Nav2 Action 接口）
- 监控导航状态和结果
- 提供坐标转换和导航控制功能

@file nav2_controller.launch.py
@brief Nav2 控制节点启动文件
@author vacuum_robot_real
"""
from launch import LaunchDescription
from rclpy.qos import qos_profile_sensor_data
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    生成 Nav2 控制节点启动描述
    
    配置并启动 nav2_controller.py 节点，该节点作为 Nav2 导航系统的
    客户端，负责发送导航目标并监控导航状态。
    
    @returns {LaunchDescription} 包含 Nav2 控制节点配置的 LaunchDescription 对象
    """
    
    # ========== 声明 Launch 参数 ==========
    
    # 仿真时间参数
    # @param {string} use_sim_time - 'true' 使用仿真时间，'false' 使用系统时间
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间'
    )
    
    # 里程计话题名称参数
    # @param {string} odom_topic - 订阅的里程计话题名称，用于获取机器人位置
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='里程计话题名称'
    )
    
    # 机器人基座坐标系参数
    # @param {string} base_frame - 机器人基座坐标系名称，通常是 'base_link'
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='机器人基座坐标系'
    )
    
    # 全局坐标系参数
    # @param {string} global_frame - 全局坐标系名称，通常是 'map' 或 'odom'
    global_frame_arg = DeclareLaunchArgument(
        'global_frame',
        default_value='map',
        description='全局坐标系'
    )
    
    # ========== 获取 Launch 配置参数 ==========
    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_topic = LaunchConfiguration('odom_topic')
    base_frame = LaunchConfiguration('base_frame')
    global_frame = LaunchConfiguration('global_frame')
    
    # ========== 启动 Nav2 控制节点 ==========
    # 启动自定义的 Nav2 控制节点，该节点位于 scripts/new_nav2_controller.py
    # 
    # 节点功能：
    # - 订阅里程计话题获取机器人当前位置
    # - 通过 Nav2 Action 接口发送导航目标
    # - 监控导航状态和结果反馈
    # - 提供坐标转换功能
    nav2_controller_node = Node(
        package='vacuum_robot_real',
        executable='new_nav2_controller.py',
        name='nav2_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_topic': odom_topic,
            'base_frame': base_frame,
            'global_frame': global_frame,
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        odom_topic_arg,
        base_frame_arg,
        global_frame_arg,
        nav2_controller_node,
    ])




