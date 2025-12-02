"""
生成并发布机器人描述（robot_description）的 launch 文件

该文件负责：
- 加载机器人的 URDF/Xacro 描述文件
- 启动 robot_state_publisher 节点，发布机器人状态到 /robot_description 话题
- 启动 joint_state_publisher 节点，发布关节状态（用于 Gazebo 仿真）

@file robot_description.launch.py
@brief 机器人描述发布启动文件
@author vacuum_robot_sim
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    """
    生成机器人描述发布启动描述
    
    配置并启动 robot_state_publisher 和 joint_state_publisher 节点，
    将机器人的 URDF/Xacro 描述文件转换为 ROS 2 话题和 TF 变换。
    
    @returns {LaunchDescription} 包含机器人描述发布配置的 LaunchDescription 对象
    """
    
    # ========== 声明 Launch 参数 ==========
    
    # 仿真时间参数
    # @param {string} use_sim_time - 'true' 使用仿真时间，'false' 使用系统时间
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间'
    )
    
    # ========== 获取 Launch 配置参数 ==========
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # ========== 机器人描述文件路径 ==========
    # 获取机器人 URDF/Xacro 描述文件的完整路径
    # 该文件定义了机器人的物理结构、关节、传感器等
    robot_description_path = PathJoinSubstitution([
        FindPackageShare('vacuum_robot_sim'),
        'urdf',
        'robot_gazebo.xacro'
    ])
    
    # ========== 机器人状态发布器节点 ==========
    # robot_state_publisher 节点负责：
    # - 读取 URDF/Xacro 文件（通过 xacro 命令解析）
    # - 发布机器人描述到 /robot_description 话题
    # - 根据关节状态发布所有坐标系之间的 TF 变换
    # 
    # 该节点是 ROS 2 中机器人描述系统的核心组件
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # 使用 Command 执行 xacro 命令解析 Xacro 文件
            'robot_description': Command([
                'xacro ',
                robot_description_path
            ])
        }]
    )
    
    # ========== 关节状态发布器节点 ==========
    # joint_state_publisher 节点负责：
    # - 发布机器人的关节状态到 /joint_states 话题
    # - 在 Gazebo 仿真中，通常发布 "fake" 关节状态（固定值或零值）
    # - robot_state_publisher 订阅 /joint_states，据此计算并发布 TF 变换
    #
    # 注意：在真实机器人上，关节状态通常由硬件驱动节点发布
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher,
        joint_state_publisher,
    ])

