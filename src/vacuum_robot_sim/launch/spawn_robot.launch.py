"""
在 Gazebo 中生成机器人的 launch 文件

该文件负责在 Gazebo 仿真环境中生成机器人模型实例。
它调用 Gazebo 的 spawn_entity 服务，将机器人模型添加到仿真世界。

@file spawn_robot.launch.py
@brief Gazebo 机器人生成启动文件
@author vacuum_robot_sim
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    生成 Gazebo 机器人生成启动描述
    
    配置并启动 gazebo_ros spawn_entity 节点，该节点通过调用
    Gazebo 服务将机器人模型添加到仿真世界中。
    
    @returns {LaunchDescription} 包含机器人生成配置的 LaunchDescription 对象
    """
    
    # ========== 声明 Launch 参数 ==========
    
    # 仿真时间参数
    # @param {string} use_sim_time - 'true' 使用仿真时间，'false' 使用系统时间
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间'
    )
    
    # 机器人初始 x 坐标参数
    # @param {float} x - 机器人在 Gazebo 世界中的初始 x 坐标（米）
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='机器人初始x坐标'
    )
    
    # 机器人初始 y 坐标参数
    # @param {float} y - 机器人在 Gazebo 世界中的初始 y 坐标（米）
    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='机器人初始y坐标'
    )
    
    # 机器人初始 z 坐标参数
    # @param {float} z - 机器人在 Gazebo 世界中的初始 z 坐标（米）
    # 通常设置为略高于地面（如 0.1 米），避免模型嵌入地面
    z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.1',
        description='机器人初始z坐标'
    )
    
    # ========== 获取 Launch 配置参数 ==========
    use_sim_time = LaunchConfiguration('use_sim_time')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    
    # ========== 机器人描述文件路径 ==========
    # 注意：虽然这里定义了路径，但实际使用的是 /robot_description 话题
    # 该话题应该由 robot_description.launch.py 发布的 robot_state_publisher 提供
    robot_description_path = PathJoinSubstitution([
        FindPackageShare('vacuum_robot_sim'),
        'urdf',
        'robot_gazebo.xacro'
    ])
    
    # ========== Gazebo 模型生成节点 ==========
    # gazebo_ros spawn_entity 节点负责：
    # - 从 /robot_description 话题读取机器人 URDF 描述
    # - 调用 Gazebo 的 spawn_entity 服务，将机器人模型添加到仿真世界
    # - 设置机器人的初始位置和姿态
    #
    # 参数说明：
    # - '-entity': 模型在 Gazebo 中的名称（实体名称）
    # - '-topic': 机器人描述话题名称，spawn_entity 从此话题读取 URDF
    # - '-x', '-y', '-z': 机器人初始位置坐标
    # - '-robot_namespace': ROS 2 命名空间，'/' 表示根命名空间
    #
    # 注意：此节点必须在以下节点之后启动：
    # 1. Gazebo 服务器已启动
    # 2. robot_state_publisher 已启动并发布 /robot_description
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-entity', 'vacuum_robot',  # Gazebo 中的模型名称
            '-topic', '/robot_description',  # 从话题读取机器人描述
            '-x', x,  # 初始 x 坐标
            '-y', y,  # 初始 y 坐标
            '-z', z,  # 初始 z 坐标
            '-robot_namespace', '/'  # ROS 2 命名空间
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        x_arg,
        y_arg,
        z_arg,
        spawn_entity,
    ])

