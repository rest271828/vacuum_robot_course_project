"""
完整的仿真启动文件：集成Gazebo、机器人、传感器和导航

该文件是真空机器人仿真的主启动文件，负责协调启动以下组件：
- Gazebo 仿真环境
- 机器人模型描述和状态发布
- 在 Gazebo 中生成机器人实例
- Nav2 导航栈（包括 SLAM 和路径规划）
- RViz2 可视化工具（可选）

@file complete_simulation.launch.py
@brief 完整的仿真系统启动文件
@author vacuum_robot_sim
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    """
    生成完整的仿真系统 launch 描述
    
    该函数创建并配置所有必要的 launch 参数和节点，包括：
    - 仿真时间配置
    - Gazebo 世界文件路径
    - GUI 和可视化选项
    - 传感器接口类型
    
    @returns {LaunchDescription} 包含所有启动配置的 LaunchDescription 对象
    """
    
    # ========== 声明 Launch 参数 ==========
    
    # 仿真时间参数：控制是否使用 Gazebo 的仿真时间而不是系统时间
    # @param {string} use_sim_time - 'true' 使用仿真时间，'false' 使用系统时间
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间'
    )
    
    # Gazebo 世界文件路径参数：指定要加载的 Gazebo 世界文件
    # @param {string} world - Gazebo .world 文件的完整路径
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('vacuum_robot_sim'),
            'worlds',
            'simple_world.world'
        ]),
        description='Gazebo世界文件路径'
    )
    
    # Gazebo GUI 参数：控制是否显示 Gazebo 图形界面
    # @param {string} gui - 'true' 显示 GUI，'false' 无头模式运行
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='是否启动Gazebo GUI'
    )
    
    # RViz2 可视化参数：控制是否启动 RViz2 进行可视化
    # @param {string} rviz - 'true' 启动 RViz2，'false' 不启动
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='是否启动RViz2'
    )
    
    # 传感器接口类型参数：指定使用仿真传感器还是真实传感器
    # @param {string} interface_type - 'sim' 使用仿真传感器，'real' 使用真实传感器
    interface_type_arg = DeclareLaunchArgument(
        'interface_type',
        default_value='sim',
        description='传感器接口类型: sim 或 real'
    )
    
    # ========== 获取 Launch 配置参数 ==========
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    interface_type = LaunchConfiguration('interface_type')
    
    # ========== 查找 ROS 2 包路径 ==========
    vacuum_robot_sim_dir = FindPackageShare('vacuum_robot_sim')
    sensor_interface_dir = FindPackageShare('sensor_interface')
    
    # ========== 启动 Gazebo 仿真环境 ==========
    # 启动 Gazebo 服务器和客户端，加载指定的世界文件
    # @see gazebo.launch.py
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            vacuum_robot_sim_dir,
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={
            'world': world,
            'gui': gui
        }.items()
    )
    
    # ========== 发布机器人描述 ==========
    # 启动 robot_state_publisher 和 joint_state_publisher
    # 发布机器人的 URDF/Xacro 描述到 /robot_description 话题
    # @see robot_description.launch.py
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            vacuum_robot_sim_dir,
            '/launch/robot_description.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # ========== 在 Gazebo 中生成机器人模型 ==========
    # 调用 Gazebo 的 spawn_entity 服务，将机器人模型添加到仿真世界
    # @param {float} x - 机器人初始 x 坐标（米）
    # @param {float} y - 机器人初始 y 坐标（米）
    # @param {float} z - 机器人初始 z 坐标（米，通常略高于地面）
    # @see spawn_robot.launch.py
    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            vacuum_robot_sim_dir,
            '/launch/spawn_robot.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x': '0.0',
            'y': '0.0',
            'z': '0.1'
        }.items()
    )
    
    # ========== 启动 Nav2 导航栈 ==========
    # 启动 SLAM Toolbox 和 Nav2 导航系统
    # 包括：建图、定位、路径规划、路径跟踪等功能
    # @see navigation.launch.py
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            vacuum_robot_sim_dir,
            '/launch/navigation.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # ========== 启动 RViz2 可视化工具（可选） ==========
    # 根据 rviz 参数决定是否启动 RViz2
    # RViz2 用于可视化机器人状态、传感器数据、地图、路径等
    # @see rviz.launch.py
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            vacuum_robot_sim_dir,
            '/launch/rviz.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(PythonExpression(["'", rviz, "' == 'true'"]))
    )
    
    # ========== 返回 Launch 描述 ==========
    # 将所有参数声明和启动配置组合成完整的 LaunchDescription
    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        gui_arg,
        rviz_arg,
        interface_type_arg,
        gazebo_launch,
        robot_description_launch,
        spawn_robot_launch,
        navigation_launch,
        rviz_launch,
    ])

