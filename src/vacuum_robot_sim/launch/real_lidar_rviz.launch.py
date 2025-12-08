"""
真实雷达数据可视化启动文件

该文件用于启动真实 RPLidar 雷达节点并在 RViz 中显示数据。
适用于实机环境，不需要 Gazebo 仿真。

@file real_lidar_rviz.launch.py
@brief 真实雷达 RViz 可视化启动文件
@author vacuum_robot_sim
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    生成真实雷达可视化启动描述
    
    启动以下组件：
    - RPLidar 节点：从真实硬件读取雷达数据并发布到 /scan 话题
    - robot_state_publisher：发布机器人模型描述和 TF 变换
    - RViz2：可视化雷达数据和机器人模型
    
    @returns {LaunchDescription} 包含所有启动配置的 LaunchDescription 对象
    """
    
    # ========== 声明 Launch 参数 ==========
    
    # RPLidar 串口参数
    # @param {string} serial_port - 雷达连接的串口设备路径，默认 /dev/ttyUSB0
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='RPLidar 串口设备路径'
    )
    
    # RPLidar 波特率参数
    # @param {string} serial_baudrate - 串口波特率，不同型号雷达波特率不同
    # A1: 115200, A2/A3: 115200, S1: 256000, S2: 1000000, C1: 460800
    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='RPLidar 串口波特率'
    )
    
    # 雷达 frame_id 参数
    # @param {string} frame_id - 雷达的坐标系名称，需要与机器人 URDF 中的 laser_link 匹配
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_link',
        description='雷达坐标系名称（frame_id）'
    )
    
    # 是否反转扫描数据
    # @param {string} inverted - 'true' 或 'false'，是否反转扫描数据
    inverted_arg = DeclareLaunchArgument(
        'inverted',
        default_value='false',
        description='是否反转扫描数据'
    )
    
    # 是否启用角度补偿
    # @param {string} angle_compensate - 'true' 或 'false'，是否启用角度补偿
    angle_compensate_arg = DeclareLaunchArgument(
        'angle_compensate',
        default_value='true',
        description='是否启用角度补偿'
    )
    
    # 扫描模式参数
    # @param {string} scan_mode - 扫描模式，如 'Standard', 'Sensitivity', 'DenseBoost' 等
    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode',
        default_value='Standard',
        description='扫描模式'
    )
    
    # 是否启动 RViz2
    # @param {string} rviz - 'true' 启动 RViz2，'false' 不启动
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='是否启动 RViz2'
    )
    
    # RViz 配置文件路径
    # @param {string} rviz_config - RViz2 配置文件的完整路径
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('vacuum_robot_sim'),
            'rviz',
            'robot_navigation.rviz'
        ]),
        description='RViz配置文件路径'
    )
    
    # ========== 获取 Launch 配置参数 ==========
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')
    inverted = LaunchConfiguration('inverted')
    angle_compensate = LaunchConfiguration('angle_compensate')
    scan_mode = LaunchConfiguration('scan_mode')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # ========== 查找 ROS 2 包路径 ==========
    vacuum_robot_sim_dir = FindPackageShare('vacuum_robot_sim')
    
    # ========== 启动 RPLidar 节点 ==========
    # RPLidar 节点从真实硬件读取雷达数据并发布到 /scan 话题
    # 话题类型：sensor_msgs/msg/LaserScan
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': frame_id,
            'inverted': inverted,
            'angle_compensate': angle_compensate,
            'scan_mode': scan_mode,
            'topic_name': 'scan'  # 发布到 /scan 话题
        }]
    )
    
    # ========== 发布机器人描述 ==========
    # robot_state_publisher 发布机器人 URDF 描述和 TF 变换
    # 这样 RViz 才能显示机器人模型和正确的坐标系
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            vacuum_robot_sim_dir,
            '/launch/robot_description.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'false'  # 实机环境不使用仿真时间
        }.items()
    )
    
    # ========== 启动 RViz2 可视化工具（可选） ==========
    # 根据 rviz 参数决定是否启动 RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': False  # 实机环境不使用仿真时间
        }],
        condition=IfCondition(PythonExpression(["'", rviz, "' == 'true'"]))
    )
    
    # ========== 返回 Launch 描述 ==========
    return LaunchDescription([
        serial_port_arg,
        serial_baudrate_arg,
        frame_id_arg,
        inverted_arg,
        angle_compensate_arg,
        scan_mode_arg,
        rviz_arg,
        rviz_config_arg,
        rplidar_node,
        robot_description_launch,
        rviz_node,
    ])

