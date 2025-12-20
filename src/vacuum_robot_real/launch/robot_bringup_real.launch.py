#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # ========== 设置环境变量，解决 AMCL 和 RViz2 消息队列溢出问题 ==========
    # 增加消息过滤器队列大小，避免 "queue is full" 错误
    set_message_filter_queue = SetEnvironmentVariable(
        'RVIZ_MESSAGE_FILTER_QUEUE_SIZE', '1000'  # 默认 100，增加到 1000
    )
    set_tf2_queue = SetEnvironmentVariable(
        'TF2_ROS_MESSAGE_FILTER_QUEUE_SIZE', '1000'  # TF2 消息过滤器队列大小
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    pkg_vacuum = FindPackageShare('vacuum_robot_real')

    # 1) 机器人描述
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_vacuum, '/launch/robot_description_real.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # 2) 串口驱动节点（发布 /odom，订阅 /cmd_vel）
    robot_serial_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_vacuum, '/launch/robot_serial_driver.launch.py'
        ]),
        launch_arguments={
            'port': '/dev/ttyUSB1',  # 根据实际串口设备修改
            'baudrate': '9600',
            'wheel_base': '0.20',  # 根据实际轮距修改
            'use_sim_time': use_sim_time,  # 传递时间参数
        }.items()
    )

    # 3) RPLidar 官方驱动
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rplidar_ros'), '/launch/rplidar_a1_launch.py'
        ]),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'laser_link',   # 和 robot_base_sim.xacro 里的 link 对齐
            'scan_mode': 'Standard',
            'use_sim_time': use_sim_time,  # 传递时间参数，确保时间戳同步
        }.items()
    )

    # 4) Nav2 导航栈（记得用"真机版本"的 nav2_params_real.yaml）
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_vacuum, '/launch/navigation.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # 建图模式：map 设置为空字符串 '' 表示运行 SLAM
            # 定位模式：map 设置为地图文件路径，例如 '/home/xxx/maps/room.yaml'

            # 'map': '/home/rest1/vacuum_robot/src/vacuum_robot_real/map/vacuum_robot.yaml',  # 定位模式（使用静态地图）
        }.items()
    )

    # 5) 你的 Nav2 控制器（自动发目标点的节点）
    # nav2_controller_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         pkg_vacuum, '/launch/nav2_controller.launch.py'
    #     ]),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #         'odom_topic': '/odom'
    #     }.items()
    # )

    return LaunchDescription([
        use_sim_time_arg,
        # 设置环境变量（必须在其他节点之前）
        set_message_filter_queue,
        set_tf2_queue,
        robot_description_launch,
        robot_serial_driver_launch,  # 串口驱动节点
        rplidar_launch,
        navigation_launch,
        #nav2_controller_launch,
    ])
