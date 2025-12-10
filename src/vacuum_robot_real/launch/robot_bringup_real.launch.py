#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

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
            'scan_mode': 'Standard'
        }.items()
    )

    # 4) Nav2 导航栈（记得用“真机版本”的 nav2_params_real.yaml）
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_vacuum, '/launch/navigation.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # 如果用 AMCL + 现成地图，就给 map 传路径
            # 'map': '/home/xxx/maps/room.yaml'
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
        robot_description_launch,
        robot_serial_driver_launch,  # 串口驱动节点
        rplidar_launch,
        navigation_launch,
        #nav2_controller_launch,
    ])
