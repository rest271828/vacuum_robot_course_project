#!/usr/bin/env python3
"""
启动串口驱动节点
发布 /odom 里程计数据，订阅 /cmd_vel 速度指令
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 声明参数
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB1',
        description='串口设备路径'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='9600',
        description='串口波特率'
    )
    
    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.20',
        description='轮距（米）'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间'
    )

    # 串口驱动节点
    robot_serial_driver_node = Node(
        package='vacuum_robot_real',
        executable='robot_serial_driver.py',
        name='robot_serial_driver',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen'
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        wheel_base_arg,
        use_sim_time_arg,
        robot_serial_driver_node,
    ])

