"""
启动Gazebo仿真环境的launch文件
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成launch描述"""
    
    # 声明参数
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('vacuum_robot_sim'),
            'worlds',
            'simple_world.world'
        ]),
        description='Gazebo世界文件路径'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='是否启动Gazebo GUI'
    )
    
    # 获取参数
    world_file = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    
    # 启动Gazebo
    # 默认总是使用factory（支持GUI和headless）
    gazebo_process = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_factory.so',
            world_file
        ],
        output='screen'
    )
    
    return LaunchDescription([
        world_file_arg,
        gui_arg,
        gazebo_process,
    ])

