"""
完整的仿真启动文件：集成Gazebo、机器人、传感器和导航
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    """生成launch描述"""
    
    # 声明参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间'
    )
    
    world_arg = DeclareLaunchArgument(
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
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='是否启动RViz2'
    )
    
    interface_type_arg = DeclareLaunchArgument(
        'interface_type',
        default_value='sim',
        description='传感器接口类型: sim 或 real'
    )
    
    # 获取参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    interface_type = LaunchConfiguration('interface_type')
    
    # 查找包路径
    vacuum_robot_sim_dir = FindPackageShare('vacuum_robot_sim')
    sensor_interface_dir = FindPackageShare('sensor_interface')
    
    # 启动Gazebo
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
    
    # 发布机器人描述
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            vacuum_robot_sim_dir,
            '/launch/robot_description.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # 在Gazebo中生成机器人
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
    
    # 启动导航栈
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            vacuum_robot_sim_dir,
            '/launch/navigation.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # 启动RViz2（可选）
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

