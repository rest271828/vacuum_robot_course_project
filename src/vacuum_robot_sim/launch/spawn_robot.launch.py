"""
在Gazebo中生成机器人的launch文件
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成launch描述"""
    
    # 声明参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间'
    )
    
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='机器人初始x坐标'
    )
    
    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='机器人初始y坐标'
    )
    
    z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.1',
        description='机器人初始z坐标'
    )
    
    # 获取参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    
    # 机器人描述文件路径
    robot_description_path = PathJoinSubstitution([
        FindPackageShare('vacuum_robot_sim'),
        'urdf',
        'robot_gazebo.xacro'
    ])
    
    # Gazebo spawn模型服务
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-entity', 'vacuum_robot',
            '-topic', '/robot_description',
            '-x', x,
            '-y', y,
            '-z', z,
            '-robot_namespace', '/'
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

