"""
启动Nav2导航栈和slam_toolbox的launch文件
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """生成launch描述"""
    
    # 声明参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('vacuum_robot_sim'),
            'config',
            'nav2_params.yaml'
        ]),
        description='Nav2参数文件路径'
    )
    
    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('vacuum_robot_sim'),
            'config',
            'slam_toolbox.yaml'
        ]),
        description='Slam Toolbox参数文件路径'
    )
    
    # 获取参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    # 查找Nav2的launch文件
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )
    
    # 在SLAM模式下，Nav2的bringup_launch.py需要map参数
    # 但我们暂时只启动SLAM，Nav2导航栈可以在有地图后单独启动
    # 如果你想要同时启动Nav2，需要提供一个空地图或使用不同的launch文件
    
    # Slam Toolbox节点
    # 注意：sync_slam_toolbox_node 是普通节点，不是生命周期节点，所以不需要lifecycle_manager
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/odom', '/odom'),
            ('/map', '/map'),
            ('/map_metadata', '/map_metadata')
        ]
    )
    
    # 注意：sync_slam_toolbox_node 不需要 lifecycle_manager
    # 如果使用 async_slam_toolbox_node，则需要 lifecycle_manager
    # lifecycle_manager 已注释，因为 sync_slam_toolbox_node 会自动启动
    
    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        slam_params_file_arg,
        # nav2_bringup,  # 暂时注释掉，因为需要map参数，SLAM模式下可以先不启动Nav2
        slam_toolbox,
        # lifecycle_manager,  # sync_slam_toolbox_node 不需要 lifecycle_manager
    ])

