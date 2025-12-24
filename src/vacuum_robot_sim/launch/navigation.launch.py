"""
Nav2 导航启动文件 (支持 SLAM 或 静态地图加载)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # ========== 声明 Launch 参数 ==========
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    map_file = LaunchConfiguration('map') # 新增地图参数

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='是否使用仿真时间')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('vacuum_robot_sim'), 'config', 'nav2_params.yaml']),  # 使用 nav2_params.yaml (较新版本)
        description='Nav2参数文件路径')

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('vacuum_robot_sim'), 'config', 'slam_toolbox.yaml']),
        description='Slam Toolbox参数文件路径')
    
    # 新增 map 参数，默认为空（空表示运行 SLAM）
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='静态地图(.yaml)的完整路径。如果为空，则运行SLAM。')

    # ========== 1. 模式 A: SLAM Toolbox (当 map 为空时运行) ==========
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
        # 只有当 map 参数为空字符串时，才启动这个节点
        condition=IfCondition(PythonExpression(["'", map_file, "' == ''"]))
    )

    # ========== 2. 模式 B: Localization (当 map 不为空时运行) ==========
    # 启动 map_server 和 amcl
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nav2_bringup'), 'launch', 'localization_launch.py'
        ])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': params_file,
            'use_lifecycle_mgr': 'false', # 由 navigation_launch 统一管理生命周期
            'use_composition': 'False',
        }.items(),
        # 只有当 map 参数不为空时，才启动定位
        condition=IfCondition(PythonExpression(["'", map_file, "' != ''"]))
    )

    # ========== 3. Nav2 导航栈 (总是运行) ==========
    # 注意：如果运行 AMCL，Nav2 应该加载 map；如果运行 SLAM，Nav2 不需要加载 map (map='')
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'
        ])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'False',
            # 如果是 Localization 模式，navigation_launch 不需要再处理 map，因为 localization_launch 处理了
            # 如果是 SLAM 模式，map 也是空的
            'map': '', 
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        slam_params_file_arg,
        map_arg,
        slam_toolbox,
        localization_launch,
        nav2_navigation_launch,
    ])