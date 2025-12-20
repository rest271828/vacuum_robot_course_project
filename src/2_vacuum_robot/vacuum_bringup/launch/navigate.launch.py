from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params = LaunchConfiguration('nav2_params')
    slam_params = LaunchConfiguration('slam_params')
    rviz_config = LaunchConfiguration('rviz_config')

    pkg_share = FindPackageShare('vacuum_bringup').find('vacuum_bringup')

    default_nav2_params = os.path.join(pkg_share, 'config', 'nav2_params_real.yaml')
    default_slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'nav2_default.rviz')

    # slam_toolbox online_async
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': use_sim_time}],
        remappings=[('scan', '/scan')],
    )

    # nav2 bringup (navigation)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('nav2_bringup').find('nav2_bringup'),
                'launch',
                'navigation_launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': 'true',
        }.items(),
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('nav2_params', default_value=default_nav2_params),
        DeclareLaunchArgument('slam_params', default_value=default_slam_params),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),

        # slam + nav2 + rviz
        slam_node,
        nav2_launch,
        rviz,
    ])
