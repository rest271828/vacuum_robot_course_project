#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 是否使用仿真时间（真机一般 false，但可以留这个参数，通用一点）
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_share = FindPackageShare('vacuum_robot_real')

    # ⭐ 注意这里用的是 real 版本的 xacro
    robot_xacro = PathJoinSubstitution([
        pkg_share,
        'urdf',
        'robot_base_real.xacro'
    ])

    robot_description = Command([
        'xacro ', robot_xacro
    ])

    # robot_state_publisher：发布 TF + /robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    # joint_state_publisher：
    # - 如果真机关节状态来自驱动（/joint_states），你可以删掉这个节点
    # - 如果没有，就先保留，至少 RViz 里关节会有东西
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher,
        joint_state_publisher,
    ])
