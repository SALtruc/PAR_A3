"""Launch Project C: reactive obstacle avoidance."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('rosbot_obstacle_avoidance')
    config = PathJoinSubstitution([pkg, 'config', 'params.yaml'])

    args = [
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('tof_topic', default_value='/range'),
        DeclareLaunchArgument('use_tof', default_value='true'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('cmd_vel_stamped', default_value='true'),
    ]

    obstacle_avoidance = Node(
        package='rosbot_obstacle_avoidance',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        parameters=[
            config,
            {
                'scan_topic': LaunchConfiguration('scan_topic'),
                'tof_topic': LaunchConfiguration('tof_topic'),
                'use_tof': LaunchConfiguration('use_tof'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'cmd_vel_stamped': LaunchConfiguration('cmd_vel_stamped'),
            },
        ],
        output='screen',
    )

    return LaunchDescription(args + [obstacle_avoidance])
