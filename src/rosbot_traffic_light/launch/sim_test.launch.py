"""Dry-test Project B controller without a camera."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('rosbot_traffic_light')
    config = PathJoinSubstitution([pkg, 'config', 'params.yaml'])

    args = [
        DeclareLaunchArgument('cmd_vel_topic', default_value='/dummy_cmd_vel'),
        DeclareLaunchArgument('cmd_vel_stamped', default_value='true'),
        DeclareLaunchArgument(
            'script',
            default_value='1.0:RED,4.0:GREEN,8.0:YELLOW,11.0:RED',
        ),
        DeclareLaunchArgument('stop_after_sec', default_value='14.0'),
    ]

    controller = Node(
        package='rosbot_traffic_light',
        executable='traffic_light_controller',
        name='traffic_light_controller',
        parameters=[
            config,
            {
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'cmd_vel_stamped': LaunchConfiguration('cmd_vel_stamped'),
            },
        ],
        output='screen',
    )

    sim = Node(
        package='rosbot_traffic_light',
        executable='traffic_light_sim',
        name='traffic_light_sim',
        parameters=[
            {
                'script': LaunchConfiguration('script'),
                'stop_after_sec': LaunchConfiguration('stop_after_sec'),
            },
        ],
        output='screen',
    )

    return LaunchDescription(args + [controller, sim])
