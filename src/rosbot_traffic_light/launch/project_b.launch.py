"""Launch Project B: traffic light detector + controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('rosbot_traffic_light')
    config = PathJoinSubstitution([pkg, 'config', 'params.yaml'])

    args = [
        DeclareLaunchArgument('image_topic', default_value='/oak/rgb/image_raw'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('cmd_vel_stamped', default_value='true'),
        DeclareLaunchArgument('show_debug', default_value='false'),
    ]

    traffic_light_detector = Node(
        package='rosbot_traffic_light',
        executable='traffic_light_detector',
        name='traffic_light_detector',
        parameters=[
            config,
            {
                'image_topic': LaunchConfiguration('image_topic'),
                'show_debug': LaunchConfiguration('show_debug'),
            },
        ],
        output='screen',
    )

    traffic_light_controller = Node(
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

    return LaunchDescription(args + [
        traffic_light_detector,
        traffic_light_controller,
    ])
