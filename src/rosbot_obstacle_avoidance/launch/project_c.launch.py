"""Launch Project C: fused perception, reactive decision, and trial logging."""

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
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_rect_raw'),
        DeclareLaunchArgument('tof_topic', default_value='/range'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_depth', default_value='true'),
        DeclareLaunchArgument('use_tof', default_value='true'),
        DeclareLaunchArgument('obstacle_topic', default_value='/obstacle_representation'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('cmd_vel_stamped', default_value='true'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('log_dir', default_value='~/rosbot_obstacle_logs'),
        DeclareLaunchArgument('max_speed', default_value='0.10'),
        DeclareLaunchArgument('emergency_distance', default_value='0.25'),
        DeclareLaunchArgument('obstacle_distance', default_value='0.55'),
        DeclareLaunchArgument('clear_distance', default_value='0.80'),
        DeclareLaunchArgument('depth_obstacle_distance', default_value='0.80'),
        DeclareLaunchArgument('dynamic_closing_speed', default_value='0.80'),
        DeclareLaunchArgument('avoid_turn_only_distance', default_value='0.45'),
        DeclareLaunchArgument('avoid_forward_distance', default_value='0.80'),
        DeclareLaunchArgument('dynamic_hold_sec', default_value='0.80'),
        DeclareLaunchArgument('front_percentile', default_value='15.0'),
        DeclareLaunchArgument('wander_enabled', default_value='false'),
        DeclareLaunchArgument('debug_decisions', default_value='true'),
        DeclareLaunchArgument('debug_period_sec', default_value='1.0'),
    ]

    obstacle_perception = Node(
        package='rosbot_obstacle_avoidance',
        executable='obstacle_perception',
        name='obstacle_perception',
        parameters=[
            config,
            {
                'scan_topic': LaunchConfiguration('scan_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'tof_topic': LaunchConfiguration('tof_topic'),
                'use_lidar': LaunchConfiguration('use_lidar'),
                'use_depth': LaunchConfiguration('use_depth'),
                'use_tof': LaunchConfiguration('use_tof'),
                'obstacle_topic': LaunchConfiguration('obstacle_topic'),
                'emergency_distance': LaunchConfiguration('emergency_distance'),
                'obstacle_distance': LaunchConfiguration('obstacle_distance'),
                'clear_distance': LaunchConfiguration('clear_distance'),
                'front_percentile': LaunchConfiguration('front_percentile'),
                'depth_obstacle_distance': LaunchConfiguration(
                    'depth_obstacle_distance'
                ),
                'dynamic_closing_speed': LaunchConfiguration(
                    'dynamic_closing_speed'
                ),
            },
        ],
        output='screen',
    )

    obstacle_avoidance = Node(
        package='rosbot_obstacle_avoidance',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        parameters=[
            config,
            {
                'obstacle_topic': LaunchConfiguration('obstacle_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'cmd_vel_stamped': LaunchConfiguration('cmd_vel_stamped'),
                'max_speed': LaunchConfiguration('max_speed'),
                'obstacle_distance': LaunchConfiguration('obstacle_distance'),
                'clear_distance': LaunchConfiguration('clear_distance'),
                'avoid_turn_only_distance': LaunchConfiguration(
                    'avoid_turn_only_distance'
                ),
                'avoid_forward_distance': LaunchConfiguration(
                    'avoid_forward_distance'
                ),
                'dynamic_hold_sec': LaunchConfiguration('dynamic_hold_sec'),
                'wander_enabled': LaunchConfiguration('wander_enabled'),
                'debug_decisions': LaunchConfiguration('debug_decisions'),
                'debug_period_sec': LaunchConfiguration('debug_period_sec'),
            },
        ],
        output='screen',
    )

    obstacle_trial_logger = Node(
        package='rosbot_obstacle_avoidance',
        executable='obstacle_trial_logger',
        name='obstacle_trial_logger',
        parameters=[
            config,
            {
                'obstacle_topic': LaunchConfiguration('obstacle_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'log_dir': LaunchConfiguration('log_dir'),
            },
        ],
        output='screen',
    )

    return LaunchDescription(args + [
        obstacle_perception,
        obstacle_avoidance,
        obstacle_trial_logger,
    ])
