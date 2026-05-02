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
        DeclareLaunchArgument('emergency_distance', default_value='0.10'),
        DeclareLaunchArgument('obstacle_distance', default_value='0.15'),
        DeclareLaunchArgument('clear_distance', default_value='0.25'),
        DeclareLaunchArgument('front_body_offset_m', default_value='0.10'),
        DeclareLaunchArgument('depth_obstacle_distance', default_value='0.80'),
        DeclareLaunchArgument('dynamic_closing_speed', default_value='0.80'),
        DeclareLaunchArgument('obstacle_hold_sec', default_value='0.35'),
        DeclareLaunchArgument('clear_confirm_sec', default_value='0.20'),
        DeclareLaunchArgument('dynamic_check_frames', default_value='4'),
        DeclareLaunchArgument('dynamic_clear_frames', default_value='2'),
        DeclareLaunchArgument('side_protect_distance', default_value='0.25'),
        DeclareLaunchArgument('front_percentile', default_value='15.0'),
        DeclareLaunchArgument('front_close_min_rays', default_value='3'),
        DeclareLaunchArgument('front_close_min_ratio', default_value='0.01'),
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
                'front_close_min_rays': LaunchConfiguration('front_close_min_rays'),
                'front_close_min_ratio': LaunchConfiguration(
                    'front_close_min_ratio'
                ),
                'depth_obstacle_distance': LaunchConfiguration(
                    'depth_obstacle_distance'
                ),
                'dynamic_closing_speed': LaunchConfiguration(
                    'dynamic_closing_speed'
                ),
                'obstacle_hold_sec': LaunchConfiguration('obstacle_hold_sec'),
                'clear_confirm_sec': LaunchConfiguration('clear_confirm_sec'),
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
                'front_body_offset_m': LaunchConfiguration('front_body_offset_m'),
                'dynamic_check_frames': LaunchConfiguration('dynamic_check_frames'),
                'dynamic_clear_frames': LaunchConfiguration('dynamic_clear_frames'),
                'side_protect_distance': LaunchConfiguration(
                    'side_protect_distance'
                ),
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
