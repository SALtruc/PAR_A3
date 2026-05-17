"""Launch Project C with safety defaults and optional Nav2 Collision Monitor."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def as_bool(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def as_float(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def as_int(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def generate_launch_description():
    pkg = FindPackageShare('rosbot_obstacle_avoidance')
    config = PathJoinSubstitution([pkg, 'config', 'params.yaml'])
    collision_config = PathJoinSubstitution(
        [pkg, 'config', 'collision_monitor_params.yaml']
    )
    controller_cmd_vel_topic = PythonExpression([
        "'",
        LaunchConfiguration('cmd_vel_raw_topic'),
        "' if '",
        LaunchConfiguration('use_nav2_collision_monitor'),
        "'.lower() in ('true', '1', 'yes') else '",
        LaunchConfiguration('cmd_vel_topic'),
        "'",
    ])

    args = [
        DeclareLaunchArgument('scan_topic', default_value='/scan_filtered'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_rect_raw'),
        DeclareLaunchArgument('pointcloud_topic', default_value='/oak/points'),
        DeclareLaunchArgument('pointcloud_frame', default_value='optical'),
        DeclareLaunchArgument('pointcloud_target_frame', default_value='base_link'),
        DeclareLaunchArgument('pointcloud_use_tf', default_value='true'),
        DeclareLaunchArgument('pointcloud_tf_timeout_sec', default_value='0.03'),
        DeclareLaunchArgument('pointcloud_qos', default_value='sensor_data'),
        DeclareLaunchArgument('pointcloud_low_min_height_m', default_value='-0.90'),
        DeclareLaunchArgument('pointcloud_low_max_height_m', default_value='0.70'),
        DeclareLaunchArgument('pointcloud_low_center_half_width_m', default_value='0.32'),
        DeclareLaunchArgument('pointcloud_low_min_points', default_value='1'),
        DeclareLaunchArgument('tof_topic', default_value='/range'),
        DeclareLaunchArgument(
            'tof_topics',
            default_value='/range/fl,/range/fr,/range/rl,/range/rr',
        ),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_depth', default_value='true'),
        DeclareLaunchArgument('use_pointcloud', default_value='true'),
        DeclareLaunchArgument('use_tof', default_value='true'),
        DeclareLaunchArgument('obstacle_topic', default_value='/obstacle_representation'),
        DeclareLaunchArgument('battery_topic', default_value='/battery'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('cmd_vel_raw_topic', default_value='/cmd_vel_raw'),
        DeclareLaunchArgument('cmd_vel_stamped', default_value='true'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument('front_tof_topics', default_value='/range/fl,/range/fr'),
        DeclareLaunchArgument('log_dir', default_value='~/rosbot_obstacle_logs'),
        DeclareLaunchArgument('max_speed', default_value='0.22'),   # speed
        DeclareLaunchArgument('emergency_distance', default_value='0.14'),
        DeclareLaunchArgument('perception_obstacle_distance', default_value='0.22'),
        DeclareLaunchArgument('perception_clear_distance', default_value='0.35'),
        DeclareLaunchArgument('clear_distance', default_value='0.45'),  # side clearance
        DeclareLaunchArgument('stop_distance', default_value='0.25'),   # front
        DeclareLaunchArgument('hard_backup_distance', default_value='0.10'),
        DeclareLaunchArgument('front_tof_obstacle_distance', default_value='0.12'),
        DeclareLaunchArgument('front_tof_hard_distance', default_value='0.12'),
        DeclareLaunchArgument('dodge_clearance', default_value='0.12'),
        DeclareLaunchArgument('rear_stop_distance', default_value='0.20'),
        DeclareLaunchArgument('side_guard_distance', default_value='0.08'),
        DeclareLaunchArgument('side_escape_distance', default_value='0.12'),
        DeclareLaunchArgument('side_escape_angular_speed', default_value='0.32'),
        DeclareLaunchArgument('side_escape_sec', default_value='0.45'),
        DeclareLaunchArgument('edge_escape_enabled', default_value='true'),
        DeclareLaunchArgument('edge_escape_front_distance', default_value='0.25'),
        DeclareLaunchArgument('edge_escape_clearance', default_value='0.20'),
        DeclareLaunchArgument('edge_escape_angular_speed', default_value='0.45'),
        DeclareLaunchArgument('edge_escape_sec', default_value='1.00'),
        DeclareLaunchArgument('edge_escape_max_attempts', default_value='3'),
        DeclareLaunchArgument('corner_backup_side_distance', default_value='0.05'),
        DeclareLaunchArgument('corner_backup_front_distance', default_value='0.35'),
        DeclareLaunchArgument('corner_backup_both_sides_distance', default_value='0.10'),
        DeclareLaunchArgument('observe_speed', default_value='0.0'),
        DeclareLaunchArgument('dodge_forward_speed', default_value='0.045'),    # force?
        DeclareLaunchArgument('dodge_angular_speed', default_value='0.55'),
        DeclareLaunchArgument('rotation_angular_speed', default_value='0.60'),
        DeclareLaunchArgument('backup_speed', default_value='0.08'),
        DeclareLaunchArgument('backup_sec', default_value='1.60'),
        DeclareLaunchArgument('dodge_step_deg', default_value='60.0'),
        DeclareLaunchArgument('dodge_pivot_sec', default_value='0.60'),
        DeclareLaunchArgument('rotation_step_deg', default_value='95.0'),
        DeclareLaunchArgument('rotation_commit_sec', default_value='0.65'),
        DeclareLaunchArgument('max_rotation_attempts', default_value='3'),
        DeclareLaunchArgument('observe_frames', default_value='8'),
        DeclareLaunchArgument('clear_observe_frames', default_value='3'),
        DeclareLaunchArgument('front_release_distance', default_value='0.45'),
        DeclareLaunchArgument('front_clear_exit_frames', default_value='5'),
        DeclareLaunchArgument('creep_speed', default_value='0.040'),    # force?
        DeclareLaunchArgument('surprise_backup_enabled', default_value='true'),
        DeclareLaunchArgument('surprise_backup_distance', default_value='0.20'),
        DeclareLaunchArgument('surprise_backup_sec', default_value='0.45'),
        DeclareLaunchArgument('surprise_backup_cooldown_sec', default_value='1.20'),
        DeclareLaunchArgument('dynamic_observe_distance', default_value='0.80'),
        DeclareLaunchArgument('depth_obstacle_distance', default_value='0.45'),
        DeclareLaunchArgument('front_center_angle_deg', default_value='180.0'),
        DeclareLaunchArgument('dynamic_closing_speed', default_value='0.80'),
        DeclareLaunchArgument('obstacle_hold_sec', default_value='0.30'),
        DeclareLaunchArgument('clear_confirm_sec', default_value='0.20'),
        DeclareLaunchArgument('robot_half_width_m', default_value='0.13'),
        DeclareLaunchArgument('front_path_half_width_m', default_value='0.14'),
        DeclareLaunchArgument('side_guard_forward_m', default_value='0.28'),
        DeclareLaunchArgument('side_guard_rear_m', default_value='0.20'),
        DeclareLaunchArgument('side_percentile', default_value='10.0'),
        DeclareLaunchArgument('front_percentile', default_value='5.0'),
        DeclareLaunchArgument('front_close_min_rays', default_value='3'),
        DeclareLaunchArgument('front_close_min_ratio', default_value='0.01'),
        DeclareLaunchArgument('clearer_side_deadband', default_value='0.05'),
        DeclareLaunchArgument('avoid_turn_direction', default_value='-1.0'),
        DeclareLaunchArgument('require_battery_ok', default_value='false'),
        DeclareLaunchArgument('min_battery_voltage', default_value='10.0'),
        DeclareLaunchArgument('warn_battery_voltage', default_value='10.5'),
        DeclareLaunchArgument('battery_stale_sec', default_value='3.0'),
        DeclareLaunchArgument('contact_recovery_enabled', default_value='true'),
        DeclareLaunchArgument('contact_cmd_speed_min', default_value='0.025'),
        DeclareLaunchArgument('contact_odom_speed_max', default_value='0.012'),
        DeclareLaunchArgument('contact_odom_angular_max', default_value='0.08'),
        DeclareLaunchArgument('contact_stall_sec', default_value='1.50'),
        DeclareLaunchArgument('contact_recovery_cooldown_sec', default_value='1.8'),
        DeclareLaunchArgument('contact_odom_stale_sec', default_value='0.5'),
        DeclareLaunchArgument('tilt_recovery_enabled', default_value='true'),
        DeclareLaunchArgument('tilt_backup_deg', default_value='8.0'),
        DeclareLaunchArgument('tilt_stop_deg', default_value='18.0'),
        DeclareLaunchArgument('tilt_stop_pause_sec', default_value='0.30'),
        DeclareLaunchArgument('tilt_imu_stale_sec', default_value='0.6'),
        DeclareLaunchArgument('debug_decisions', default_value='true'),
        DeclareLaunchArgument('debug_period_sec', default_value='1.0'),
        DeclareLaunchArgument('local_only', default_value='false'),
        DeclareLaunchArgument('use_nav2_collision_monitor', default_value='true'),
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
                'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
                'pointcloud_frame': LaunchConfiguration('pointcloud_frame'),
                'pointcloud_target_frame': LaunchConfiguration('pointcloud_target_frame'),
                'pointcloud_use_tf': as_bool('pointcloud_use_tf'),
                'pointcloud_tf_timeout_sec': as_float('pointcloud_tf_timeout_sec'),
                'pointcloud_qos': LaunchConfiguration('pointcloud_qos'),
                'pointcloud_low_min_height_m': as_float('pointcloud_low_min_height_m'),
                'pointcloud_low_max_height_m': as_float('pointcloud_low_max_height_m'),
                'pointcloud_low_center_half_width_m': as_float(
                    'pointcloud_low_center_half_width_m'
                ),
                'pointcloud_low_min_points': as_int('pointcloud_low_min_points'),
                'tof_topic': LaunchConfiguration('tof_topic'),
                'tof_topics': LaunchConfiguration('tof_topics'),
                'use_lidar': as_bool('use_lidar'),
                'use_depth': as_bool('use_depth'),
                'use_pointcloud': as_bool('use_pointcloud'),
                'use_tof': as_bool('use_tof'),
                'obstacle_topic': LaunchConfiguration('obstacle_topic'),
                'emergency_distance': as_float('emergency_distance'),
                'obstacle_distance': as_float('perception_obstacle_distance'),
                'clear_distance': as_float('perception_clear_distance'),
                'front_center_angle_deg': as_float('front_center_angle_deg'),
                'robot_half_width_m': as_float('robot_half_width_m'),
                'front_path_half_width_m': as_float('front_path_half_width_m'),
                'side_guard_forward_m': as_float('side_guard_forward_m'),
                'side_guard_rear_m': as_float('side_guard_rear_m'),
                'side_percentile': as_float('side_percentile'),
                'front_percentile': as_float('front_percentile'),
                'front_close_min_rays': as_int('front_close_min_rays'),
                'front_close_min_ratio': as_float('front_close_min_ratio'),
                'depth_obstacle_distance': as_float('depth_obstacle_distance'),
                'dynamic_closing_speed': as_float('dynamic_closing_speed'),
                'obstacle_hold_sec': as_float('obstacle_hold_sec'),
                'clear_confirm_sec': as_float('clear_confirm_sec'),
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
                'battery_topic': LaunchConfiguration('battery_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'front_tof_topics': LaunchConfiguration('front_tof_topics'),
                'cmd_vel_topic': controller_cmd_vel_topic,
                'cmd_vel_stamped': as_bool('cmd_vel_stamped'),
                'max_speed': as_float('max_speed'),
                'observe_speed': as_float('observe_speed'),
                'dodge_forward_speed': as_float('dodge_forward_speed'),
                'dodge_angular_speed': as_float('dodge_angular_speed'),
                'rotation_angular_speed': as_float('rotation_angular_speed'),
                'backup_speed': as_float('backup_speed'),
                'clear_distance': as_float('clear_distance'),
                'stop_distance': as_float('stop_distance'),
                'hard_backup_distance': as_float('hard_backup_distance'),
                'front_tof_obstacle_distance': as_float('front_tof_obstacle_distance'),
                'front_tof_hard_distance': as_float('front_tof_hard_distance'),
                'dodge_clearance': as_float('dodge_clearance'),
                'rear_stop_distance': as_float('rear_stop_distance'),
                'side_guard_distance': as_float('side_guard_distance'),
                'side_escape_distance': as_float('side_escape_distance'),
                'side_escape_angular_speed': as_float('side_escape_angular_speed'),
                'side_escape_sec': as_float('side_escape_sec'),
                'edge_escape_enabled': as_bool('edge_escape_enabled'),
                'edge_escape_front_distance': as_float('edge_escape_front_distance'),
                'edge_escape_clearance': as_float('edge_escape_clearance'),
                'edge_escape_angular_speed': as_float('edge_escape_angular_speed'),
                'edge_escape_sec': as_float('edge_escape_sec'),
                'edge_escape_max_attempts': as_int('edge_escape_max_attempts'),
                'corner_backup_side_distance': as_float('corner_backup_side_distance'),
                'corner_backup_front_distance': as_float('corner_backup_front_distance'),
                'corner_backup_both_sides_distance': as_float(
                    'corner_backup_both_sides_distance'
                ),
                'dynamic_observe_distance': as_float('dynamic_observe_distance'),
                'observe_frames': as_int('observe_frames'),
                'clear_observe_frames': as_int('clear_observe_frames'),
                'front_release_distance': as_float('front_release_distance'),
                'front_clear_exit_frames': as_int('front_clear_exit_frames'),
                'backup_sec': as_float('backup_sec'),
                'dodge_step_deg': as_float('dodge_step_deg'),
                'dodge_pivot_sec': as_float('dodge_pivot_sec'),
                'rotation_step_deg': as_float('rotation_step_deg'),
                'rotation_commit_sec': as_float('rotation_commit_sec'),
                'max_rotation_attempts': as_int('max_rotation_attempts'),
                'clearer_side_deadband': as_float('clearer_side_deadband'),
                'avoid_turn_direction': as_float('avoid_turn_direction'),
                'creep_speed': as_float('creep_speed'),
                'surprise_backup_enabled': as_bool('surprise_backup_enabled'),
                'surprise_backup_distance': as_float('surprise_backup_distance'),
                'surprise_backup_sec': as_float('surprise_backup_sec'),
                'surprise_backup_cooldown_sec': as_float(
                    'surprise_backup_cooldown_sec'
                ),
                'require_battery_ok': as_bool('require_battery_ok'),
                'min_battery_voltage': as_float('min_battery_voltage'),
                'warn_battery_voltage': as_float('warn_battery_voltage'),
                'battery_stale_sec': as_float('battery_stale_sec'),
                'contact_recovery_enabled': as_bool('contact_recovery_enabled'),
                'contact_cmd_speed_min': as_float('contact_cmd_speed_min'),
                'contact_odom_speed_max': as_float('contact_odom_speed_max'),
                'contact_odom_angular_max': as_float('contact_odom_angular_max'),
                'contact_stall_sec': as_float('contact_stall_sec'),
                'contact_recovery_cooldown_sec': as_float('contact_recovery_cooldown_sec'),
                'contact_odom_stale_sec': as_float('contact_odom_stale_sec'),
                'tilt_recovery_enabled': as_bool('tilt_recovery_enabled'),
                'tilt_backup_deg': as_float('tilt_backup_deg'),
                'tilt_stop_deg': as_float('tilt_stop_deg'),
                'tilt_stop_pause_sec': as_float('tilt_stop_pause_sec'),
                'tilt_imu_stale_sec': as_float('tilt_imu_stale_sec'),
                'debug_decisions': as_bool('debug_decisions'),
                'debug_period_sec': as_float('debug_period_sec'),
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

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        condition=IfCondition(LaunchConfiguration('use_nav2_collision_monitor')),
        output='screen',
        parameters=[
            collision_config,
            {
                'cmd_vel_in_topic': LaunchConfiguration('cmd_vel_raw_topic'),
                'cmd_vel_out_topic': LaunchConfiguration('cmd_vel_topic'),
                'scan.topic': LaunchConfiguration('scan_topic'),
                'pointcloud.topic': LaunchConfiguration('pointcloud_topic'),
            },
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor',
        condition=IfCondition(LaunchConfiguration('use_nav2_collision_monitor')),
        output='screen',
        parameters=[
            {
                'autostart': True,
                'node_names': ['collision_monitor'],
            }
        ],
    )

    local_only_env = [
        SetEnvironmentVariable(
            'ROS_AUTOMATIC_DISCOVERY_RANGE',
            'LOCALHOST',
            condition=IfCondition(LaunchConfiguration('local_only')),
        ),
    ]

    return LaunchDescription(args + local_only_env + [
        obstacle_perception,
        obstacle_avoidance,
        obstacle_trial_logger,
        collision_monitor,
        lifecycle_manager,
    ])
