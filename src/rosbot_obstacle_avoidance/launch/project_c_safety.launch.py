"""Launch Project C with Nav2 Collision Monitor as the final safety layer."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('rosbot_obstacle_avoidance')
    project_c_launch = PathJoinSubstitution([pkg, 'launch', 'project_c.launch.py'])
    collision_config = PathJoinSubstitution(
        [pkg, 'config', 'collision_monitor_params.yaml']
    )

    args = [
        DeclareLaunchArgument('scan_topic', default_value='/scan_filtered'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_rect_raw'),
        DeclareLaunchArgument('pointcloud_topic', default_value='/oak/points'),
        DeclareLaunchArgument('pointcloud_frame', default_value='optical'),
        DeclareLaunchArgument('pointcloud_target_frame', default_value='base_link'),
        DeclareLaunchArgument('pointcloud_use_tf', default_value='true'),
        DeclareLaunchArgument('pointcloud_tf_timeout_sec', default_value='0.03'),
        DeclareLaunchArgument('pointcloud_qos', default_value='sensor_data'),
        DeclareLaunchArgument('tof_topic', default_value='/range'),
        DeclareLaunchArgument(
            'tof_topics',
            default_value='/range/fl,/range/fr,/range/rl,/range/rr',
        ),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_depth', default_value='false'),
        DeclareLaunchArgument('use_pointcloud', default_value='true'),
        DeclareLaunchArgument('use_tof', default_value='true'),
        DeclareLaunchArgument('front_center_angle_deg', default_value='180.0'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('cmd_vel_raw_topic', default_value='/cmd_vel_raw'),
        DeclareLaunchArgument('cmd_vel_stamped', default_value='true'),
        DeclareLaunchArgument('max_speed', default_value='0.06'),
        DeclareLaunchArgument('stop_distance', default_value='0.15'),
        DeclareLaunchArgument('hard_backup_distance', default_value='0.10'),
        DeclareLaunchArgument('side_guard_distance', default_value='0.07'),
        DeclareLaunchArgument('backup_speed', default_value='0.04'),
        DeclareLaunchArgument('battery_topic', default_value='/battery'),
        DeclareLaunchArgument('require_battery_ok', default_value='false'),
        DeclareLaunchArgument('min_battery_voltage', default_value='11.1'),
        DeclareLaunchArgument('warn_battery_voltage', default_value='11.4'),
        DeclareLaunchArgument('battery_stale_sec', default_value='3.0'),
        DeclareLaunchArgument('debug_decisions', default_value='true'),
        DeclareLaunchArgument('use_nav2_collision_monitor', default_value='false'),
    ]

    project_c_direct = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(project_c_launch),
        condition=UnlessCondition(LaunchConfiguration('use_nav2_collision_monitor')),
        launch_arguments={
            'scan_topic': LaunchConfiguration('scan_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'pointcloud_frame': LaunchConfiguration('pointcloud_frame'),
            'pointcloud_target_frame': LaunchConfiguration('pointcloud_target_frame'),
            'pointcloud_use_tf': LaunchConfiguration('pointcloud_use_tf'),
            'pointcloud_tf_timeout_sec': LaunchConfiguration('pointcloud_tf_timeout_sec'),
            'pointcloud_qos': LaunchConfiguration('pointcloud_qos'),
            'tof_topic': LaunchConfiguration('tof_topic'),
            'tof_topics': LaunchConfiguration('tof_topics'),
            'use_lidar': LaunchConfiguration('use_lidar'),
            'use_depth': LaunchConfiguration('use_depth'),
            'use_pointcloud': LaunchConfiguration('use_pointcloud'),
            'use_tof': LaunchConfiguration('use_tof'),
            'front_center_angle_deg': LaunchConfiguration('front_center_angle_deg'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'cmd_vel_stamped': LaunchConfiguration('cmd_vel_stamped'),
            'max_speed': LaunchConfiguration('max_speed'),
            'stop_distance': LaunchConfiguration('stop_distance'),
            'hard_backup_distance': LaunchConfiguration('hard_backup_distance'),
            'side_guard_distance': LaunchConfiguration('side_guard_distance'),
            'backup_speed': LaunchConfiguration('backup_speed'),
            'battery_topic': LaunchConfiguration('battery_topic'),
            'require_battery_ok': LaunchConfiguration('require_battery_ok'),
            'min_battery_voltage': LaunchConfiguration('min_battery_voltage'),
            'warn_battery_voltage': LaunchConfiguration('warn_battery_voltage'),
            'battery_stale_sec': LaunchConfiguration('battery_stale_sec'),
            'debug_decisions': LaunchConfiguration('debug_decisions'),
        }.items(),
    )

    project_c_raw = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(project_c_launch),
        condition=IfCondition(LaunchConfiguration('use_nav2_collision_monitor')),
        launch_arguments={
            'scan_topic': LaunchConfiguration('scan_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'pointcloud_frame': LaunchConfiguration('pointcloud_frame'),
            'pointcloud_target_frame': LaunchConfiguration('pointcloud_target_frame'),
            'pointcloud_use_tf': LaunchConfiguration('pointcloud_use_tf'),
            'pointcloud_tf_timeout_sec': LaunchConfiguration('pointcloud_tf_timeout_sec'),
            'pointcloud_qos': LaunchConfiguration('pointcloud_qos'),
            'tof_topic': LaunchConfiguration('tof_topic'),
            'tof_topics': LaunchConfiguration('tof_topics'),
            'use_lidar': LaunchConfiguration('use_lidar'),
            'use_depth': LaunchConfiguration('use_depth'),
            'use_pointcloud': LaunchConfiguration('use_pointcloud'),
            'use_tof': LaunchConfiguration('use_tof'),
            'front_center_angle_deg': LaunchConfiguration('front_center_angle_deg'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_raw_topic'),
            'cmd_vel_stamped': LaunchConfiguration('cmd_vel_stamped'),
            'max_speed': LaunchConfiguration('max_speed'),
            'stop_distance': LaunchConfiguration('stop_distance'),
            'hard_backup_distance': LaunchConfiguration('hard_backup_distance'),
            'side_guard_distance': LaunchConfiguration('side_guard_distance'),
            'backup_speed': LaunchConfiguration('backup_speed'),
            'battery_topic': LaunchConfiguration('battery_topic'),
            'require_battery_ok': LaunchConfiguration('require_battery_ok'),
            'min_battery_voltage': LaunchConfiguration('min_battery_voltage'),
            'warn_battery_voltage': LaunchConfiguration('warn_battery_voltage'),
            'battery_stale_sec': LaunchConfiguration('battery_stale_sec'),
            'debug_decisions': LaunchConfiguration('debug_decisions'),
        }.items(),
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

    return LaunchDescription(args + [
        project_c_direct,
        project_c_raw,
        collision_monitor,
        lifecycle_manager,
    ])
