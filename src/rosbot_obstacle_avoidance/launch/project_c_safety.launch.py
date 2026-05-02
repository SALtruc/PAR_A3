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
        DeclareLaunchArgument('pointcloud_topic', default_value='/oak/points'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('cmd_vel_raw_topic', default_value='/cmd_vel_raw'),
        DeclareLaunchArgument('max_speed', default_value='0.06'),
        DeclareLaunchArgument('backup_speed', default_value='0.04'),
        DeclareLaunchArgument('battery_topic', default_value='/battery'),
        DeclareLaunchArgument('require_battery_ok', default_value='true'),
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
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'max_speed': LaunchConfiguration('max_speed'),
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
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_raw_topic'),
            'max_speed': LaunchConfiguration('max_speed'),
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
