"""Launch Project C with Nav2 Collision Monitor as the final safety layer."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
        DeclareLaunchArgument('debug_decisions', default_value='true'),
    ]

    project_c = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(project_c_launch),
        launch_arguments={
            'scan_topic': LaunchConfiguration('scan_topic'),
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_raw_topic'),
            'debug_decisions': LaunchConfiguration('debug_decisions'),
        }.items(),
    )

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
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
        output='screen',
        parameters=[
            {
                'autostart': True,
                'node_names': ['collision_monitor'],
            }
        ],
    )

    return LaunchDescription(args + [
        project_c,
        collision_monitor,
        lifecycle_manager,
    ])
