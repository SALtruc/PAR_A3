"""
Simulation launch file for testing command interpretation and navigation FSM.

This does not start the camera QR detector. Instead, simulation_driver publishes
fake /qr_detected messages into the same pipeline used by the real robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('rosbot_qr_navigation')
    config = PathJoinSubstitution([pkg, 'config', 'params.yaml'])

    args = [
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('cmd_vel_stamped', default_value='true'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('start_state', default_value='STOPPED'),
        DeclareLaunchArgument('stop_after_sec', default_value='35.0'),
        DeclareLaunchArgument('obstacle_start_sec', default_value='0.5'),
        DeclareLaunchArgument('obstacle_end_sec', default_value='2.5'),
        DeclareLaunchArgument(
            'script',
            default_value=(
                '1.0:GO,3.0:SPEED_UP,5.0:TURN_LEFT,5.5:AND_TURN_RIGHT,'
                '6.0:AND_U_TURN,16.0:STOP,19.0:GO,23.0:SPEED_DOWN'
            ),
        ),
        DeclareLaunchArgument('log_dir', default_value='~/rosbot_qr_logs'),
    ]

    command_interpreter = Node(
        package='rosbot_qr_navigation',
        executable='command_interpreter',
        name='command_interpreter',
        parameters=[config],
        output='screen',
    )

    navigation_fsm = Node(
        package='rosbot_qr_navigation',
        executable='navigation_fsm',
        name='navigation_fsm',
        parameters=[
            config,
            {
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'cmd_vel_stamped': LaunchConfiguration('cmd_vel_stamped'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'start_state': LaunchConfiguration('start_state'),
            },
        ],
        output='screen',
    )

    event_logger = Node(
        package='rosbot_qr_navigation',
        executable='event_logger',
        name='event_logger',
        parameters=[
            config,
            {'log_dir': LaunchConfiguration('log_dir')},
        ],
        output='screen',
    )

    simulation_driver = Node(
        package='rosbot_qr_navigation',
        executable='simulation_driver',
        name='simulation_driver',
        parameters=[
            {
                'script': LaunchConfiguration('script'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'cmd_vel_stamped': LaunchConfiguration('cmd_vel_stamped'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'stop_after_sec': LaunchConfiguration('stop_after_sec'),
                'obstacle_start_sec': LaunchConfiguration('obstacle_start_sec'),
                'obstacle_end_sec': LaunchConfiguration('obstacle_end_sec'),
            },
        ],
        output='screen',
    )

    return LaunchDescription(args + [
        command_interpreter,
        navigation_fsm,
        event_logger,
        simulation_driver,
    ])
