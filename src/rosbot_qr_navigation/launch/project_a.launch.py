"""
Launch file – Project A: QR Code Command Navigation
----------------------------------------------------
Starts all four nodes with parameters loaded from config/params.yaml.

Overridable from CLI:
    image_topic     – camera image (default: /oak/rgb/image_raw)
    cmd_vel_topic   – velocity output (default: /cmd_vel)
    cmd_vel_stamped – publish geometry_msgs/TwistStamped (default: true)
    scan_topic      – LaserScan input for obstacle avoidance (default: /scan)
    show_debug      – show OpenCV window (default: false)
    start_state     – initial FSM state (default: DRIVING)
    stop_after_turn – stop after TURN_LEFT/RIGHT/U_TURN (default: true)
    continuous_obstacle_avoidance – react to obstacles while driving (default: true)
    obstacle_safety_enabled – enable LIDAR/depth obstacle layer (default: true)
    obstacle_stop_only – stop instead of timed side-step avoidance (default: true)
    log_dir         – CSV log output directory

Example (real ROSbot OAK-D Pro):
    ros2 launch rosbot_qr_navigation project_a.launch.py

Example (webcam test, no robot):
    ros2 launch rosbot_qr_navigation project_a.launch.py \
        image_topic:=/image_raw \
        cmd_vel_topic:=/dummy_cmd_vel \
        show_debug:=true \
        start_state:=STOPPED
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('rosbot_qr_navigation')
    config = PathJoinSubstitution([pkg, 'config', 'params.yaml'])

    # ── Declare overridable arguments ─────────────────────────────────
    args = [
        DeclareLaunchArgument('image_topic',   default_value='/oak/rgb/image_raw'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('cmd_vel_stamped', default_value='true'),
        DeclareLaunchArgument('cmd_vel_frame_id', default_value='base_link'),
        DeclareLaunchArgument('scan_topic',    default_value='/scan'),
        DeclareLaunchArgument('show_debug',    default_value='false'),
        DeclareLaunchArgument('start_state',   default_value='DRIVING'),
        DeclareLaunchArgument('stop_after_turn', default_value='true'),
        DeclareLaunchArgument('continuous_obstacle_avoidance', default_value='true'),
        DeclareLaunchArgument('obstacle_safety_enabled', default_value='true'),
        DeclareLaunchArgument('obstacle_stop_only', default_value='true'),
        DeclareLaunchArgument('obstacle_distance', default_value='0.30'),
        DeclareLaunchArgument('obstacle_front_angle_deg', default_value='15.0'),
        DeclareLaunchArgument('obstacle_confirm_sec', default_value='0.35'),
        DeclareLaunchArgument('obstacle_min_points', default_value='5'),
        DeclareLaunchArgument('obstacle_percentile', default_value='10.0'),
        DeclareLaunchArgument('avoid_turn_sec', default_value='3.14'),
        DeclareLaunchArgument('avoid_forward_sec', default_value='1.5'),
        DeclareLaunchArgument('avoid_pass_sec', default_value='1.2'),
        DeclareLaunchArgument('avoid_return_sec', default_value='1.5'),
        DeclareLaunchArgument('avoid_return_to_path', default_value='true'),
        DeclareLaunchArgument('log_dir',       default_value='~/rosbot_qr_logs'),
        DeclareLaunchArgument('imu_topic',     default_value='/imu/data'),
        DeclareLaunchArgument('tof_topic',     default_value='/range'),
        DeclareLaunchArgument('depth_topic',   default_value='/camera/depth/image_rect_raw'),
        DeclareLaunchArgument('use_imu_for_turns', default_value='true'),
    ]

    # ── Nodes ─────────────────────────────────────────────────────────
    qr_detector = Node(
        package='rosbot_qr_navigation',
        executable='qr_detector',
        name='qr_detector',
        parameters=[
            config,
            {
                'image_topic': LaunchConfiguration('image_topic'),
                'show_debug':  LaunchConfiguration('show_debug'),
            },
        ],
        output='screen',
    )

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
                'cmd_vel_topic':     LaunchConfiguration('cmd_vel_topic'),
                'cmd_vel_stamped':   LaunchConfiguration('cmd_vel_stamped'),
                'cmd_vel_frame_id':  LaunchConfiguration('cmd_vel_frame_id'),
                'scan_topic':        LaunchConfiguration('scan_topic'),
                'start_state':       LaunchConfiguration('start_state'),
                'stop_after_turn':   LaunchConfiguration('stop_after_turn'),
                'continuous_obstacle_avoidance': LaunchConfiguration(
                    'continuous_obstacle_avoidance'
                ),
                'obstacle_safety_enabled': LaunchConfiguration(
                    'obstacle_safety_enabled'
                ),
                'obstacle_stop_only': LaunchConfiguration('obstacle_stop_only'),
                'obstacle_distance': LaunchConfiguration('obstacle_distance'),
                'obstacle_front_angle_deg': LaunchConfiguration(
                    'obstacle_front_angle_deg'
                ),
                'obstacle_confirm_sec': LaunchConfiguration('obstacle_confirm_sec'),
                'obstacle_min_points': LaunchConfiguration('obstacle_min_points'),
                'obstacle_percentile': LaunchConfiguration('obstacle_percentile'),
                'avoid_turn_sec': LaunchConfiguration('avoid_turn_sec'),
                'avoid_forward_sec': LaunchConfiguration('avoid_forward_sec'),
                'avoid_pass_sec': LaunchConfiguration('avoid_pass_sec'),
                'avoid_return_sec': LaunchConfiguration('avoid_return_sec'),
                'avoid_return_to_path': LaunchConfiguration('avoid_return_to_path'),
                'imu_topic':         LaunchConfiguration('imu_topic'),
                'tof_topic':         LaunchConfiguration('tof_topic'),
                'depth_topic':       LaunchConfiguration('depth_topic'),
                'use_imu_for_turns': LaunchConfiguration('use_imu_for_turns'),
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

    return LaunchDescription(args + [
        qr_detector,
        command_interpreter,
        navigation_fsm,
        event_logger,
    ])
