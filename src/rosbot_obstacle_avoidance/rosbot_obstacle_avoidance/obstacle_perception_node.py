"""
Project C fused obstacle perception.

This node converts the current S2 LIDAR scan, OAK-D depth stream/point cloud,
and VL53L0X ToF range into one JSON obstacle representation. Keeping perception separate
from control makes the reactive policy easy to inspect and supports the
LIDAR-only vs. LIDAR+depth ablation required for evaluation.
"""

import json
import math
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Image, LaserScan, PointCloud2, Range
from std_msgs.msg import String

try:
    from sensor_msgs_py import point_cloud2
except Exception:  # pragma: no cover - only happens outside ROS installations.
    point_cloud2 = None

try:
    import tf2_ros
except Exception:  # pragma: no cover - only happens outside ROS installations.
    tf2_ros = None


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def _json_float(value: float) -> float | None:
    return float(value) if math.isfinite(value) else None


def _min_finite(*values: float) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return min(finite) if finite else math.inf


def _split_topics(value) -> list[str]:
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        items = value
    else:
        items = str(value).split(',')
    return [str(item).strip() for item in items if str(item).strip()]


@dataclass
class GapTarget:
    angle: float
    clearance: float
    width: int


class ObstaclePerceptionNode(Node):

    def __init__(self):
        super().__init__('obstacle_perception')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('depth_topic', '/oak/stereo/image_raw')
        self.declare_parameter('pointcloud_topic', '/oak/points')
        self.declare_parameter('tof_topic', '/range')
        self.declare_parameter('tof_topics', '/range/fl,/range/fr,/range/rl,/range/rr')
        self.declare_parameter('tof_msg_type', 'scan')
        self.declare_parameter('front_tof_topics', '/range/fl,/range/fr')
        self.declare_parameter('rear_tof_topics', '/range/rl,/range/rr')
        self.declare_parameter('front_tof_obstacle_distance', 0.30)
        self.declare_parameter('front_tof_hard_distance', 0.12)
        self.declare_parameter('obstacle_topic', '/obstacle_representation')
        self.declare_parameter('use_lidar', True)
        self.declare_parameter('use_depth', True)
        self.declare_parameter('use_pointcloud', False)
        self.declare_parameter('use_tof', True)
        self.declare_parameter('publish_hz', 20.0)

        self.declare_parameter('emergency_distance', 0.10)
        self.declare_parameter('obstacle_distance', 0.35)
        self.declare_parameter('clear_distance', 0.35)
        self.declare_parameter('front_center_angle_deg', 0.0)
        self.declare_parameter('front_angle_deg', 30.0)
        self.declare_parameter('front_percentile', 15.0)
        self.declare_parameter('robot_half_width_m', 0.13)
        self.declare_parameter('front_path_half_width_m', 0.16)
        self.declare_parameter('side_guard_forward_m', 0.35)
        self.declare_parameter('side_guard_rear_m', 0.20)
        self.declare_parameter('side_percentile', 10.0)
        self.declare_parameter('front_close_min_rays', 3)
        self.declare_parameter('front_close_min_ratio', 0.01)
        self.declare_parameter('rear_angle_deg', 35.0)
        self.declare_parameter('gap_angle_limit_deg', 110.0)
        self.declare_parameter('sensor_stale_sec', 1.0)

        self.declare_parameter('depth_obstacle_distance', 0.35)
        self.declare_parameter('depth_center_fraction', 0.33)
        self.declare_parameter('depth_side_fraction', 0.30)
        self.declare_parameter('depth_height_fraction', 0.40)
        self.declare_parameter('depth_percentile', 50.0)
        self.declare_parameter('depth_qos', 'auto')
        self.declare_parameter('pointcloud_frame', 'optical')
        self.declare_parameter('pointcloud_target_frame', 'base_link')
        self.declare_parameter('pointcloud_use_tf', True)
        self.declare_parameter('pointcloud_tf_timeout_sec', 0.03)
        self.declare_parameter('pointcloud_qos', 'auto')
        self.declare_parameter('pointcloud_percentile', 10.0)
        self.declare_parameter('pointcloud_center_half_width_m', 0.35)
        self.declare_parameter('pointcloud_side_min_abs_m', 0.15)
        self.declare_parameter('pointcloud_side_max_abs_m', 0.75)
        self.declare_parameter('pointcloud_vertical_abs_m', 0.75)
        self.declare_parameter('pointcloud_low_min_height_m', -0.90)
        self.declare_parameter('pointcloud_low_max_height_m', 0.70)
        self.declare_parameter('pointcloud_low_center_half_width_m', 0.32)
        self.declare_parameter('pointcloud_low_min_points', 1)
        self.declare_parameter('pointcloud_min_forward_m', 0.02)
        self.declare_parameter('pointcloud_max_forward_m', 4.00)
        self.declare_parameter('pointcloud_sample_step', 1)
        self.declare_parameter('pointcloud_roi_stride_px', 6)
        self.declare_parameter('pointcloud_process_hz', 8.0)
        self.declare_parameter('pointcloud_unorganized_max_points', 8000)
        self.declare_parameter('dynamic_obstacle_distance', 1.0)
        self.declare_parameter('dynamic_closing_speed', 0.80)
        self.declare_parameter('dynamic_confirm_sec', 0.10)
        self.declare_parameter('front_filter_alpha', 0.35)
        self.declare_parameter('obstacle_hold_sec', 0.35)
        self.declare_parameter('clear_confirm_sec', 0.20)

        # OAK stereo-depth dynamic fallback. The first depth ROI above can miss
        # a foot because it looks around the image centre. This lower-centre ROI
        # compares consecutive depth frames so a moving leg/person in front of
        # the robot becomes a dynamic obstacle even when the centre depth value
        # still sees the background.
        self.declare_parameter('depth_motion_enabled', True)
        self.declare_parameter('depth_motion_y_center', 0.68)
        self.declare_parameter('depth_motion_width_fraction', 0.45)
        self.declare_parameter('depth_motion_height_fraction', 0.38)
        self.declare_parameter('depth_motion_delta_m', 0.10)
        self.declare_parameter('depth_motion_near_m', 1.20)
        self.declare_parameter('depth_motion_min_ratio', 0.015)
        self.declare_parameter('depth_motion_confirm_frames', 2)
        # Suppress depth_motion when the robot itself is rotating or backing up.
        # Any |angular.z| above this (rad/s) counts as "robot rotating".
        self.declare_parameter('depth_motion_ego_suppress_ang', 0.10)
        # Any linear.x below this (m/s, negative = backward) counts as backing.
        self.declare_parameter('depth_motion_ego_suppress_lin', -0.005)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_stamped', True)

        # Low-region static depth check: catches close obstacles that appear
        # below the centre ROI (y > 0.5 in image = physically lower in scene).
        self.declare_parameter('depth_low_enabled', True)
        self.declare_parameter('depth_low_y_center', 0.78)
        self.declare_parameter('depth_low_width_fraction', 0.36)
        self.declare_parameter('depth_low_height_fraction', 0.18)

        scan_topic = self.get_parameter('scan_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        self._depth_topic = depth_topic
        pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self._pointcloud_topic = pointcloud_topic
        tof_topic = self.get_parameter('tof_topic').value
        tof_topics = _split_topics(self.get_parameter('tof_topics').value)
        if not tof_topics:
            tof_topics = _split_topics(tof_topic)
        tof_msg_type = str(self.get_parameter('tof_msg_type').value).strip().lower()
        if tof_msg_type not in ('scan', 'laserscan', 'range'):
            self.get_logger().warn(
                f'Unknown tof_msg_type={tof_msg_type!r}; using scan.'
            )
            tof_msg_type = 'scan'
        self._front_tof_topics = _split_topics(
            self.get_parameter('front_tof_topics').value
        )
        self._rear_tof_topics = _split_topics(
            self.get_parameter('rear_tof_topics').value
        )
        if not self._front_tof_topics:
            self._front_tof_topics = list(tof_topics)
        obstacle_topic = self.get_parameter('obstacle_topic').value

        self._use_lidar = _as_bool(self.get_parameter('use_lidar').value)
        self._use_depth = _as_bool(self.get_parameter('use_depth').value)
        self._use_pointcloud = _as_bool(
            self.get_parameter('use_pointcloud').value
        )
        self._use_tof = _as_bool(self.get_parameter('use_tof').value)
        publish_hz = float(self.get_parameter('publish_hz').value)

        self._emergency_distance = float(
            self.get_parameter('emergency_distance').value
        )
        self._obstacle_distance = float(self.get_parameter('obstacle_distance').value)
        self._clear_distance = float(self.get_parameter('clear_distance').value)
        self._front_center_angle = math.radians(
            float(self.get_parameter('front_center_angle_deg').value)
        )
        self._front_angle = math.radians(
            float(self.get_parameter('front_angle_deg').value)
        )
        self._front_percentile = max(
            0.0,
            min(100.0, float(self.get_parameter('front_percentile').value)),
        )
        self._robot_half_width_m = max(
            0.0,
            float(self.get_parameter('robot_half_width_m').value),
        )
        self._front_path_half_width_m = max(
            self._robot_half_width_m,
            float(self.get_parameter('front_path_half_width_m').value),
        )
        self._side_guard_forward_m = max(
            0.0,
            float(self.get_parameter('side_guard_forward_m').value),
        )
        self._side_guard_rear_m = max(
            0.0,
            float(self.get_parameter('side_guard_rear_m').value),
        )
        self._side_percentile = max(
            0.0,
            min(100.0, float(self.get_parameter('side_percentile').value)),
        )
        self._front_close_min_rays = max(
            1,
            int(self.get_parameter('front_close_min_rays').value),
        )
        self._front_close_min_ratio = max(
            0.0,
            min(1.0, float(self.get_parameter('front_close_min_ratio').value)),
        )
        self._rear_angle = math.radians(
            float(self.get_parameter('rear_angle_deg').value)
        )
        self._gap_angle_limit = math.radians(
            float(self.get_parameter('gap_angle_limit_deg').value)
        )
        self._sensor_stale_sec = float(self.get_parameter('sensor_stale_sec').value)
        self._depth_obstacle_distance = float(
            self.get_parameter('depth_obstacle_distance').value
        )
        self._front_tof_obstacle_distance = max(
            0.0,
            float(self.get_parameter('front_tof_obstacle_distance').value),
        )
        self._front_tof_hard_distance = max(
            0.0,
            min(
                float(self.get_parameter('front_tof_hard_distance').value),
                self._front_tof_obstacle_distance,
            ),
        )
        self._depth_center_fraction = float(
            self.get_parameter('depth_center_fraction').value
        )
        self._depth_side_fraction = float(
            self.get_parameter('depth_side_fraction').value
        )
        self._depth_height_fraction = float(
            self.get_parameter('depth_height_fraction').value
        )
        self._depth_percentile = max(
            0.0,
            min(100.0, float(self.get_parameter('depth_percentile').value)),
        )
        self._depth_qos = str(
            self.get_parameter('depth_qos').value
        ).strip().lower().replace('-', '_')
        self._pointcloud_frame = str(
            self.get_parameter('pointcloud_frame').value
        ).strip().lower()
        self._pointcloud_target_frame = str(
            self.get_parameter('pointcloud_target_frame').value
        ).strip()
        self._pointcloud_use_tf = _as_bool(
            self.get_parameter('pointcloud_use_tf').value
        )
        self._pointcloud_tf_timeout = max(
            0.0,
            float(self.get_parameter('pointcloud_tf_timeout_sec').value),
        )
        self._pointcloud_qos = str(
            self.get_parameter('pointcloud_qos').value
        ).strip().lower()
        self._pointcloud_percentile = max(
            0.0,
            min(100.0, float(self.get_parameter('pointcloud_percentile').value)),
        )
        self._pointcloud_center_half_width_m = float(
            self.get_parameter('pointcloud_center_half_width_m').value
        )
        self._pointcloud_side_min_abs_m = float(
            self.get_parameter('pointcloud_side_min_abs_m').value
        )
        self._pointcloud_side_max_abs_m = float(
            self.get_parameter('pointcloud_side_max_abs_m').value
        )
        self._pointcloud_vertical_abs_m = float(
            self.get_parameter('pointcloud_vertical_abs_m').value
        )
        self._pointcloud_low_min_height_m = float(
            self.get_parameter('pointcloud_low_min_height_m').value
        )
        self._pointcloud_low_max_height_m = float(
            self.get_parameter('pointcloud_low_max_height_m').value
        )
        self._pointcloud_low_center_half_width_m = max(
            0.0,
            float(self.get_parameter('pointcloud_low_center_half_width_m').value),
        )
        self._pointcloud_low_min_points = max(
            1,
            int(self.get_parameter('pointcloud_low_min_points').value),
        )
        self._pointcloud_min_forward_m = float(
            self.get_parameter('pointcloud_min_forward_m').value
        )
        self._pointcloud_max_forward_m = float(
            self.get_parameter('pointcloud_max_forward_m').value
        )
        self._pointcloud_sample_step = max(
            1,
            int(self.get_parameter('pointcloud_sample_step').value),
        )
        self._pointcloud_roi_stride_px = max(
            1,
            int(self.get_parameter('pointcloud_roi_stride_px').value),
        )
        self._pointcloud_process_period = 1.0 / max(
            0.1,
            float(self.get_parameter('pointcloud_process_hz').value),
        )
        self._pointcloud_unorganized_max_points = max(
            100,
            int(self.get_parameter('pointcloud_unorganized_max_points').value),
        )
        self._dynamic_obstacle_distance = float(
            self.get_parameter('dynamic_obstacle_distance').value
        )
        self._dynamic_closing_speed = float(
            self.get_parameter('dynamic_closing_speed').value
        )
        self._dynamic_confirm_sec = float(
            self.get_parameter('dynamic_confirm_sec').value
        )
        self._front_filter_alpha = float(
            self.get_parameter('front_filter_alpha').value
        )
        self._obstacle_hold_sec = float(
            self.get_parameter('obstacle_hold_sec').value
        )
        self._clear_confirm_sec = float(
            self.get_parameter('clear_confirm_sec').value
        )
        self._depth_motion_enabled = _as_bool(
            self.get_parameter('depth_motion_enabled').value
        )
        self._depth_motion_y_center = float(
            self.get_parameter('depth_motion_y_center').value
        )
        self._depth_motion_width_fraction = float(
            self.get_parameter('depth_motion_width_fraction').value
        )
        self._depth_motion_height_fraction = float(
            self.get_parameter('depth_motion_height_fraction').value
        )
        self._depth_motion_delta_m = float(
            self.get_parameter('depth_motion_delta_m').value
        )
        self._depth_motion_near_m = float(
            self.get_parameter('depth_motion_near_m').value
        )
        self._depth_motion_min_ratio = float(
            self.get_parameter('depth_motion_min_ratio').value
        )
        self._depth_motion_confirm_frames = max(
            1, int(self.get_parameter('depth_motion_confirm_frames').value)
        )
        self._depth_motion_ego_suppress_ang = float(
            self.get_parameter('depth_motion_ego_suppress_ang').value
        )
        self._depth_motion_ego_suppress_lin = float(
            self.get_parameter('depth_motion_ego_suppress_lin').value
        )
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self._cmd_vel_stamped = _as_bool(self.get_parameter('cmd_vel_stamped').value)
        self._depth_low_enabled = _as_bool(
            self.get_parameter('depth_low_enabled').value
        )
        self._depth_low_y_center = float(
            self.get_parameter('depth_low_y_center').value
        )
        self._depth_low_width_fraction = float(
            self.get_parameter('depth_low_width_fraction').value
        )
        self._depth_low_height_fraction = float(
            self.get_parameter('depth_low_height_fraction').value
        )

        self._latest_scan: LaserScan | None = None
        self._last_scan_time: float | None = None
        self._depth_front = math.inf
        self._depth_left = math.inf
        self._depth_right = math.inf
        self._depth_low_front = math.inf
        self._last_depth_time: float | None = None
        self._prev_depth_motion_roi = None
        self._depth_motion = False
        self._depth_motion_score = 0.0
        self._depth_motion_front = math.inf
        self._depth_motion_count = 0
        self._cmd_vel_lin_x: float = 0.0
        self._cmd_vel_ang_z: float = 0.0
        self._pointcloud_front = math.inf
        self._pointcloud_low_front = math.inf
        self._pointcloud_low_front_count = 0
        self._pointcloud_sample_count = 0
        self._pointcloud_low_fallback_count = 0
        self._pointcloud_left = math.inf
        self._pointcloud_right = math.inf
        self._last_pointcloud_time: float | None = None
        self._last_pointcloud_process_time = 0.0
        self._start_time = time.monotonic()
        self._tof_range = math.inf
        self._last_tof_time: float | None = None
        self._tof_ranges: dict[str, float] = {}
        self._last_tof_times: dict[str, float] = {}
        self._prev_front_sample: tuple[float, float] | None = None
        self._filtered_front_distance: float | None = None
        self._dynamic_first_seen_time: float | None = None
        self._blocked_latched = False
        self._blocked_until = 0.0
        self._clear_seen_since: float | None = None
        self._blocked_sources: list[str] = []
        self._bridge = CvBridge()
        self._subscriptions = []
        self._tf_buffer = None
        self._tf_listener = None
        self._last_pointcloud_warn = 0.0
        self._last_depth_warn = 0.0
        self._last_depth_stamp_key: tuple[int, int] | None = None
        self._depth_subscription_labels: list[str] = []
        self._pointcloud_subscription_labels: list[str] = []

        if self._use_pointcloud and self._pointcloud_use_tf:
            if tf2_ros is None:
                self.get_logger().warn(
                    'tf2_ros is unavailable; point cloud uses fallback optical/base axes.'
                )
                self._pointcloud_use_tf = False
            else:
                self._tf_buffer = tf2_ros.Buffer()
                self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        if self._use_lidar:
            self._subscriptions.append(
                self.create_subscription(
                    LaserScan,
                    scan_topic,
                    self._on_scan,
                    qos_profile_sensor_data,
                )
            )
        if self._use_depth:
            for label, qos_profile in self._depth_qos_profiles():
                self._subscriptions.append(
                    self.create_subscription(
                        Image,
                        depth_topic,
                        self._on_depth,
                        qos_profile,
                    )
                )
                self._depth_subscription_labels.append(label)
        if self._use_pointcloud:
            if point_cloud2 is None:
                self.get_logger().warn(
                    'sensor_msgs_py.point_cloud2 is unavailable; '
                    'OAK point cloud input disabled.'
                )
                self._use_pointcloud = False
            else:
                for label, qos_profile in self._pointcloud_qos_profiles():
                    self._subscriptions.append(
                        self.create_subscription(
                            PointCloud2,
                            pointcloud_topic,
                            self._on_pointcloud,
                            qos_profile,
                        )
                    )
                    self._pointcloud_subscription_labels.append(label)
        if self._use_tof:
            for topic in tof_topics:
                msg_type = Range if tof_msg_type == 'range' else LaserScan
                callback = (
                    self._on_tof_range
                    if tof_msg_type == 'range'
                    else self._on_tof_scan
                )
                self._subscriptions.append(
                    self.create_subscription(
                        msg_type,
                        topic,
                        lambda msg, topic=topic, callback=callback: callback(
                            msg,
                            topic,
                        ),
                        qos_profile_sensor_data,
                    )
                )

        cmd_vel_msg_type = TwistStamped if self._cmd_vel_stamped else Twist
        self._subscriptions.append(
            self.create_subscription(
                cmd_vel_msg_type,
                cmd_vel_topic,
                self._on_cmd_vel,
                10,
            )
        )

        self._pub = self.create_publisher(String, obstacle_topic, 10)
        self.create_timer(1.0 / max(publish_hz, 1.0), self._publish_representation)

        self.get_logger().info(
            'Obstacle perception ready. '
            f'lidar={scan_topic if self._use_lidar else "disabled"}, '
            f'front_center={math.degrees(self._front_center_angle):.0f}deg, '
            f'depth_image={depth_topic if self._use_depth else "disabled"}, '
            f'depth_qos={self._depth_qos_label()}, '
            f'pointcloud={pointcloud_topic if self._use_pointcloud else "disabled"}, '
            f'pointcloud_qos={self._pointcloud_qos_label()}, '
            f'pointcloud_tf={self._pointcloud_target_frame if self._pointcloud_use_tf else "fallback"}, '
            f'tof={",".join(tof_topics) if self._use_tof else "disabled"} '
            f'({tof_msg_type}), '
            f'out={obstacle_topic}'
        )

    def _depth_qos_label(self) -> str:
        if not self._depth_subscription_labels:
            return self._depth_qos
        if self._depth_qos == 'auto':
            return 'auto[' + ','.join(self._depth_subscription_labels) + ']'
        return self._depth_subscription_labels[0]

    def _depth_qos_profiles(self) -> list[tuple[str, QoSProfile]]:
        if self._depth_qos in ('auto', ''):
            return [
                ('sensor_data', qos_profile_sensor_data),
                ('reliable', self._reliable_volatile_qos()),
            ]
        if self._depth_qos in ('reliable',):
            return [('reliable', self._reliable_volatile_qos())]
        if self._depth_qos not in ('sensor_data', 'best_effort'):
            self.get_logger().warn(
                f'Unknown depth_qos={self._depth_qos!r}; using sensor_data.'
            )
        return [('sensor_data', qos_profile_sensor_data)]

    def _reliable_volatile_qos(self) -> QoSProfile:
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

    def _pointcloud_qos_label(self) -> str:
        if not self._pointcloud_subscription_labels:
            return self._pointcloud_qos
        if self._pointcloud_qos == 'auto':
            return 'auto[' + ','.join(self._pointcloud_subscription_labels) + ']'
        return self._pointcloud_subscription_labels[0]

    def _pointcloud_qos_profiles(self) -> list[tuple[str, QoSProfile]]:
        if self._pointcloud_qos in ('auto', ''):
            return [
                ('sensor_data', qos_profile_sensor_data),
                ('reliable', self._pointcloud_qos_profile('reliable')),
            ]
        return [(self._pointcloud_qos, self._pointcloud_qos_profile(self._pointcloud_qos))]

    def _pointcloud_qos_profile(self, qos_name: str) -> QoSProfile:
        if qos_name in ('reliable_transient_local', 'transient_local'):
            return QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
        if qos_name in ('reliable',):
            return self._reliable_volatile_qos()
        if qos_name not in ('sensor_data', 'best_effort', 'best-effort'):
            self.get_logger().warn(
                f'Unknown pointcloud_qos={qos_name!r}; using sensor_data.'
            )
        return qos_profile_sensor_data

    def _on_scan(self, msg: LaserScan):
        self._latest_scan = msg
        self._last_scan_time = time.monotonic()

    def _on_tof_range(self, msg: Range, topic: str):
        now = time.monotonic()
        self._last_tof_time = now
        self._last_tof_times[topic] = now
        if math.isfinite(msg.range) and msg.min_range <= msg.range < msg.max_range:
            distance = msg.range
        else:
            distance = math.inf
        self._tof_ranges[topic] = distance
        self._tof_range = self._tof_min_recent()

    def _on_tof_scan(self, msg: LaserScan, topic: str):
        now = time.monotonic()
        self._last_tof_time = now
        self._last_tof_times[topic] = now
        values = [
            distance
            for value in msg.ranges
            if (distance := self._normalized_scan_distance(msg, value)) is not None
        ]
        self._tof_ranges[topic] = min(values) if values else math.inf
        self._tof_range = self._tof_min_recent()

    def _on_depth(self, msg: Image):
        stamp_key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp_key == self._last_depth_stamp_key:
            return
        self._last_depth_stamp_key = stamp_key

        try:
            depth_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception:
            return

        self._last_depth_time = time.monotonic()
        self._depth_front = self._depth_roi_distance(
            depth_img,
            msg.encoding,
            x_center=0.50,
            width_fraction=self._depth_center_fraction,
        )
        self._depth_left = self._depth_roi_distance(
            depth_img,
            msg.encoding,
            x_center=0.25,
            width_fraction=self._depth_side_fraction,
        )
        self._depth_right = self._depth_roi_distance(
            depth_img,
            msg.encoding,
            x_center=0.75,
            width_fraction=self._depth_side_fraction,
        )
        if self._depth_low_enabled:
            self._depth_low_front = self._depth_roi_distance(
                depth_img,
                msg.encoding,
                x_center=0.50,
                width_fraction=self._depth_low_width_fraction,
                y_center=self._depth_low_y_center,
                height_fraction=self._depth_low_height_fraction,
            )
        else:
            self._depth_low_front = math.inf
        self._update_depth_motion(depth_img, msg.encoding)

    def _on_pointcloud(self, msg: PointCloud2):
        now = time.monotonic()
        if now - self._last_pointcloud_process_time < self._pointcloud_process_period:
            return
        self._last_pointcloud_process_time = now

        front_vals: list[float] = []
        fallback_front_vals: list[float] = []
        low_front_vals: list[float] = []
        fallback_low_front_vals: list[float] = []
        left_vals: list[float] = []
        right_vals: list[float] = []
        sample_count = 0
        transform = self._pointcloud_transform(msg)

        try:
            points = self._sample_pointcloud(msg)
        except Exception as exc:
            self._warn_pointcloud(f'Point cloud read failed: {exc}')
            return

        for index, point in enumerate(points):
            if (
                    msg.height <= 1
                    and index % self._pointcloud_sample_step != 0):
                continue
            if msg.height <= 1 and index >= self._pointcloud_unorganized_max_points:
                break
            xyz = self._point_xyz(point)
            if xyz is None:
                continue
            x, y, z = xyz
            sample_count += 1

            fallback_forward, fallback_lateral, fallback_vertical = self._pointcloud_axes(
                x, y, z, 'optical'
            )
            if self._pointcloud_axes_usable(
                    fallback_forward,
                    fallback_lateral,
                    fallback_vertical):
                fallback_is_low = (
                    self._pointcloud_low_min_height_m
                    <= fallback_vertical
                    <= self._pointcloud_low_max_height_m
                    and abs(fallback_lateral)
                    <= self._pointcloud_low_center_half_width_m
                )
                if fallback_is_low:
                    fallback_low_front_vals.append(fallback_forward)
                elif abs(fallback_lateral) <= self._pointcloud_center_half_width_m:
                    fallback_front_vals.append(fallback_forward)

            if transform is not None:
                forward, lateral, vertical = self._transform_point_axes(
                    x, y, z, transform
                )
            else:
                forward, lateral, vertical = self._pointcloud_axes(
                    x, y, z, msg.header.frame_id
                )
            if not self._pointcloud_axes_usable(forward, lateral, vertical):
                continue
            is_low = (
                self._pointcloud_low_min_height_m
                <= vertical
                <= self._pointcloud_low_max_height_m
                and abs(lateral) <= self._pointcloud_low_center_half_width_m
            )
            if is_low:
                low_front_vals.append(forward)
            if abs(lateral) <= self._pointcloud_center_half_width_m and not is_low:
                front_vals.append(forward)
            elif (
                    self._pointcloud_side_min_abs_m
                    <= lateral
                    <= self._pointcloud_side_max_abs_m):
                left_vals.append(forward)
            elif (
                    -self._pointcloud_side_max_abs_m
                    <= lateral
                    <= -self._pointcloud_side_min_abs_m):
                right_vals.append(forward)

        self._last_pointcloud_time = now
        general_front = _min_finite(
            self._pointcloud_stat(front_vals),
            self._pointcloud_stat(fallback_front_vals),
        )
        primary_low_count = len(low_front_vals)
        fallback_low_count = len(fallback_low_front_vals)
        primary_low_front = (
            self._pointcloud_stat(low_front_vals)
            if primary_low_count >= self._pointcloud_low_min_points
            else math.inf
        )
        fallback_low_front = (
            self._pointcloud_stat(fallback_low_front_vals)
            if fallback_low_count >= self._pointcloud_low_min_points
            else math.inf
        )
        self._pointcloud_sample_count = sample_count
        self._pointcloud_low_fallback_count = fallback_low_count
        self._pointcloud_low_front_count = max(primary_low_count, fallback_low_count)
        self._pointcloud_low_front = _min_finite(
            primary_low_front,
            fallback_low_front,
        )
        self._pointcloud_front = general_front
        self._pointcloud_left = self._pointcloud_stat(left_vals)
        self._pointcloud_right = self._pointcloud_stat(right_vals)
        if sample_count == 0:
            fields = ','.join(field.name for field in msg.fields)
            self._warn_pointcloud(
                f'Point cloud received but no xyz samples parsed: '
                f'frame={msg.header.frame_id!r}, size={msg.width}x{msg.height}, '
                f'fields={fields}'
            )

    def _pointcloud_axes_usable(
            self,
            forward: float,
            lateral: float,
            vertical: float) -> bool:
        return (
            math.isfinite(forward)
            and math.isfinite(lateral)
            and math.isfinite(vertical)
            and self._pointcloud_min_forward_m <= forward <= self._pointcloud_max_forward_m
            and abs(vertical) <= self._pointcloud_vertical_abs_m
        )

    def _sample_pointcloud(self, msg: PointCloud2):
        if msg.height > 1 and msg.width > 1:
            stride = self._pointcloud_roi_stride_px
            uvs = [
                v * msg.width + u
                for v in range(0, msg.height, stride)
                for u in range(0, msg.width, stride)
            ]
            return point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True,
                uvs=uvs,
            )
        return point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True,
        )

    @staticmethod
    def _point_xyz(point) -> tuple[float, float, float] | None:
        try:
            return float(point[0]), float(point[1]), float(point[2])
        except (IndexError, KeyError, TypeError, ValueError):
            try:
                return float(point['x']), float(point['y']), float(point['z'])
            except (IndexError, KeyError, TypeError, ValueError):
                return None

    def _pointcloud_transform(self, msg: PointCloud2):
        if (
                not self._pointcloud_use_tf
                or self._tf_buffer is None
                or not self._pointcloud_target_frame):
            return None

        source_frame = str(msg.header.frame_id).strip().lstrip('/')
        target_frame = self._pointcloud_target_frame.strip().lstrip('/')
        if not source_frame or source_frame == target_frame:
            return None

        try:
            return self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=self._pointcloud_tf_timeout),
            ).transform
        except Exception as exc:
            self._warn_pointcloud(
                f'Point cloud TF {source_frame}->{target_frame} unavailable: {exc}'
            )
            return None

    def _warn_pointcloud(self, text: str):
        now = time.monotonic()
        if now - self._last_pointcloud_warn >= 2.0:
            self._last_pointcloud_warn = now
            self.get_logger().warn(text)

    def _warn_depth(self, text: str):
        now = time.monotonic()
        if now - self._last_depth_warn >= 2.0:
            self._last_depth_warn = now
            self.get_logger().warn(text)

    @staticmethod
    def _transform_point_axes(x: float, y: float, z: float, transform) -> tuple[float, float, float]:
        q = transform.rotation
        t = transform.translation
        tx = 2.0 * (q.y * z - q.z * y)
        ty = 2.0 * (q.z * x - q.x * z)
        tz = 2.0 * (q.x * y - q.y * x)
        rx = x + q.w * tx + (q.y * tz - q.z * ty)
        ry = y + q.w * ty + (q.z * tx - q.x * tz)
        rz = z + q.w * tz + (q.x * ty - q.y * tx)
        return rx + t.x, ry + t.y, rz + t.z

    def _pointcloud_axes(
            self,
            x: float,
            y: float,
            z: float,
            frame_id: str = '') -> tuple[float, float, float]:
        frame = self._pointcloud_frame
        if frame == 'auto':
            frame = str(frame_id).strip().lower()
        if frame in ('base', 'base_link', 'ros') or frame.endswith('base_link'):
            return x, y, z
        return z, -x, -y

    def _pointcloud_stat(self, values: list[float]) -> float:
        if not values:
            return math.inf
        return float(np.percentile(values, self._pointcloud_percentile))

    def _depth_roi_values_m(
            self,
            depth_img,
            encoding: str,
            x_center: float,
            y_center: float,
            width_fraction: float,
            height_fraction: float):
        """Return valid depth ROI values in metres."""
        h, w = depth_img.shape[:2]
        roi_w = max(1, int(w * width_fraction))
        roi_h = max(1, int(h * height_fraction))
        cx = int(w * max(0.0, min(1.0, x_center)))
        cy = int(h * max(0.0, min(1.0, y_center)))
        x0 = max(0, cx - roi_w // 2)
        x1 = min(w, cx + roi_w // 2)
        y0 = max(0, cy - roi_h // 2)
        y1 = min(h, cy + roi_h // 2)
        roi = depth_img[y0:y1, x0:x1]
        arr = roi.astype('float32', copy=False)
        encoding_l = encoding.lower()
        if not (
                '32f' in encoding_l
                or '64f' in encoding_l
                or np.issubdtype(roi.dtype, np.floating)):
            arr = arr / 1000.0
        valid = arr[np.isfinite(arr) & (arr > 0.02)]
        return arr, valid

    def _on_cmd_vel(self, msg):
        twist = msg.twist if hasattr(msg, 'twist') else msg
        self._cmd_vel_lin_x = float(twist.linear.x)
        self._cmd_vel_ang_z = float(twist.angular.z)

    def _update_depth_motion(self, depth_img, encoding: str):
        if not self._depth_motion_enabled:
            self._depth_motion = False
            self._depth_motion_score = 0.0
            self._depth_motion_front = math.inf
            self._prev_depth_motion_roi = None
            self._depth_motion_count = 0
            return

        # When the robot itself is rotating or backing up the depth camera
        # sweeps across new scene content — every pixel looks "moved".  Reset
        # the reference frame so we only compare frames from when the robot is
        # stationary or moving slowly forward.
        robot_rotating = abs(self._cmd_vel_ang_z) > self._depth_motion_ego_suppress_ang
        robot_backing = self._cmd_vel_lin_x < self._depth_motion_ego_suppress_lin
        if robot_rotating or robot_backing:
            self._prev_depth_motion_roi = None
            self._depth_motion = False
            self._depth_motion_score = 0.0
            self._depth_motion_count = 0
            return

        roi, valid = self._depth_roi_values_m(
            depth_img,
            encoding,
            x_center=0.50,
            y_center=self._depth_motion_y_center,
            width_fraction=self._depth_motion_width_fraction,
            height_fraction=self._depth_motion_height_fraction,
        )
        if valid.size == 0:
            self._depth_motion = False
            self._depth_motion_score = 0.0
            self._depth_motion_front = math.inf
            self._prev_depth_motion_roi = None
            self._depth_motion_count = 0
            return

        near_front = float(np.percentile(valid, 10))
        self._depth_motion_front = near_front

        if self._prev_depth_motion_roi is None or self._prev_depth_motion_roi.shape != roi.shape:
            self._prev_depth_motion_roi = roi.copy()
            self._depth_motion = False
            self._depth_motion_score = 0.0
            self._depth_motion_count = 0
            return

        prev = self._prev_depth_motion_roi
        valid_pair = (
            np.isfinite(roi)
            & np.isfinite(prev)
            & (roi > 0.02)
            & (prev > 0.02)
        )
        near_pair = valid_pair & ((roi < self._depth_motion_near_m) | (prev < self._depth_motion_near_m))
        changed = near_pair & (np.abs(roi - prev) >= self._depth_motion_delta_m)
        denom = max(int(near_pair.sum()), 1)
        motion_ratio = float(changed.sum()) / denom
        self._depth_motion_score = motion_ratio
        raw_motion = (
            near_front <= self._depth_motion_near_m
            and motion_ratio >= self._depth_motion_min_ratio
        )
        if raw_motion:
            self._depth_motion_count += 1
        else:
            self._depth_motion_count = 0
        self._depth_motion = self._depth_motion_count >= self._depth_motion_confirm_frames
        self._prev_depth_motion_roi = roi.copy()

    def _depth_roi_distance(
            self,
            depth_img,
            encoding: str,
            x_center: float,
            width_fraction: float,
            y_center: float = 0.5,
            height_fraction: float | None = None) -> float:
        h, w = depth_img.shape[:2]
        hf = height_fraction if height_fraction is not None else self._depth_height_fraction
        roi_w = max(1, int(w * width_fraction))
        roi_h = max(1, int(h * hf))
        cx = int(w * x_center)
        cy = int(h * y_center)
        x0 = max(0, cx - roi_w // 2)
        x1 = min(w, cx + roi_w // 2)
        y0 = max(0, cy - roi_h // 2)
        y1 = min(h, cy + roi_h // 2)
        roi = depth_img[y0:y1, x0:x1]

        valid = roi[np.isfinite(roi) & (roi > 0)]
        if valid.size == 0:
            return math.inf

        depth_value = float(np.percentile(valid, self._depth_percentile))
        encoding_l = encoding.lower()
        if (
                '32f' in encoding_l
                or '64f' in encoding_l
                or np.issubdtype(valid.dtype, np.floating)):
            return depth_value
        return depth_value / 1000.0

    def _sensor_recent(self, stamp: float | None) -> bool:
        return stamp is not None and time.monotonic() - stamp <= self._sensor_stale_sec

    def _publish_representation(self):
        now = time.monotonic()
        wall_stamp = time.time()
        scan_recent = self._use_lidar and self._sensor_recent(self._last_scan_time)
        depth_image_recent = self._use_depth and self._sensor_recent(
            self._last_depth_time
        )
        pointcloud_recent = self._use_pointcloud and self._sensor_recent(
            self._last_pointcloud_time
        )
        if (
                self._use_pointcloud
                and self._last_pointcloud_time is None
                and now - self._start_time >= 5.0):
            self._warn_pointcloud(
                f'No PointCloud2 messages received from {self._pointcloud_topic}. '
                'Check husarion-depthai, or run ros2 topic list -t | grep PointCloud2.'
            )
        if (
                self._use_depth
                and self._last_depth_time is None
                and now - self._start_time >= 5.0):
            self._warn_depth(
                f'No depth Image messages received from {self._depth_topic}. '
                'Check CYCLONEDDS_URI or ros2 topic hz '
                f'{self._depth_topic}.'
            )
        depth_recent = depth_image_recent or pointcloud_recent
        front_tof_range = self._tof_min_recent(self._front_tof_topics)
        rear_tof_range = self._tof_min_recent(self._rear_tof_topics)
        front_tof_fusion = (
            front_tof_range
            if front_tof_range <= self._front_tof_obstacle_distance
            else math.inf
        )
        tof_range = self._tof_min_recent()
        tof_recent = self._use_tof and (
            math.isfinite(front_tof_range)
            or math.isfinite(rear_tof_range)
            or math.isfinite(tof_range)
        )

        lidar_front = math.inf
        lidar_front_control = math.inf
        lidar_front_mean = math.inf
        lidar_front_samples = 0
        lidar_front_close_count = 0
        lidar_front_emergency_count = 0
        lidar_left = math.inf
        lidar_right = math.inf
        lidar_rear = math.inf
        best_gap: GapTarget | None = None
        if scan_recent:
            (
                lidar_front,
                lidar_front_control,
                lidar_front_mean,
                lidar_front_samples,
                lidar_front_close_count,
                lidar_front_emergency_count,
                lidar_left,
                lidar_right,
                lidar_rear,
                best_gap,
            ) = self._process_scan()

        depth_image_front = self._depth_front if depth_image_recent else math.inf
        depth_image_left = self._depth_left if depth_image_recent else math.inf
        depth_image_right = self._depth_right if depth_image_recent else math.inf
        pointcloud_front = self._pointcloud_front if pointcloud_recent else math.inf
        pointcloud_low_front = (
            self._pointcloud_low_front if pointcloud_recent else math.inf
        )
        pointcloud_low_front_count = (
            self._pointcloud_low_front_count if pointcloud_recent else 0
        )
        pointcloud_sample_count = (
            self._pointcloud_sample_count if pointcloud_recent else 0
        )
        pointcloud_low_fallback_count = (
            self._pointcloud_low_fallback_count if pointcloud_recent else 0
        )
        pointcloud_left = self._pointcloud_left if pointcloud_recent else math.inf
        pointcloud_right = self._pointcloud_right if pointcloud_recent else math.inf
        depth_low_front = self._depth_low_front if depth_image_recent else math.inf
        depth_front = _min_finite(depth_image_front, pointcloud_front)
        depth_motion_active = bool(depth_image_recent and self._depth_motion)
        if depth_motion_active:
            depth_front = _min_finite(depth_front, self._depth_motion_front)
        depth_left = _min_finite(depth_image_left, pointcloud_left)
        depth_right = _min_finite(depth_image_right, pointcloud_right)

        front_distance = _min_finite(
            lidar_front_control,
            depth_front,
            front_tof_fusion,
        )
        left_distance = _min_finite(lidar_left, depth_left)
        right_distance = _min_finite(lidar_right, depth_right)
        rear_distance = _min_finite(lidar_rear, rear_tof_range)

        front_close_ratio = (
            lidar_front_close_count / lidar_front_samples
            if lidar_front_samples > 0
            else 0.0
        )
        front_emergency_ratio = (
            lidar_front_emergency_count / lidar_front_samples
            if lidar_front_samples > 0
            else 0.0
        )
        front_close_cluster = (
            lidar_front_close_count >= self._front_close_min_rays
            and front_close_ratio >= self._front_close_min_ratio
        )
        front_emergency_cluster = (
            lidar_front_emergency_count >= self._front_close_min_rays
            and front_emergency_ratio >= self._front_close_min_ratio
        )

        blocked_lidar = (
            lidar_front_control < self._obstacle_distance
            or front_close_cluster
        )
        blocked_depth = depth_front < self._depth_obstacle_distance
        blocked_tof = front_tof_fusion < self._clear_distance
        lidar_emergency = (
            lidar_front_control < self._emergency_distance
            or front_emergency_cluster
        )
        depth_emergency = depth_front < self._emergency_distance
        tof_emergency = front_tof_range <= self._front_tof_hard_distance
        raw_blocked = blocked_lidar or blocked_depth or blocked_tof
        raw_blocked_sources = []
        if blocked_lidar:
            raw_blocked_sources.append('lidar')
        if blocked_depth:
            raw_blocked_sources.append('depth')
        if blocked_tof:
            raw_blocked_sources.append('tof')

        blocked, blocked_held = self._apply_blocked_hysteresis(
            raw_blocked,
            front_distance >= self._clear_distance,
            now,
            raw_blocked_sources,
        )
        emergency = lidar_emergency or depth_emergency or tof_emergency
        dead_end = blocked and (
            best_gap is None
            or (
                left_distance < self._obstacle_distance
                and right_distance < self._obstacle_distance
            )
        )
        dynamic_obstacle, closing_speed = self._dynamic_obstacle(front_distance, now)
        dynamic_obstacle = bool(dynamic_obstacle or depth_motion_active)

        sources = []
        if blocked_lidar or lidar_emergency or (
                blocked and not raw_blocked_sources and 'lidar' in self._blocked_sources):
            sources.append('lidar')
        if blocked_depth or depth_emergency or depth_motion_active or (
                blocked and not raw_blocked_sources and 'depth' in self._blocked_sources):
            sources.append('depth')
        if blocked_tof or tof_emergency:
            sources.append('tof')

        rep = {
            'stamp': wall_stamp,
            'ages': {
                'scan': self._age(self._last_scan_time),
                'depth': self._min_age(
                    self._last_depth_time,
                    self._last_pointcloud_time,
                ),
                'depth_image': self._age(self._last_depth_time),
                'pointcloud': self._age(self._last_pointcloud_time),
                'tof': self._age(self._last_tof_time),
            },
            'lidar': {
                'available': bool(scan_recent),
                'front_min': _json_float(lidar_front),
                'front_control': _json_float(lidar_front_control),
                'front_mean': _json_float(lidar_front_mean),
                'front_samples': int(lidar_front_samples),
                'front_close_count': int(lidar_front_close_count),
                'front_close_ratio': _json_float(front_close_ratio),
                'front_emergency_count': int(lidar_front_emergency_count),
                'left_mean': _json_float(lidar_left),
                'right_mean': _json_float(lidar_right),
                'left_body_clearance': _json_float(lidar_left),
                'right_body_clearance': _json_float(lidar_right),
                'rear_mean': _json_float(lidar_rear),
                'rear_control': _json_float(lidar_rear),
                'best_gap': self._gap_json(best_gap),
            },
            'depth': {
                'available': bool(depth_recent),
                'image_available': bool(depth_image_recent),
                'pointcloud_available': bool(pointcloud_recent),
                'front_min': _json_float(depth_front),
                'left_min': _json_float(depth_left),
                'right_min': _json_float(depth_right),
                'image_front_min': _json_float(depth_image_front),
                'image_low_front_min': _json_float(depth_low_front),
                'image_low_roi': {
                    'enabled': bool(self._depth_low_enabled),
                    'y_center': _json_float(self._depth_low_y_center),
                    'width_fraction': _json_float(self._depth_low_width_fraction),
                    'height_fraction': _json_float(self._depth_low_height_fraction),
                },
                'pointcloud_front_min': _json_float(pointcloud_front),
                'pointcloud_low_front_min': _json_float(pointcloud_low_front),
                'pointcloud_low_front_count': int(pointcloud_low_front_count),
                'pointcloud_sample_count': int(pointcloud_sample_count),
                'pointcloud_low_fallback_count': int(pointcloud_low_fallback_count),
                'pointcloud_low_height_min': _json_float(
                    self._pointcloud_low_min_height_m
                ),
                'pointcloud_low_height_max': _json_float(
                    self._pointcloud_low_max_height_m
                ),
                'pointcloud_left_min': _json_float(pointcloud_left),
                'pointcloud_right_min': _json_float(pointcloud_right),
                'motion': bool(depth_motion_active),
                'motion_score': _json_float(self._depth_motion_score),
                'motion_front_min': _json_float(self._depth_motion_front),
            },
            'tof': {
                'available': bool(tof_recent),
                'range': _json_float(front_tof_range),
                'front_range': _json_float(front_tof_range),
                'front_fusion_range': _json_float(front_tof_fusion),
                'rear_range': _json_float(rear_tof_range),
                'topics': {
                    topic: _json_float(distance)
                    for topic, distance in self._tof_ranges.items()
                },
            },
            'fused': {
                'front_distance': _json_float(front_distance),
                'effective_front_distance': _json_float(front_distance),
                'left_distance': _json_float(left_distance),
                'right_distance': _json_float(right_distance),
                'rear_distance': _json_float(rear_distance),
                'blocked': bool(blocked),
                'blocked_raw': bool(raw_blocked),
                'blocked_held': bool(blocked_held),
                'clear': bool(front_distance >= self._clear_distance),
                'emergency': bool(emergency),
                'dead_end': bool(dead_end),
                'dynamic_obstacle': bool(dynamic_obstacle),
                'depth_motion': bool(depth_motion_active),
                'closing_speed_mps': _json_float(closing_speed),
                'source': sources,
                'best_gap_angle': _json_float(best_gap.angle if best_gap else math.inf),
                'best_gap_clearance': _json_float(
                    best_gap.clearance if best_gap else math.inf
                ),
                'best_gap_width': int(best_gap.width) if best_gap else 0,
            },
        }

        msg = String()
        msg.data = json.dumps(rep, separators=(',', ':'))
        self._pub.publish(msg)

    def _apply_blocked_hysteresis(
            self,
            raw_blocked: bool,
            clear_now: bool,
            now: float,
            raw_sources: list[str]) -> tuple[bool, bool]:
        if raw_blocked:
            self._blocked_latched = True
            self._blocked_until = now + max(0.0, self._obstacle_hold_sec)
            self._clear_seen_since = None
            self._blocked_sources = list(raw_sources)
            return True, False

        if not self._blocked_latched:
            self._clear_seen_since = None
            self._blocked_sources = []
            return False, False

        if not clear_now:
            self._clear_seen_since = None
            return True, True

        if self._clear_seen_since is None:
            self._clear_seen_since = now

        clear_confirmed = now - self._clear_seen_since >= self._clear_confirm_sec
        hold_expired = now >= self._blocked_until
        if clear_confirmed and hold_expired:
            self._blocked_latched = False
            self._clear_seen_since = None
            self._blocked_sources = []
            return False, False

        return True, True

    def _age(self, stamp: float | None) -> float | None:
        if stamp is None:
            return None
        return max(0.0, time.monotonic() - stamp)

    def _min_age(self, *stamps: float | None) -> float | None:
        ages = [self._age(stamp) for stamp in stamps if stamp is not None]
        return min(ages) if ages else None

    def _tof_min_recent(self, topics: list[str] | None = None) -> float:
        if not self._use_tof:
            return math.inf
        selected = set(topics) if topics is not None else None
        recent_ranges = [
            distance
            for topic, distance in self._tof_ranges.items()
            if (selected is None or topic in selected)
            if self._sensor_recent(self._last_tof_times.get(topic))
            and math.isfinite(distance)
        ]
        return min(recent_ranges) if recent_ranges else math.inf

    @staticmethod
    def _gap_json(gap: GapTarget | None) -> dict | None:
        if gap is None:
            return None
        return {
            'angle': gap.angle,
            'clearance': gap.clearance,
            'width': gap.width,
        }

    def _dynamic_obstacle(self, front_distance: float, now: float) -> tuple[bool, float]:
        closing_speed = 0.0
        dynamic = False
        if math.isfinite(front_distance):
            if self._filtered_front_distance is None:
                self._filtered_front_distance = front_distance
            else:
                # If front_distance jumps by more than 30 cm in one tick it
                # almost certainly means the robot just rotated to a new heading
                # (or a large sensor artefact), not a genuine closing obstacle.
                # Reset the filter and history so the next tick starts fresh.
                if abs(front_distance - self._filtered_front_distance) > 0.30:
                    self._filtered_front_distance = front_distance
                    self._prev_front_sample = None
                    self._dynamic_first_seen_time = None
                    return False, 0.0
                alpha = max(0.0, min(1.0, self._front_filter_alpha))
                self._filtered_front_distance = (
                    alpha * front_distance
                    + (1.0 - alpha) * self._filtered_front_distance
                )

            if self._prev_front_sample is not None:
                prev_time, prev_dist = self._prev_front_sample
                dt = now - prev_time
                if dt > 0.01 and math.isfinite(prev_dist):
                    closing_speed = (prev_dist - self._filtered_front_distance) / dt
                    raw_dynamic = (
                        self._filtered_front_distance < self._dynamic_obstacle_distance
                        and closing_speed >= self._dynamic_closing_speed
                    )
                    if raw_dynamic:
                        if self._dynamic_first_seen_time is None:
                            self._dynamic_first_seen_time = now
                        dynamic = (
                            now - self._dynamic_first_seen_time
                            >= self._dynamic_confirm_sec
                        )
                    else:
                        self._dynamic_first_seen_time = None
            self._prev_front_sample = (now, self._filtered_front_distance)
        else:
            self._prev_front_sample = None
            self._filtered_front_distance = None
            self._dynamic_first_seen_time = None
        return dynamic, closing_speed

    def _process_scan(self) -> tuple:
        """Single-pass scan processing: computes all sector stats and gap target."""
        scan = self._latest_scan
        if scan is None:
            return (
                math.inf,
                math.inf,
                math.inf,
                0,
                0,
                0,
                math.inf,
                math.inf,
                math.inf,
                None,
            )

        front_vals: list[float] = []
        front_path_vals: list[float] = []
        left_clearance_vals: list[float] = []
        right_clearance_vals: list[float] = []
        rear_vals: list[float] = []
        gap_points: list[tuple[float, float, bool]] = []

        rear_lo = math.pi - self._rear_angle
        rear_hi = -math.pi + self._rear_angle

        angle = scan.angle_min
        for value in scan.ranges:
            rel_angle = self._relative_angle(angle, self._front_center_angle)
            distance = self._normalized_scan_distance(scan, value)
            valid = distance is not None
            if valid:
                forward = distance * math.cos(rel_angle)
                lateral = distance * math.sin(rel_angle)
                if -self._front_angle <= rel_angle <= self._front_angle:
                    front_vals.append(distance)
                if rear_lo <= rel_angle <= math.pi or -math.pi <= rel_angle <= rear_hi:
                    rear_vals.append(distance)
                if (
                        forward >= 0.0
                        and abs(lateral) <= self._front_path_half_width_m):
                    front_path_vals.append(forward)
                if (
                        -self._side_guard_rear_m
                        <= forward
                        <= self._side_guard_forward_m):
                    if lateral > self._robot_half_width_m:
                        left_clearance_vals.append(
                            max(0.0, lateral - self._robot_half_width_m)
                        )
                    elif lateral < -self._robot_half_width_m:
                        right_clearance_vals.append(
                            max(0.0, abs(lateral) - self._robot_half_width_m)
                        )
            if -self._gap_angle_limit <= rel_angle <= self._gap_angle_limit:
                gap_valid = valid and distance >= self._obstacle_distance
                gap_points.append((rel_angle, distance if valid else math.inf, gap_valid))
            angle += scan.angle_increment

        front_control_candidates = []
        if front_vals:
            front_control_candidates.append(
                float(np.percentile(front_vals, self._front_percentile))
            )
        if front_path_vals:
            front_control_candidates.append(
                float(np.percentile(front_path_vals, self._front_percentile))
            )
        front_close_samples = front_vals + front_path_vals

        if not front_vals and not front_path_vals:
            lidar_front = math.inf
            lidar_front_control = math.inf
            lidar_front_mean = math.inf
            front_close_count = 0
            front_emergency_count = 0
        else:
            lidar_front = min(front_close_samples)
            lidar_front_control = min(front_control_candidates)
            lidar_front_mean = float(sum(front_close_samples) / len(front_close_samples))
            front_close_count = sum(
                1 for item in front_close_samples if item < self._obstacle_distance
            )
            front_emergency_count = sum(
                1 for item in front_close_samples if item < self._emergency_distance
            )
        lidar_left = (
            float(np.percentile(left_clearance_vals, self._side_percentile))
            if left_clearance_vals
            else math.inf
        )
        lidar_right = (
            float(np.percentile(right_clearance_vals, self._side_percentile))
            if right_clearance_vals
            else math.inf
        )
        lidar_rear = (
            float(np.percentile(rear_vals, self._side_percentile))
            if rear_vals
            else math.inf
        )
        best_gap = self._find_best_gap(gap_points)

        return (
            lidar_front,
            lidar_front_control,
            lidar_front_mean,
            len(front_vals),
            front_close_count,
            front_emergency_count,
            lidar_left,
            lidar_right,
            lidar_rear,
            best_gap,
        )

    @staticmethod
    def _normalized_scan_distance(scan: LaserScan, value: float) -> float | None:
        if math.isfinite(value):
            if scan.range_min <= value <= scan.range_max:
                return float(value)
            return None
        if math.isinf(value) and value > 0.0:
            if math.isfinite(scan.range_max) and scan.range_max > 0.0:
                return float(scan.range_max)
            return 10.0
        return None

    @staticmethod
    def _relative_angle(angle: float, center: float) -> float:
        delta = angle - center
        return math.atan2(math.sin(delta), math.cos(delta))

    def _find_best_gap(self, points: list[tuple[float, float, bool]]) -> GapTarget | None:
        best: GapTarget | None = None
        start = 0
        while start < len(points):
            while start < len(points) and not points[start][2]:
                start += 1
            end = start
            while end < len(points) and points[end][2]:
                end += 1
            if end > start:
                segment = points[start:end]
                center = len(segment) // 2
                candidate = GapTarget(
                    segment[center][0],
                    float(sum(item[1] for item in segment) / len(segment)),
                    len(segment),
                )
                if best is None or (candidate.width, candidate.clearance) > (
                        best.width, best.clearance):
                    best = candidate
            start = end + 1
        return best


def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except ValueError:
            pass
        if rclpy.ok():
            rclpy.shutdown()
