"""
Project C fused obstacle perception.

This node converts the current S2 LIDAR scan, OAK-D depth image, and VL53L0X
ToF range into one JSON obstacle representation. Keeping perception separate
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
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Range
from std_msgs.msg import String


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


@dataclass
class GapTarget:
    angle: float
    clearance: float
    width: int


class ObstaclePerceptionNode(Node):

    def __init__(self):
        super().__init__('obstacle_perception')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('tof_topic', '/range')
        self.declare_parameter('obstacle_topic', '/obstacle_representation')
        self.declare_parameter('use_lidar', True)
        self.declare_parameter('use_depth', True)
        self.declare_parameter('use_tof', True)
        self.declare_parameter('publish_hz', 20.0)

        self.declare_parameter('emergency_distance', 0.18)
        self.declare_parameter('obstacle_distance', 0.55)
        self.declare_parameter('clear_distance', 0.75)
        self.declare_parameter('front_angle_deg', 30.0)
        self.declare_parameter('side_angle_deg', 70.0)
        self.declare_parameter('rear_angle_deg', 35.0)
        self.declare_parameter('gap_angle_limit_deg', 110.0)
        self.declare_parameter('sensor_stale_sec', 1.0)

        self.declare_parameter('depth_obstacle_distance', 0.65)
        self.declare_parameter('depth_center_fraction', 0.33)
        self.declare_parameter('depth_side_fraction', 0.30)
        self.declare_parameter('dynamic_obstacle_distance', 1.0)
        self.declare_parameter('dynamic_closing_speed', 0.25)

        scan_topic = self.get_parameter('scan_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        tof_topic = self.get_parameter('tof_topic').value
        obstacle_topic = self.get_parameter('obstacle_topic').value

        self._use_lidar = _as_bool(self.get_parameter('use_lidar').value)
        self._use_depth = _as_bool(self.get_parameter('use_depth').value)
        self._use_tof = _as_bool(self.get_parameter('use_tof').value)
        publish_hz = float(self.get_parameter('publish_hz').value)

        self._emergency_distance = float(
            self.get_parameter('emergency_distance').value
        )
        self._obstacle_distance = float(self.get_parameter('obstacle_distance').value)
        self._clear_distance = float(self.get_parameter('clear_distance').value)
        self._front_angle = math.radians(
            float(self.get_parameter('front_angle_deg').value)
        )
        self._side_angle = math.radians(
            float(self.get_parameter('side_angle_deg').value)
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
        self._depth_center_fraction = float(
            self.get_parameter('depth_center_fraction').value
        )
        self._depth_side_fraction = float(
            self.get_parameter('depth_side_fraction').value
        )
        self._dynamic_obstacle_distance = float(
            self.get_parameter('dynamic_obstacle_distance').value
        )
        self._dynamic_closing_speed = float(
            self.get_parameter('dynamic_closing_speed').value
        )

        self._latest_scan: LaserScan | None = None
        self._last_scan_time: float | None = None
        self._depth_front = math.inf
        self._depth_left = math.inf
        self._depth_right = math.inf
        self._last_depth_time: float | None = None
        self._tof_range = math.inf
        self._last_tof_time: float | None = None
        self._prev_front_sample: tuple[float, float] | None = None
        self._bridge = CvBridge()

        if self._use_lidar:
            self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        if self._use_depth:
            self.create_subscription(Image, depth_topic, self._on_depth, 10)
        if self._use_tof:
            self.create_subscription(Range, tof_topic, self._on_tof, 10)

        self._pub = self.create_publisher(String, obstacle_topic, 10)
        self.create_timer(1.0 / max(publish_hz, 1.0), self._publish_representation)

        self.get_logger().info(
            'Obstacle perception ready. '
            f'lidar={scan_topic if self._use_lidar else "disabled"}, '
            f'depth={depth_topic if self._use_depth else "disabled"}, '
            f'tof={tof_topic if self._use_tof else "disabled"}, '
            f'out={obstacle_topic}'
        )

    def _on_scan(self, msg: LaserScan):
        self._latest_scan = msg
        self._last_scan_time = time.monotonic()

    def _on_tof(self, msg: Range):
        self._last_tof_time = time.monotonic()
        if math.isfinite(msg.range) and msg.min_range < msg.range < msg.max_range:
            self._tof_range = msg.range
        else:
            self._tof_range = math.inf

    def _on_depth(self, msg: Image):
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

    def _depth_roi_distance(
            self,
            depth_img,
            encoding: str,
            x_center: float,
            width_fraction: float) -> float:
        h, w = depth_img.shape[:2]
        roi_w = max(1, int(w * width_fraction))
        roi_h = max(1, int(h * self._depth_center_fraction))
        cx = int(w * x_center)
        cy = h // 2
        x0 = max(0, cx - roi_w // 2)
        x1 = min(w, cx + roi_w // 2)
        y0 = max(0, cy - roi_h // 2)
        y1 = min(h, cy + roi_h // 2)
        roi = depth_img[y0:y1, x0:x1]

        valid = roi[np.isfinite(roi) & (roi > 0)]
        if valid.size == 0:
            return math.inf

        depth_value = float(np.percentile(valid, 10))
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
        depth_recent = self._use_depth and self._sensor_recent(self._last_depth_time)
        tof_recent = self._use_tof and self._sensor_recent(self._last_tof_time)

        lidar_front = math.inf
        lidar_front_mean = math.inf
        lidar_left = math.inf
        lidar_right = math.inf
        lidar_rear = math.inf
        best_gap: GapTarget | None = None
        if scan_recent:
            lidar_front = self._sector_min(-self._front_angle, self._front_angle)
            lidar_front_mean = self._sector_mean(-self._front_angle, self._front_angle)
            lidar_left = self._sector_mean(math.radians(35.0), self._side_angle)
            lidar_right = self._sector_mean(-self._side_angle, math.radians(-35.0))
            lidar_rear = self._sector_mean(
                math.pi - self._rear_angle,
                math.pi,
            )
            best_gap = self._best_gap_target()

        depth_front = self._depth_front if depth_recent else math.inf
        depth_left = self._depth_left if depth_recent else math.inf
        depth_right = self._depth_right if depth_recent else math.inf
        tof_range = self._tof_range if tof_recent else math.inf

        front_distance = _min_finite(lidar_front, depth_front)
        left_distance = _min_finite(lidar_left, depth_left)
        right_distance = _min_finite(lidar_right, depth_right)

        blocked_lidar = lidar_front < self._obstacle_distance
        blocked_depth = depth_front < self._depth_obstacle_distance
        blocked = blocked_lidar or blocked_depth
        emergency = (
            front_distance < self._emergency_distance
            or tof_range < self._emergency_distance
        )
        dead_end = blocked and (
            best_gap is None
            or (
                left_distance < self._obstacle_distance
                and right_distance < self._obstacle_distance
            )
        )
        dynamic_obstacle, closing_speed = self._dynamic_obstacle(front_distance, now)

        sources = []
        if blocked_lidar:
            sources.append('lidar')
        if blocked_depth:
            sources.append('depth')
        if tof_range < self._emergency_distance:
            sources.append('tof')

        rep = {
            'stamp': wall_stamp,
            'ages': {
                'scan': self._age(self._last_scan_time),
                'depth': self._age(self._last_depth_time),
                'tof': self._age(self._last_tof_time),
            },
            'lidar': {
                'available': bool(scan_recent),
                'front_min': _json_float(lidar_front),
                'front_mean': _json_float(lidar_front_mean),
                'left_mean': _json_float(lidar_left),
                'right_mean': _json_float(lidar_right),
                'rear_mean': _json_float(lidar_rear),
                'best_gap': self._gap_json(best_gap),
            },
            'depth': {
                'available': bool(depth_recent),
                'front_min': _json_float(depth_front),
                'left_min': _json_float(depth_left),
                'right_min': _json_float(depth_right),
            },
            'tof': {
                'available': bool(tof_recent),
                'range': _json_float(tof_range),
            },
            'fused': {
                'front_distance': _json_float(front_distance),
                'left_distance': _json_float(left_distance),
                'right_distance': _json_float(right_distance),
                'blocked': bool(blocked),
                'clear': bool(front_distance >= self._clear_distance),
                'emergency': bool(emergency),
                'dead_end': bool(dead_end),
                'dynamic_obstacle': bool(dynamic_obstacle),
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

    def _age(self, stamp: float | None) -> float | None:
        if stamp is None:
            return None
        return max(0.0, time.monotonic() - stamp)

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
            if self._prev_front_sample is not None:
                prev_time, prev_dist = self._prev_front_sample
                dt = now - prev_time
                if dt > 0.01 and math.isfinite(prev_dist):
                    closing_speed = (prev_dist - front_distance) / dt
                    dynamic = (
                        front_distance < self._dynamic_obstacle_distance
                        and closing_speed >= self._dynamic_closing_speed
                    )
            self._prev_front_sample = (now, front_distance)
        else:
            self._prev_front_sample = None
        return dynamic, closing_speed

    def _best_gap_target(self) -> GapTarget | None:
        scan = self._latest_scan
        if scan is None:
            return None

        points = []
        angle = scan.angle_min
        for value in scan.ranges:
            if -self._gap_angle_limit <= angle <= self._gap_angle_limit:
                valid = (
                    math.isfinite(value)
                    and scan.range_min <= value <= scan.range_max
                    and value >= self._obstacle_distance
                )
                points.append((angle, value, valid))
            angle += scan.angle_increment

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

    def _sector_min(self, angle_lo: float, angle_hi: float) -> float:
        values = self._sector_values(angle_lo, angle_hi)
        return min(values) if values else math.inf

    def _sector_mean(self, angle_lo: float, angle_hi: float) -> float:
        values = self._sector_values(angle_lo, angle_hi)
        return float(sum(values) / len(values)) if values else math.inf

    def _sector_values(self, angle_lo: float, angle_hi: float) -> list[float]:
        scan = self._latest_scan
        if scan is None:
            return []
        values = []
        angle = scan.angle_min
        for value in scan.ranges:
            if angle_lo <= angle <= angle_hi:
                if math.isfinite(value) and scan.range_min <= value <= scan.range_max:
                    values.append(value)
            angle += scan.angle_increment
        return values


def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
