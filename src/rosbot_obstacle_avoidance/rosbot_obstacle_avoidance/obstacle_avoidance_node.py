"""
Project C obstacle avoidance.

Reactive controller for the ROSbot 3 PRO using LaserScan sectors:
  - static/dynamic obstacles: follow the clearest local gap
  - narrow passages: centre between left/right walls
  - dead ends: reverse, turn toward the clearer side, and retry
  - emergency stop: front LIDAR or ToF below emergency threshold
"""

import math
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import String


DRIVE = 'DRIVE'
AVOID = 'AVOID'
BACKUP = 'BACKUP'
TURN_OUT = 'TURN_OUT'
EMERGENCY = 'EMERGENCY'
NO_SCAN = 'NO_SCAN'


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class GapTarget:
    angle: float
    clearance: float
    width: int


class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('tof_topic', '/range')
        self.declare_parameter('use_tof', True)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_stamped', True)
        self.declare_parameter('cmd_vel_frame_id', 'base_link')
        self.declare_parameter('state_topic', '/obstacle_avoidance_state')
        self.declare_parameter('control_hz', 20.0)

        self.declare_parameter('max_speed', 0.18)
        self.declare_parameter('min_speed', 0.05)
        self.declare_parameter('reverse_speed', 0.08)
        self.declare_parameter('max_angular_speed', 0.65)
        self.declare_parameter('gap_kp', 1.25)
        self.declare_parameter('corridor_kp', 0.45)

        self.declare_parameter('emergency_distance', 0.18)
        self.declare_parameter('obstacle_distance', 0.55)
        self.declare_parameter('clear_distance', 0.75)
        self.declare_parameter('front_angle_deg', 30.0)
        self.declare_parameter('side_angle_deg', 70.0)
        self.declare_parameter('gap_angle_limit_deg', 110.0)
        self.declare_parameter('sensor_stale_sec', 1.0)

        self.declare_parameter('backup_sec', 0.9)
        self.declare_parameter('turn_out_sec', 1.8)

        scan_topic = self.get_parameter('scan_topic').value
        tof_topic = self.get_parameter('tof_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        state_topic = self.get_parameter('state_topic').value
        self._use_tof = _as_bool(self.get_parameter('use_tof').value)
        self._cmd_vel_stamped = _as_bool(self.get_parameter('cmd_vel_stamped').value)
        self._cmd_vel_frame_id = self.get_parameter('cmd_vel_frame_id').value
        control_hz = float(self.get_parameter('control_hz').value)

        self._max_speed = float(self.get_parameter('max_speed').value)
        self._min_speed = float(self.get_parameter('min_speed').value)
        self._reverse_speed = float(self.get_parameter('reverse_speed').value)
        self._max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )
        self._gap_kp = float(self.get_parameter('gap_kp').value)
        self._corridor_kp = float(self.get_parameter('corridor_kp').value)

        self._emergency_distance = float(
            self.get_parameter('emergency_distance').value
        )
        self._obstacle_distance = float(self.get_parameter('obstacle_distance').value)
        self._clear_distance = float(self.get_parameter('clear_distance').value)
        self._front_angle = math.radians(self.get_parameter('front_angle_deg').value)
        self._side_angle = math.radians(self.get_parameter('side_angle_deg').value)
        self._gap_angle_limit = math.radians(
            self.get_parameter('gap_angle_limit_deg').value
        )
        self._sensor_stale_sec = float(self.get_parameter('sensor_stale_sec').value)
        self._backup_sec = float(self.get_parameter('backup_sec').value)
        self._turn_out_sec = float(self.get_parameter('turn_out_sec').value)

        self._latest_scan: LaserScan | None = None
        self._last_scan_time: float | None = None
        self._tof_range = math.inf
        self._last_tof_time: float | None = None
        self._state = NO_SCAN
        self._state_end_time = 0.0
        self._turn_direction = 1.0

        self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        if self._use_tof:
            self.create_subscription(Range, tof_topic, self._on_tof, 10)

        vel_type = TwistStamped if self._cmd_vel_stamped else Twist
        self._vel_pub = self.create_publisher(vel_type, cmd_vel_topic, 10)
        self._state_pub = self.create_publisher(String, state_topic, 10)
        self.create_timer(1.0 / max(control_hz, 1.0), self._control_loop)

        self.get_logger().info(
            f'Obstacle avoidance ready. scan={scan_topic}, cmd_vel={cmd_vel_topic} '
            f'({vel_type.__name__}), tof={tof_topic if self._use_tof else "disabled"}'
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

    def _sensor_recent(self, stamp: float | None) -> bool:
        return stamp is not None and time.monotonic() - stamp <= self._sensor_stale_sec

    def _transition(self, new_state: str, duration: float = 0.0):
        if self._state != new_state:
            self.get_logger().info(f'Obstacle FSM: {self._state} -> {new_state}')
            msg = String()
            msg.data = f'{time.time():.3f},{new_state}'
            self._state_pub.publish(msg)
        self._state = new_state
        self._state_end_time = time.monotonic() + duration

    def _control_loop(self):
        twist = Twist()
        now = time.monotonic()

        if self._latest_scan is None or not self._sensor_recent(self._last_scan_time):
            self._transition(NO_SCAN)
            self._publish_velocity(twist)
            return

        front = self._sector_min(-self._front_angle, self._front_angle)
        left = self._sector_mean(math.radians(35.0), self._side_angle)
        right = self._sector_mean(-self._side_angle, math.radians(-35.0))
        tof_recent = self._sensor_recent(self._last_tof_time)
        tof_blocked = self._use_tof and tof_recent and self._tof_range < self._emergency_distance

        if tof_blocked or front < self._emergency_distance:
            self._transition(EMERGENCY)
            self._publish_velocity(twist)
            return

        if self._state == EMERGENCY:
            self._transition(AVOID if front < self._clear_distance else DRIVE)

        if self._state == BACKUP:
            if now >= self._state_end_time:
                self._turn_direction = self._clearer_side(left, right)
                self._transition(TURN_OUT, self._turn_out_sec)
            else:
                twist.linear.x = -self._reverse_speed
                self._publish_velocity(twist)
                return

        if self._state == TURN_OUT:
            if now >= self._state_end_time:
                self._transition(AVOID)
            else:
                twist.angular.z = self._turn_direction * self._max_angular_speed
                self._publish_velocity(twist)
                return

        target = self._best_gap_target()
        dead_end = target is None or (
            front < self._obstacle_distance
            and left < self._obstacle_distance
            and right < self._obstacle_distance
        )
        if dead_end:
            self._turn_direction = self._clearer_side(left, right)
            self._transition(BACKUP, self._backup_sec)
            self._publish_velocity(twist)
            return

        if front < self._obstacle_distance:
            self._transition(AVOID)
            self._drive_toward_gap(twist, target, front)
        else:
            self._transition(DRIVE)
            self._drive_clear_path(twist, target, left, right, front)

        self._publish_velocity(twist)

    def _drive_toward_gap(self, twist: Twist, target: GapTarget, front: float):
        angular = _clamp(
            self._gap_kp * target.angle,
            -self._max_angular_speed,
            self._max_angular_speed,
        )
        turn_scale = max(0.25, 1.0 - abs(target.angle) / self._gap_angle_limit)
        distance_scale = _clamp(front / self._obstacle_distance, 0.2, 1.0)
        twist.linear.x = self._min_speed * turn_scale * distance_scale
        twist.angular.z = angular

    def _drive_clear_path(
            self,
            twist: Twist,
            target: GapTarget,
            left: float,
            right: float,
            front: float):
        twist.linear.x = self._max_speed
        if math.isfinite(left) and math.isfinite(right):
            corridor_error = left - right
            twist.angular.z = _clamp(
                self._corridor_kp * corridor_error,
                -self._max_angular_speed,
                self._max_angular_speed,
            )
        elif target is not None:
            twist.angular.z = _clamp(
                0.45 * self._gap_kp * target.angle,
                -self._max_angular_speed,
                self._max_angular_speed,
            )

        if front < self._clear_distance:
            twist.linear.x = max(self._min_speed, self._max_speed * 0.55)

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
                angle_center = segment[center][0]
                clearance = float(np_mean([item[1] for item in segment]))
                width = len(segment)
                candidate = GapTarget(angle_center, clearance, width)
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
        return float(np_mean(values)) if values else math.inf

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

    @staticmethod
    def _clearer_side(left: float, right: float) -> float:
        if not math.isfinite(left) and not math.isfinite(right):
            return 1.0
        if not math.isfinite(left):
            return -1.0
        if not math.isfinite(right):
            return 1.0
        return 1.0 if left >= right else -1.0

    def _publish_velocity(self, twist: Twist):
        if not self._cmd_vel_stamped:
            self._vel_pub.publish(twist)
            return

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self._cmd_vel_frame_id)
        msg.twist = twist
        self._vel_pub.publish(msg)


def np_mean(values: list[float]) -> float:
    return sum(values) / max(len(values), 1)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
