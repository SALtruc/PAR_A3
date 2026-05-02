"""
Project C reactive decision node.

Consumes the fused obstacle representation from obstacle_perception_node and
publishes /cmd_vel. The policy remains purely reactive: every control cycle is
based on the current fused sensor snapshot, with only short timed recovery
states for backing out of dead ends.
"""

import json
import math
import random
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from std_msgs.msg import String


DRIVE = 'DRIVE'
AVOID = 'AVOID'
DYNAMIC_AVOID = 'DYNAMIC_AVOID'
BACKUP = 'BACKUP'
TURN_OUT = 'TURN_OUT'
EMERGENCY = 'EMERGENCY'
NO_OBSTACLES = 'NO_OBSTACLES'


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _finite_or_inf(value) -> float:
    if value is None:
        return math.inf
    try:
        value_f = float(value)
    except (TypeError, ValueError):
        return math.inf
    return value_f if math.isfinite(value_f) else math.inf


@dataclass
class GapTarget:
    angle: float
    clearance: float
    width: int


class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.declare_parameter('obstacle_topic', '/obstacle_representation')
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

        self.declare_parameter('obstacle_distance', 0.55)
        self.declare_parameter('clear_distance', 0.75)
        self.declare_parameter('dynamic_stop_distance', 0.90)
        self.declare_parameter('obstacle_stale_sec', 1.0)
        self.declare_parameter('backup_sec', 0.9)
        self.declare_parameter('turn_out_sec', 1.8)
        self.declare_parameter('wander_enabled', True)
        self.declare_parameter('wander_interval_sec', 4.0)
        self.declare_parameter('wander_angular_speed', 0.18)

        obstacle_topic = self.get_parameter('obstacle_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        state_topic = self.get_parameter('state_topic').value
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

        self._obstacle_distance = float(self.get_parameter('obstacle_distance').value)
        self._clear_distance = float(self.get_parameter('clear_distance').value)
        self._dynamic_stop_distance = float(
            self.get_parameter('dynamic_stop_distance').value
        )
        self._obstacle_stale_sec = float(self.get_parameter('obstacle_stale_sec').value)
        self._backup_sec = float(self.get_parameter('backup_sec').value)
        self._turn_out_sec = float(self.get_parameter('turn_out_sec').value)
        self._wander_enabled = _as_bool(self.get_parameter('wander_enabled').value)
        self._wander_interval_sec = float(
            self.get_parameter('wander_interval_sec').value
        )
        self._wander_angular_speed = float(
            self.get_parameter('wander_angular_speed').value
        )

        self._latest_obstacles: dict | None = None
        self._last_obstacle_time: float | None = None
        self._state = NO_OBSTACLES
        self._state_end_time = 0.0
        self._turn_direction = 1.0
        self._wander_next_time = 0.0
        self._wander_bias = 0.0

        self.create_subscription(String, obstacle_topic, self._on_obstacles, 10)
        vel_type = TwistStamped if self._cmd_vel_stamped else Twist
        self._vel_pub = self.create_publisher(vel_type, cmd_vel_topic, 10)
        self._state_pub = self.create_publisher(String, state_topic, 10)
        self.create_timer(1.0 / max(control_hz, 1.0), self._control_loop)

        self.get_logger().info(
            f'Obstacle decision ready. obstacles={obstacle_topic}, '
            f'cmd_vel={cmd_vel_topic} ({vel_type.__name__})'
        )

    def _on_obstacles(self, msg: String):
        try:
            self._latest_obstacles = json.loads(msg.data)
            self._last_obstacle_time = time.monotonic()
        except json.JSONDecodeError:
            self.get_logger().warn('Ignored invalid obstacle representation JSON.')

    def _obstacles_recent(self) -> bool:
        return (
            self._last_obstacle_time is not None
            and time.monotonic() - self._last_obstacle_time <= self._obstacle_stale_sec
        )

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

        if self._latest_obstacles is None or not self._obstacles_recent():
            self._transition(NO_OBSTACLES)
            self._publish_velocity(twist)
            return

        fused = self._latest_obstacles.get('fused', {})
        front = _finite_or_inf(fused.get('front_distance'))
        left = _finite_or_inf(fused.get('left_distance'))
        right = _finite_or_inf(fused.get('right_distance'))
        emergency = bool(fused.get('emergency', False))
        blocked = bool(fused.get('blocked', False))
        dead_end = bool(fused.get('dead_end', False))
        dynamic_obstacle = bool(fused.get('dynamic_obstacle', False))
        target = self._target_from_fused(fused)

        if emergency:
            self._transition(EMERGENCY)
            self._publish_velocity(twist)
            return

        if self._state == EMERGENCY:
            self._transition(AVOID if blocked else DRIVE)

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

        if dead_end:
            self._turn_direction = self._clearer_side(left, right)
            self._transition(BACKUP, self._backup_sec)
            self._publish_velocity(twist)
            return

        if dynamic_obstacle and front < self._dynamic_stop_distance:
            self._transition(DYNAMIC_AVOID)
            self._drive_dynamic_obstacle(twist, target)
        elif blocked or front < self._obstacle_distance:
            self._transition(AVOID)
            self._drive_toward_gap(twist, target, front)
        else:
            self._transition(DRIVE)
            self._drive_clear_path(twist, target, left, right, front)
            self._apply_wander(twist, left, right, front, now)

        self._publish_velocity(twist)

    @staticmethod
    def _target_from_fused(fused: dict) -> GapTarget | None:
        width = int(fused.get('best_gap_width', 0) or 0)
        angle = _finite_or_inf(fused.get('best_gap_angle'))
        clearance = _finite_or_inf(fused.get('best_gap_clearance'))
        if width <= 0 or not math.isfinite(angle):
            return None
        return GapTarget(angle=angle, clearance=clearance, width=width)

    def _drive_dynamic_obstacle(self, twist: Twist, target: GapTarget | None):
        if target is None:
            return
        twist.angular.z = _clamp(
            self._gap_kp * target.angle,
            -self._max_angular_speed,
            self._max_angular_speed,
        )
        if abs(target.angle) < 0.20:
            twist.linear.x = self._min_speed * 0.5

    def _drive_toward_gap(
            self,
            twist: Twist,
            target: GapTarget | None,
            front: float):
        if target is None:
            self._transition(BACKUP, self._backup_sec)
            return

        angular = _clamp(
            self._gap_kp * target.angle,
            -self._max_angular_speed,
            self._max_angular_speed,
        )
        turn_scale = max(0.25, 1.0 - abs(target.angle) / math.radians(110.0))
        distance_scale = _clamp(front / self._obstacle_distance, 0.2, 1.0)
        twist.linear.x = self._min_speed * turn_scale * distance_scale
        twist.angular.z = angular

    def _drive_clear_path(
            self,
            twist: Twist,
            target: GapTarget | None,
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

    def _apply_wander(
            self,
            twist: Twist,
            left: float,
            right: float,
            front: float,
            now: float):
        if not self._wander_enabled:
            return

        open_space = (
            front > self._clear_distance
            and (not math.isfinite(left) or left > self._clear_distance)
            and (not math.isfinite(right) or right > self._clear_distance)
        )
        if not open_space:
            return

        if now >= self._wander_next_time:
            self._wander_bias = random.uniform(
                -self._wander_angular_speed,
                self._wander_angular_speed,
            )
            self._wander_next_time = now + self._wander_interval_sec

        twist.angular.z = _clamp(
            twist.angular.z + self._wander_bias,
            -self._max_angular_speed,
            self._max_angular_speed,
        )

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


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
