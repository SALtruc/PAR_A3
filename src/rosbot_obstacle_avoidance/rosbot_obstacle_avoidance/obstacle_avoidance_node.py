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


def _fmt_m(value: float) -> str:
    return f'{value:.2f}m' if math.isfinite(value) else 'inf'


def _fmt_opt_m(value) -> str:
    return _fmt_m(_finite_or_inf(value))


def _fmt_age(value) -> str:
    if value is None:
        return 'none'
    return f'{float(value):.2f}s'


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

        self.declare_parameter('max_speed', 0.10)
        self.declare_parameter('min_speed', 0.05)
        self.declare_parameter('reverse_speed', 0.08)
        self.declare_parameter('max_angular_speed', 0.40)
        self.declare_parameter('drive_max_angular_speed', 0.18)
        self.declare_parameter('gap_kp', 0.85)
        self.declare_parameter('corridor_kp', 0.18)

        self.declare_parameter('obstacle_distance', 0.55)
        self.declare_parameter('clear_distance', 0.80)
        self.declare_parameter('dynamic_stop_distance', 0.90)
        self.declare_parameter('avoid_turn_only_distance', 0.45)
        self.declare_parameter('avoid_forward_distance', 0.80)
        self.declare_parameter('backup_clear_distance', 0.35)
        self.declare_parameter('dynamic_hold_sec', 0.80)
        self.declare_parameter('obstacle_stale_sec', 1.0)
        self.declare_parameter('backup_sec', 0.45)
        self.declare_parameter('turn_out_sec', 1.2)
        self.declare_parameter('narrow_passage_distance', 0.65)
        self.declare_parameter('wander_enabled', True)
        self.declare_parameter('wander_interval_sec', 4.0)
        self.declare_parameter('wander_angular_speed', 0.15)
        self.declare_parameter('side_balance_distance', 0.95)
        self.declare_parameter('debug_decisions', True)
        self.declare_parameter('debug_period_sec', 1.0)

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
        self._drive_max_angular_speed = float(
            self.get_parameter('drive_max_angular_speed').value
        )
        self._gap_kp = float(self.get_parameter('gap_kp').value)
        self._corridor_kp = float(self.get_parameter('corridor_kp').value)

        self._obstacle_distance = float(self.get_parameter('obstacle_distance').value)
        self._clear_distance = float(self.get_parameter('clear_distance').value)
        self._dynamic_stop_distance = float(
            self.get_parameter('dynamic_stop_distance').value
        )
        self._avoid_turn_only_distance = float(
            self.get_parameter('avoid_turn_only_distance').value
        )
        self._avoid_forward_distance = float(
            self.get_parameter('avoid_forward_distance').value
        )
        self._backup_clear_distance = float(
            self.get_parameter('backup_clear_distance').value
        )
        self._dynamic_hold_sec = float(self.get_parameter('dynamic_hold_sec').value)
        self._obstacle_stale_sec = float(self.get_parameter('obstacle_stale_sec').value)
        self._backup_sec = float(self.get_parameter('backup_sec').value)
        self._turn_out_sec = float(self.get_parameter('turn_out_sec').value)
        self._narrow_passage_distance = float(
            self.get_parameter('narrow_passage_distance').value
        )
        self._wander_enabled = _as_bool(self.get_parameter('wander_enabled').value)
        self._wander_interval_sec = float(
            self.get_parameter('wander_interval_sec').value
        )
        self._wander_angular_speed = float(
            self.get_parameter('wander_angular_speed').value
        )
        self._side_balance_distance = float(
            self.get_parameter('side_balance_distance').value
        )
        self._debug_decisions = _as_bool(self.get_parameter('debug_decisions').value)
        self._debug_period_sec = float(self.get_parameter('debug_period_sec').value)

        self._latest_obstacles: dict | None = None
        self._last_obstacle_time: float | None = None
        self._state = NO_OBSTACLES
        self._state_end_time = 0.0
        self._turn_direction = 1.0
        self._dynamic_hold_until = 0.0
        self._wander_next_time = 0.0
        self._wander_bias = 0.0
        self._last_debug_time = 0.0
        self._debug_transition: str | None = None

        self.create_subscription(String, obstacle_topic, self._on_obstacles, 10)
        vel_type = TwistStamped if self._cmd_vel_stamped else Twist
        self._vel_pub = self.create_publisher(vel_type, cmd_vel_topic, 10)
        self._state_pub = self.create_publisher(String, state_topic, 10)
        self.create_timer(1.0 / max(control_hz, 1.0), self._control_loop)

        self.get_logger().info(
            f'Obstacle decision ready. obstacles={obstacle_topic}, '
            f'cmd_vel={cmd_vel_topic} ({vel_type.__name__}), '
            f'debug_decisions={self._debug_decisions}'
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
            previous = self._state
            self.get_logger().info(f'Obstacle FSM: {self._state} -> {new_state}')
            msg = String()
            msg.data = f'{time.time():.3f},{new_state}'
            self._state_pub.publish(msg)
            self._debug_transition = f'{previous}->{new_state}'
            self._state = new_state
            self._state_end_time = time.monotonic() + duration
            return

        if duration <= 0.0:
            self._state_end_time = time.monotonic()

    def _control_loop(self):
        twist = Twist()
        now = time.monotonic()

        if self._latest_obstacles is None or not self._obstacles_recent():
            self._transition(NO_OBSTACLES)
            self._log_decision('no_fresh_obstacle_representation', twist)
            self._publish_velocity(twist)
            return

        fused = self._latest_obstacles.get('fused', {})
        front = _finite_or_inf(fused.get('front_distance'))
        left = _finite_or_inf(fused.get('left_distance'))
        right = _finite_or_inf(fused.get('right_distance'))
        rear = _finite_or_inf(fused.get('rear_distance'))
        source = list(fused.get('source', []))
        emergency = bool(fused.get('emergency', False))
        blocked = bool(fused.get('blocked', False))
        dead_end = bool(fused.get('dead_end', False))
        dynamic_obstacle = bool(fused.get('dynamic_obstacle', False))
        target = self._target_from_fused(fused)

        if not self._any_sensor_available(self._latest_obstacles):
            self._transition(NO_OBSTACLES)
            self._log_decision('no_active_sensors', twist, fused, target)
            self._publish_velocity(twist)
            return

        if emergency and 'tof' in source:
            self._transition(EMERGENCY)
            self._log_decision('tof_emergency_stop', twist, fused, target)
            self._publish_velocity(twist)
            return

        if self._state == EMERGENCY:
            self._transition(AVOID if blocked else DRIVE)

        if self._state == DYNAMIC_AVOID and now < self._dynamic_hold_until:
            self._log_decision('dynamic_hold', twist, fused, target)
            self._publish_velocity(twist)
            return

        if self._state == BACKUP:
            if rear <= self._backup_clear_distance:
                self._turn_direction = self._turn_direction_from(target, left, right)
                self._transition(TURN_OUT, self._turn_out_sec)
                self._log_decision('backup_rear_blocked_turn_out', twist, fused, target)
                self._publish_velocity(twist)
                return

            if now >= self._state_end_time:
                self._turn_direction = self._turn_direction_from(target, left, right)
                self._transition(TURN_OUT, self._turn_out_sec)
            else:
                twist.linear.x = -self._reverse_speed
                self._log_decision('backup_reverse', twist, fused, target)
                self._publish_velocity(twist)
                return

        if self._state == TURN_OUT:
            if now >= self._state_end_time:
                self._transition(AVOID)
            else:
                twist.angular.z = self._turn_direction * self._max_angular_speed
                self._log_decision('turn_out', twist, fused, target)
                self._publish_velocity(twist)
                return

        if emergency:
            self._turn_direction = self._turn_direction_from(target, left, right)
            if rear > self._backup_clear_distance:
                self._transition(BACKUP, self._backup_sec)
                twist.linear.x = -self._reverse_speed
                self._log_decision('lidar_emergency_backup', twist, fused, target)
            else:
                self._transition(TURN_OUT, self._turn_out_sec)
                self._log_decision('lidar_emergency_turn_out', twist, fused, target)
            self._publish_velocity(twist)
            return

        if dead_end:
            self._turn_direction = self._turn_direction_from(target, left, right)
            if rear > self._backup_clear_distance:
                self._transition(BACKUP, self._backup_sec)
            else:
                self._transition(TURN_OUT, self._turn_out_sec)
            self._log_decision('dead_end_recovery', twist, fused, target)
            self._publish_velocity(twist)
            return

        if dynamic_obstacle and front < self._dynamic_stop_distance:
            self._dynamic_hold_until = max(
                self._dynamic_hold_until,
                now + self._dynamic_hold_sec,
            )
            self._transition(DYNAMIC_AVOID, self._dynamic_hold_sec)
            self._drive_dynamic_obstacle(twist, target)
            self._log_decision('dynamic_obstacle_confirmed', twist, fused, target)
        elif blocked or front < self._obstacle_distance:
            self._transition(AVOID)
            self._drive_toward_gap(twist, target, front, left, right)
            self._log_decision('blocked_or_too_close', twist, fused, target)
        else:
            self._transition(DRIVE)
            self._drive_clear_path(twist, target, left, right, front)
            self._apply_wander(twist, left, right, front, now)
            self._log_decision('clear_drive', twist, fused, target)

        self._publish_velocity(twist)

    @staticmethod
    def _target_from_fused(fused: dict) -> GapTarget | None:
        width = int(fused.get('best_gap_width', 0) or 0)
        angle = _finite_or_inf(fused.get('best_gap_angle'))
        clearance = _finite_or_inf(fused.get('best_gap_clearance'))
        if width <= 0 or not math.isfinite(angle):
            return None
        return GapTarget(angle=angle, clearance=clearance, width=width)

    @staticmethod
    def _any_sensor_available(data: dict | None) -> bool:
        if not data:
            return False
        return any(
            bool(data.get(name, {}).get('available', False))
            for name in ('lidar', 'depth', 'tof')
        )

    def _drive_dynamic_obstacle(self, twist: Twist, target: GapTarget | None):
        # Dynamic objects are usually people. Stop first; the next cycle can
        # re-enter AVOID/DRIVE after the hold window if the path is clear.
        del target
        twist.linear.x = 0.0
        twist.angular.z = 0.0

    def _drive_toward_gap(
            self,
            twist: Twist,
            target: GapTarget | None,
            front: float,
            left: float,
            right: float):
        if target is None:
            self._transition(BACKUP, self._backup_sec)
            return

        turn_angle = target.angle
        if abs(turn_angle) < 0.12 and front < self._avoid_forward_distance:
            turn_angle = self._clearer_side(left, right) * 0.55

        twist.angular.z = _clamp(
            self._gap_kp * turn_angle,
            -self._max_angular_speed,
            self._max_angular_speed,
        )

        # Never creep into a close obstacle. Rotate until the front sector is
        # comfortably clear, then resume slow forward motion through the gap.
        if front < self._avoid_turn_only_distance:
            twist.linear.x = 0.0
            return

        if front >= self._avoid_forward_distance:
            turn_scale = max(0.25, 1.0 - abs(target.angle) / math.radians(110.0))
            twist.linear.x = self._min_speed * turn_scale

    def _drive_clear_path(
            self,
            twist: Twist,
            target: GapTarget | None,
            left: float,
            right: float,
            front: float):
        twist.linear.x = self._max_speed

        # narrow passage: both sides close → creep and steer to centre
        narrow = (
            math.isfinite(left)
            and math.isfinite(right)
            and min(left, right) < self._narrow_passage_distance
        )
        if narrow:
            twist.linear.x = self._min_speed
            corridor_error = left - right
            twist.angular.z = _clamp(
                self._corridor_kp * corridor_error,
                -self._drive_max_angular_speed,
                self._drive_max_angular_speed,
            )
        elif (
                math.isfinite(left)
                and math.isfinite(right)
                and min(left, right) < self._side_balance_distance):
            corridor_error = left - right
            twist.angular.z = _clamp(
                self._corridor_kp * corridor_error,
                -self._drive_max_angular_speed,
                self._drive_max_angular_speed,
            )
        else:
            twist.angular.z = 0.0

        if front < self._clear_distance:
            twist.linear.x = self._min_speed

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

    def _log_decision(
            self,
            reason: str,
            twist: Twist,
            fused: dict | None = None,
            target: GapTarget | None = None):
        if not self._debug_decisions:
            return

        now = time.monotonic()
        transition = self._debug_transition
        if (
                transition is None
                and now - self._last_debug_time < self._debug_period_sec):
            return

        self._last_debug_time = now
        self._debug_transition = None
        data = self._latest_obstacles or {}
        fused = fused or data.get('fused', {})
        lidar = data.get('lidar', {})
        depth = data.get('depth', {})
        tof = data.get('tof', {})
        ages = data.get('ages', {})
        source = '+'.join(fused.get('source', [])) or 'none'

        front = _finite_or_inf(fused.get('front_distance'))
        left = _finite_or_inf(fused.get('left_distance'))
        right = _finite_or_inf(fused.get('right_distance'))
        rear = _finite_or_inf(fused.get('rear_distance'))
        gap_angle = target.angle if target else _finite_or_inf(
            fused.get('best_gap_angle')
        )
        gap_angle_deg = (
            f'{math.degrees(gap_angle):.1f}deg'
            if math.isfinite(gap_angle)
            else 'none'
        )
        gap_width = int(fused.get('best_gap_width', 0) or 0)

        transition_text = transition or f'{self._state}->same'
        self.get_logger().warn(
            'OBS_DEBUG '
            f'trans={transition_text} reason={reason} state={self._state} '
            f'flags(blocked={bool(fused.get("blocked", False))}, '
            f'raw={bool(fused.get("blocked_raw", False))}, '
            f'held={bool(fused.get("blocked_held", False))}, '
            f'emergency={bool(fused.get("emergency", False))}, '
            f'dead_end={bool(fused.get("dead_end", False))}, '
            f'dynamic={bool(fused.get("dynamic_obstacle", False))}) '
            f'src={source} '
            f'fused(front={_fmt_m(front)}, left={_fmt_m(left)}, '
            f'right={_fmt_m(right)}, rear={_fmt_m(rear)}) '
            f'lidar(front_min={_fmt_opt_m(lidar.get("front_min"))}, '
            f'front_control={_fmt_opt_m(lidar.get("front_control"))}, '
            f'front_mean={_fmt_opt_m(lidar.get("front_mean"))}, '
            f'left={_fmt_opt_m(lidar.get("left_mean"))}, '
            f'right={_fmt_opt_m(lidar.get("right_mean"))}) '
            f'depth(front={_fmt_opt_m(depth.get("front_min"))}, '
            f'left={_fmt_opt_m(depth.get("left_min"))}, '
            f'right={_fmt_opt_m(depth.get("right_min"))}) '
            f'tof(range={_fmt_opt_m(tof.get("range"))}) '
            f'gap(angle={gap_angle_deg}, width={gap_width}) '
            f'cmd(x={twist.linear.x:.2f}, z={twist.angular.z:.2f}) '
            f'ages(scan={_fmt_age(ages.get("scan"))}, '
            f'depth={_fmt_age(ages.get("depth"))}, '
            f'tof={_fmt_age(ages.get("tof"))})'
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

    def _turn_direction_from(
            self,
            target: GapTarget | None,
            left: float,
            right: float) -> float:
        """Prefer gap angle sign; fall back to clearer side."""
        if target is not None and math.isfinite(target.angle) and abs(target.angle) > 0.1:
            return 1.0 if target.angle > 0 else -1.0
        return self._clearer_side(left, right)

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
