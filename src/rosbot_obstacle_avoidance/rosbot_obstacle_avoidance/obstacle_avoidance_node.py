"""
Project C reactive decision node.

The controller is intentionally conservative and deterministic:
drive straight when the fused front sector is clear, rotate away from obstacles,
and only reverse when the front is genuinely blocked and the rear is clear.
"""

import json
import math
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
        self.declare_parameter('max_angular_speed', 0.35)
        self.declare_parameter('drive_max_angular_speed', 0.12)
        self.declare_parameter('gap_kp', 0.65)
        self.declare_parameter('corridor_kp', 0.14)

        self.declare_parameter('obstacle_distance', 0.55)
        self.declare_parameter('clear_distance', 0.85)
        self.declare_parameter('dynamic_stop_distance', 0.90)
        self.declare_parameter('avoid_turn_only_distance', 0.65)
        self.declare_parameter('avoid_forward_distance', 0.95)
        self.declare_parameter('backup_clear_distance', 0.45)
        self.declare_parameter('dynamic_hold_sec', 0.80)
        self.declare_parameter('obstacle_stale_sec', 1.0)
        self.declare_parameter('backup_sec', 0.35)
        self.declare_parameter('turn_out_sec', 0.90)
        self.declare_parameter('side_balance_distance', 0.80)
        self.declare_parameter('turn_direction_hold_sec', 0.80)
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
        self._side_balance_distance = float(
            self.get_parameter('side_balance_distance').value
        )
        self._turn_direction_hold_sec = float(
            self.get_parameter('turn_direction_hold_sec').value
        )
        self._debug_decisions = _as_bool(self.get_parameter('debug_decisions').value)
        self._debug_period_sec = float(self.get_parameter('debug_period_sec').value)

        self._latest_obstacles: dict | None = None
        self._last_obstacle_time: float | None = None
        self._state = NO_OBSTACLES
        self._state_end_time = 0.0
        self._turn_direction = 1.0
        self._turn_direction_until = 0.0
        self._dynamic_hold_until = 0.0
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

        if self._run_timed_recovery(twist, now, front, left, right, rear, target):
            self._log_decision(self._state.lower(), twist, fused, target)
            self._publish_velocity(twist)
            return

        if dynamic_obstacle and front < self._dynamic_stop_distance:
            self._dynamic_hold_until = now + self._dynamic_hold_sec
            self._transition(DYNAMIC_AVOID, self._dynamic_hold_sec)
            self._log_decision('dynamic_stop', twist, fused, target)
            self._publish_velocity(twist)
            return

        if emergency:
            self._start_recovery(front, left, right, rear, target)
            self._log_decision('emergency_recovery', twist, fused, target)
            self._publish_velocity(twist)
            return

        if dead_end:
            self._start_recovery(front, left, right, rear, target)
            self._log_decision('dead_end_recovery', twist, fused, target)
            self._publish_velocity(twist)
            return

        if blocked or front < self._obstacle_distance:
            self._transition(AVOID)
            self._drive_avoid(twist, target, front, left, right, now)
            self._log_decision('avoid_obstacle', twist, fused, target)
            self._publish_velocity(twist)
            return

        self._transition(DRIVE)
        self._drive_clear(twist, front, left, right)
        self._log_decision('drive_clear', twist, fused, target)
        self._publish_velocity(twist)

    def _run_timed_recovery(
            self,
            twist: Twist,
            now: float,
            front: float,
            left: float,
            right: float,
            rear: float,
            target: GapTarget | None) -> bool:
        if self._state == DYNAMIC_AVOID and now < self._dynamic_hold_until:
            return True

        if self._state == BACKUP:
            if rear <= self._backup_clear_distance:
                self._set_turn_direction(target, left, right, now, force=True)
                self._transition(TURN_OUT, self._turn_out_sec)
                return True

            if now < self._state_end_time:
                twist.linear.x = -self._reverse_speed
                return True

            self._set_turn_direction(target, left, right, now, force=True)
            self._transition(TURN_OUT, self._turn_out_sec)
            return True

        if self._state == TURN_OUT:
            if now < self._state_end_time:
                twist.angular.z = self._turn_direction * self._max_angular_speed
                return True

            if front < self._avoid_forward_distance:
                self._transition(AVOID)
                return False

            self._transition(DRIVE)
            return False

        if self._state == EMERGENCY:
            self._transition(AVOID if front < self._clear_distance else DRIVE)

        return False

    def _start_recovery(
            self,
            front: float,
            left: float,
            right: float,
            rear: float,
            target: GapTarget | None):
        now = time.monotonic()
        self._set_turn_direction(target, left, right, now, force=True)

        if front < self._avoid_turn_only_distance and rear > self._backup_clear_distance:
            self._transition(BACKUP, self._backup_sec)
            return

        self._transition(TURN_OUT, self._turn_out_sec)

    def _drive_avoid(
            self,
            twist: Twist,
            target: GapTarget | None,
            front: float,
            left: float,
            right: float,
            now: float):
        direction = self._set_turn_direction(target, left, right, now)

        if front >= self._avoid_forward_distance and target is not None:
            twist.angular.z = _clamp(
                self._gap_kp * target.angle,
                -self._max_angular_speed,
                self._max_angular_speed,
            )
            if abs(target.angle) < math.radians(25.0):
                twist.linear.x = self._min_speed
            return

        twist.angular.z = direction * self._max_angular_speed

    def _drive_clear(self, twist: Twist, front: float, left: float, right: float):
        twist.linear.x = self._max_speed
        twist.angular.z = 0.0

        if front < self._clear_distance:
            twist.linear.x = self._min_speed

        if (
                math.isfinite(left)
                and math.isfinite(right)
                and min(left, right) < self._side_balance_distance):
            corridor_error = left - right
            twist.angular.z = _clamp(
                self._corridor_kp * corridor_error,
                -self._drive_max_angular_speed,
                self._drive_max_angular_speed,
            )

    def _set_turn_direction(
            self,
            target: GapTarget | None,
            left: float,
            right: float,
            now: float,
            force: bool = False) -> float:
        if not force and now < self._turn_direction_until:
            return self._turn_direction

        if target is not None and math.isfinite(target.angle) and abs(target.angle) > 0.12:
            self._turn_direction = 1.0 if target.angle > 0.0 else -1.0
        else:
            self._turn_direction = self._clearer_side(left, right)

        self._turn_direction_until = now + self._turn_direction_hold_sec
        return self._turn_direction

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

    @staticmethod
    def _clearer_side(left: float, right: float) -> float:
        if not math.isfinite(left) and not math.isfinite(right):
            return 1.0
        if not math.isfinite(left):
            return -1.0
        if not math.isfinite(right):
            return 1.0
        return 1.0 if left >= right else -1.0

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
            f'front_close={int(lidar.get("front_close_count", 0) or 0)}/'
            f'{int(lidar.get("front_samples", 0) or 0)}, '
            f'left={_fmt_opt_m(lidar.get("left_mean"))}, '
            f'right={_fmt_opt_m(lidar.get("right_mean"))}) '
            f'depth(front={_fmt_opt_m(depth.get("front_min"))}, '
            f'left={_fmt_opt_m(depth.get("left_min"))}, '
            f'right={_fmt_opt_m(depth.get("right_min"))}) '
            f'tof(range={_fmt_opt_m(tof.get("range"))}) '
            f'gap(angle={gap_angle_deg}, width={gap_width}) '
            f'turn_dir={self._turn_direction:+.0f} '
            f'cmd(x={twist.linear.x:.2f}, z={twist.angular.z:.2f}) '
            f'ages(scan={_fmt_age(ages.get("scan"))}, '
            f'depth={_fmt_age(ages.get("depth"))}, '
            f'tof={_fmt_age(ages.get("tof"))})'
        )

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
