"""
Project C reactive decision node.

Policy:
- drive straight at full speed until front obstacle is within stop_distance (15 cm);
- try to squeeze past by moving forward + turning toward the clearer side (DODGE);
- if squeeze fails or both sides are blocked, rotate exactly 90 degrees in place (ROTATE);
- after each 90-degree rotation check if the path is now clear (> clear_distance);
- if still blocked, flip direction and try another 90 degrees — up to max_rotation_attempts;
- when all rotation attempts are exhausted, back up and restart rotation search.

No map, no route, no long-term memory.
"""

import json
import math
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String


DRIVE = 'DRIVE'
DODGE = 'DODGE'
ROTATE = 'ROTATE'
BACKUP = 'BACKUP'
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


def _fmt_cm(value: float) -> str:
    return f'{value * 100:.0f}cm' if math.isfinite(value) else 'inf'


@dataclass
class GapTarget:
    angle: float
    clearance: float
    width: int


@dataclass
class Snapshot:
    front_raw: float
    front: float
    left: float
    right: float
    rear: float
    source: list[str]
    target: GapTarget | None
    lidar_available: bool
    depth_available: bool
    tof_available: bool
    lidar_front: float
    depth_front: float
    lidar_obstacle: bool
    depth_obstacle: bool
    static_obstacle: bool
    dynamic_candidate: bool
    emergency: bool
    tof_emergency: bool
    dead_end: bool
    side_escape: float | None


class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.declare_parameter('obstacle_topic', '/obstacle_representation')
        self.declare_parameter('battery_topic', '/battery')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_stamped', True)
        self.declare_parameter('cmd_vel_frame_id', 'base_link')
        self.declare_parameter('state_topic', '/obstacle_avoidance_state')
        self.declare_parameter('control_hz', 20.0)

        self.declare_parameter('max_speed', 0.10)
        self.declare_parameter('min_speed', 0.05)
        self.declare_parameter('max_angular_speed', 0.35)
        self.declare_parameter('drive_max_angular_speed', 0.12)
        self.declare_parameter('corridor_kp', 0.14)

        # Distance thresholds
        self.declare_parameter('obstacle_distance', 0.15)   # react when front <= this
        self.declare_parameter('clear_distance', 0.20)      # path clear when front >= this
        self.declare_parameter('slow_distance', 0.18)       # slow down approaching this
        self.declare_parameter('front_body_offset_m', 0.11)
        self.declare_parameter('face_wall_distance', 0.10)  # force backup when <= this

        # Dodge: try to squeeze past by going forward + turning
        self.declare_parameter('dodge_clearance', 0.25)     # min side room to attempt dodge
        self.declare_parameter('dodge_forward_speed', 0.02) # linear speed during dodge
        self.declare_parameter('dodge_duration_sec', 1.5)   # max time to attempt dodge

        # Rotate: spin exactly 90 degrees, check, try other direction if needed
        self.declare_parameter('rotation_angular_speed', 0.45)  # rad/s for 90-degree rotation
        self.declare_parameter('max_rotation_attempts', 4)      # attempts before backup

        # Backup
        self.declare_parameter('backup_speed', 0.08)
        self.declare_parameter('backup_sec', 0.90)
        self.declare_parameter('backup_rear_stop_distance', 0.25)

        # Side protection
        self.declare_parameter('side_balance_distance', 0.45)
        self.declare_parameter('side_protect_distance', 0.12)

        # Battery safety
        self.declare_parameter('require_battery_ok', True)
        self.declare_parameter('min_battery_voltage', 11.1)
        self.declare_parameter('warn_battery_voltage', 11.4)
        self.declare_parameter('battery_stale_sec', 3.0)

        self.declare_parameter('debug_decisions', True)
        self.declare_parameter('debug_period_sec', 1.0)

        obstacle_topic = self.get_parameter('obstacle_topic').value
        battery_topic = self.get_parameter('battery_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        state_topic = self.get_parameter('state_topic').value
        self._cmd_vel_stamped = _as_bool(self.get_parameter('cmd_vel_stamped').value)
        self._cmd_vel_frame_id = self.get_parameter('cmd_vel_frame_id').value
        control_hz = float(self.get_parameter('control_hz').value)

        self._max_speed = float(self.get_parameter('max_speed').value)
        self._min_speed = float(self.get_parameter('min_speed').value)
        self._max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self._drive_max_angular_speed = float(
            self.get_parameter('drive_max_angular_speed').value
        )
        self._corridor_kp = float(self.get_parameter('corridor_kp').value)
        self._obstacle_distance = float(self.get_parameter('obstacle_distance').value)
        self._clear_distance = float(self.get_parameter('clear_distance').value)
        self._slow_distance = float(self.get_parameter('slow_distance').value)
        self._front_body_offset_m = max(
            0.0, float(self.get_parameter('front_body_offset_m').value)
        )
        self._face_wall_distance = float(self.get_parameter('face_wall_distance').value)

        self._dodge_clearance = float(self.get_parameter('dodge_clearance').value)
        self._dodge_forward_speed = float(self.get_parameter('dodge_forward_speed').value)
        self._dodge_duration_sec = float(self.get_parameter('dodge_duration_sec').value)

        self._rotation_angular_speed = float(
            self.get_parameter('rotation_angular_speed').value
        )
        self._max_rotation_attempts = int(self.get_parameter('max_rotation_attempts').value)
        self._rotation_90_sec = (math.pi / 2) / max(self._rotation_angular_speed, 0.01)

        self._backup_speed = float(self.get_parameter('backup_speed').value)
        self._backup_sec = float(self.get_parameter('backup_sec').value)
        self._backup_rear_stop_distance = float(
            self.get_parameter('backup_rear_stop_distance').value
        )
        self._side_balance_distance = float(
            self.get_parameter('side_balance_distance').value
        )
        self._side_protect_distance = float(
            self.get_parameter('side_protect_distance').value
        )
        self._require_battery_ok = _as_bool(
            self.get_parameter('require_battery_ok').value
        )
        self._min_battery_voltage = float(self.get_parameter('min_battery_voltage').value)
        self._warn_battery_voltage = float(
            self.get_parameter('warn_battery_voltage').value
        )
        self._battery_stale_sec = float(self.get_parameter('battery_stale_sec').value)
        self._debug_decisions = _as_bool(self.get_parameter('debug_decisions').value)
        self._debug_period_sec = float(self.get_parameter('debug_period_sec').value)

        self._latest_obstacles: dict | None = None
        self._last_obstacle_time: float | None = None
        self._state = NO_OBSTACLES
        self._state_end_time = 0.0
        self._turn_direction = 1.0
        self._dodge_direction = 1.0
        self._rotation_attempts = 0
        self._battery_voltage: float | None = None
        self._last_battery_time: float | None = None
        self._last_battery_warn_time = 0.0
        self._last_debug_time = 0.0
        self._debug_transition: str | None = None

        self.create_subscription(String, obstacle_topic, self._on_obstacles, 10)
        self.create_subscription(BatteryState, battery_topic, self._on_battery, 10)
        vel_type = TwistStamped if self._cmd_vel_stamped else Twist
        self._vel_pub = self.create_publisher(vel_type, cmd_vel_topic, 10)
        self._state_pub = self.create_publisher(String, state_topic, 10)
        self.create_timer(1.0 / max(control_hz, 1.0), self._control_loop)

        self.get_logger().info(
            f'Obstacle decision ready. obstacles={obstacle_topic}, '
            f'cmd_vel={cmd_vel_topic} ({vel_type.__name__}), '
            f'battery={battery_topic}, '
            f'stop_dist={self._obstacle_distance*100:.0f}cm '
            f'clear_dist={self._clear_distance*100:.0f}cm '
            f'rotation_90_sec={self._rotation_90_sec:.1f}s '
            f'debug_decisions={self._debug_decisions}'
        )

    def _on_obstacles(self, msg: String):
        try:
            self._latest_obstacles = json.loads(msg.data)
            self._last_obstacle_time = time.monotonic()
        except json.JSONDecodeError:
            self.get_logger().warn('Ignored invalid obstacle representation JSON.')

    def _on_battery(self, msg: BatteryState):
        if math.isfinite(msg.voltage) and msg.voltage > 0.0:
            self._battery_voltage = float(msg.voltage)
            self._last_battery_time = time.monotonic()

    def _obstacles_recent(self) -> bool:
        return (
            self._last_obstacle_time is not None
            and time.monotonic() - self._last_obstacle_time <= 1.0
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

        if self._battery_stop_required(now):
            self._transition(EMERGENCY)
            self._log_battery_safety(now)
            self._publish_velocity(twist)
            return

        self._log_battery_safety(now)

        if self._latest_obstacles is None or not self._obstacles_recent():
            self._transition(NO_OBSTACLES)
            self._log_decision('no_fresh_obstacle_representation')
            self._publish_velocity(twist)
            return

        snapshot = self._snapshot()

        if not self._any_sensor_available():
            self._transition(NO_OBSTACLES)
            self._log_decision('no_active_sensors', snapshot)
            self._publish_velocity(twist)
            return

        if snapshot.tof_emergency:
            self._transition(EMERGENCY)
            self._log_decision('tof_emergency_stop', snapshot)
            self._publish_velocity(twist)
            return

        # Continue active avoidance states
        if self._state == BACKUP:
            if self._handle_backup(twist, snapshot, now):
                self._log_decision('backup', snapshot)
                self._publish_velocity(twist)
                return

        if self._state == DODGE:
            if self._handle_dodge(twist, snapshot, now):
                self._log_decision('dodge', snapshot)
                self._publish_velocity(twist)
                return

        if self._state == ROTATE:
            if self._handle_rotate(twist, snapshot, now):
                self._log_decision('rotate', snapshot)
                self._publish_velocity(twist)
                return

        # Force backup when dangerously close (overrides new obstacle decisions)
        if self._needs_backup(snapshot):
            self._start_backup(snapshot, now)
            self._handle_backup(twist, snapshot, now)
            self._log_decision('face_wall_backup', snapshot)
            self._publish_velocity(twist)
            return

        # Obstacle at stop distance: try dodge first, fall back to rotate
        if snapshot.front <= self._obstacle_distance or snapshot.static_obstacle:
            has_side_room = (
                snapshot.left > self._dodge_clearance
                or snapshot.right > self._dodge_clearance
            )
            if has_side_room and not snapshot.dead_end:
                self._start_dodge(snapshot, now)
                self._handle_dodge(twist, snapshot, now)
                self._log_decision('dodge_start', snapshot)
            else:
                self._start_rotate(snapshot, now)
                self._handle_rotate(twist, snapshot, now)
                self._log_decision('rotate_start', snapshot)
            self._publish_velocity(twist)
            return

        # Drive straight
        self._transition(DRIVE)
        self._drive_straight(twist, snapshot)
        self._log_decision('drive_straight', snapshot)
        self._publish_velocity(twist)

    # ── DODGE: move forward slowly while turning toward the clearer side ──────

    def _start_dodge(self, snapshot: Snapshot, now: float):
        self._dodge_direction = self._clearer_side(snapshot.left, snapshot.right)
        if snapshot.side_escape is not None:
            self._dodge_direction = snapshot.side_escape
        self._transition(DODGE, self._dodge_duration_sec)

    def _handle_dodge(self, twist: Twist, snapshot: Snapshot, now: float) -> bool:
        # Success: squeezed through
        if snapshot.front >= self._clear_distance and snapshot.side_escape is None:
            self._transition(DRIVE)
            return False

        # Give up: hit wall or timed out
        if snapshot.front <= self._face_wall_distance or now >= self._state_end_time:
            self._start_rotate(snapshot, now)
            return True

        # Side escape overrides dodge direction
        direction = snapshot.side_escape if snapshot.side_escape is not None \
            else self._dodge_direction

        twist.linear.x = self._dodge_forward_speed
        twist.angular.z = direction * self._max_angular_speed
        return True

    # ── ROTATE: spin exactly 90 degrees, check, try other direction if needed ─

    def _start_rotate(self, snapshot: Snapshot, now: float):
        self._turn_direction = self._clearer_side(snapshot.left, snapshot.right)
        if snapshot.side_escape is not None:
            self._turn_direction = snapshot.side_escape
        self._rotation_attempts = 0
        self._transition(ROTATE, self._rotation_90_sec)

    def _handle_rotate(self, twist: Twist, snapshot: Snapshot, now: float) -> bool:
        # Side escape can override turn direction during rotation
        if snapshot.side_escape is not None:
            self._turn_direction = snapshot.side_escape

        # Still rotating this 90-degree slice
        if now < self._state_end_time:
            twist.linear.x = 0.0
            twist.angular.z = self._turn_direction * self._rotation_angular_speed
            return True

        # 90 degrees done — check if the path ahead is clear
        if snapshot.front >= self._clear_distance and snapshot.side_escape is None:
            self._transition(DRIVE)
            return False

        # Still blocked: try the opposite 90 degrees if attempts remain
        self._rotation_attempts += 1
        if self._rotation_attempts < self._max_rotation_attempts:
            self._turn_direction = -self._turn_direction
            self._state_end_time = now + self._rotation_90_sec
            self.get_logger().info(
                f'Rotate attempt {self._rotation_attempts}/{self._max_rotation_attempts}'
                f' — flipping to {"left" if self._turn_direction > 0 else "right"}'
            )
            twist.linear.x = 0.0
            twist.angular.z = self._turn_direction * self._rotation_angular_speed
            return True

        # All rotation attempts exhausted — back up and try again
        self.get_logger().warn('All rotation attempts failed — backing up')
        self._start_backup(snapshot, now)
        return True

    # ── BACKUP ────────────────────────────────────────────────────────────────

    def _needs_backup(self, snapshot: Snapshot) -> bool:
        if snapshot.dynamic_candidate:
            return False
        if not (snapshot.static_obstacle or snapshot.emergency):
            return False
        return math.isfinite(snapshot.front) and snapshot.front <= self._face_wall_distance

    def _start_backup(self, snapshot: Snapshot, now: float):
        self._transition(BACKUP, self._backup_sec)

    def _handle_backup(self, twist: Twist, snapshot: Snapshot, now: float) -> bool:
        rear_blocked = (
            math.isfinite(snapshot.rear)
            and snapshot.rear < self._backup_rear_stop_distance
        )
        if not rear_blocked and now < self._state_end_time:
            twist.linear.x = -abs(self._backup_speed)
            twist.angular.z = 0.0
            return True

        # Backup done — start a fresh rotation search
        self._start_rotate(snapshot, now)
        twist.linear.x = 0.0
        twist.angular.z = self._turn_direction * self._rotation_angular_speed
        return True

    # ── DRIVE straight with corridor centering ────────────────────────────────

    def _drive_straight(self, twist: Twist, snapshot: Snapshot):
        twist.linear.x = self._max_speed
        twist.angular.z = 0.0

        if snapshot.front < self._slow_distance:
            twist.linear.x = self._min_speed

        if (
                math.isfinite(snapshot.left)
                and math.isfinite(snapshot.right)
                and min(snapshot.left, snapshot.right) < self._side_balance_distance):
            corridor_error = snapshot.left - snapshot.right
            twist.angular.z = _clamp(
                self._corridor_kp * corridor_error,
                -self._drive_max_angular_speed,
                self._drive_max_angular_speed,
            )

    # ── Snapshot ──────────────────────────────────────────────────────────────

    def _snapshot(self) -> Snapshot:
        data = self._latest_obstacles or {}
        fused = data.get('fused', {})
        lidar = data.get('lidar', {})
        depth = data.get('depth', {})
        tof = data.get('tof', {})

        source = list(fused.get('source', []))
        front_raw = _finite_or_inf(fused.get('front_distance'))
        front = self._front_spacing(front_raw)
        left = _finite_or_inf(fused.get('left_distance'))
        right = _finite_or_inf(fused.get('right_distance'))
        rear = _finite_or_inf(fused.get('rear_distance'))
        target = self._target_from_fused(fused)

        lidar_available = bool(lidar.get('available', False))
        depth_available = bool(depth.get('available', False))
        tof_available = bool(tof.get('available', False))
        lidar_front = self._front_spacing(_finite_or_inf(lidar.get('front_control')))
        depth_front = self._front_spacing(_finite_or_inf(depth.get('front_min')))

        lidar_obstacle = lidar_available and (
            'lidar' in source
            or lidar_front < self._obstacle_distance
        )
        depth_obstacle = depth_available and (
            'depth' in source
            or depth_front < self._obstacle_distance
        )
        static_obstacle = lidar_obstacle
        dynamic_candidate = bool(
            fused.get('dynamic_obstacle', False)
            or (not lidar_obstacle and depth_obstacle)
        )
        emergency = bool(fused.get('emergency', False))
        tof_emergency = bool(emergency and 'tof' in source)
        dead_end = bool(
            fused.get('dead_end', False)
            or (
                front < self._obstacle_distance
                and left < self._obstacle_distance
                and right < self._obstacle_distance
            )
        )
        side_escape = self._side_escape_direction(left, right)

        return Snapshot(
            front_raw=front_raw,
            front=front,
            left=left,
            right=right,
            rear=rear,
            source=source,
            target=target,
            lidar_available=lidar_available,
            depth_available=depth_available,
            tof_available=tof_available,
            lidar_front=lidar_front,
            depth_front=depth_front,
            lidar_obstacle=lidar_obstacle,
            depth_obstacle=depth_obstacle,
            static_obstacle=static_obstacle,
            dynamic_candidate=dynamic_candidate,
            emergency=emergency,
            tof_emergency=tof_emergency,
            dead_end=dead_end,
            side_escape=side_escape,
        )

    def _front_spacing(self, value: float) -> float:
        if not math.isfinite(value):
            return value
        return max(0.0, value - self._front_body_offset_m)

    @staticmethod
    def _target_from_fused(fused: dict) -> GapTarget | None:
        width = int(fused.get('best_gap_width', 0) or 0)
        angle = _finite_or_inf(fused.get('best_gap_angle'))
        clearance = _finite_or_inf(fused.get('best_gap_clearance'))
        if width <= 0 or not math.isfinite(angle):
            return None
        return GapTarget(angle=angle, clearance=clearance, width=width)

    def _any_sensor_available(self) -> bool:
        data = self._latest_obstacles or {}
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

    def _side_escape_direction(self, left: float, right: float) -> float | None:
        left_close = math.isfinite(left) and left < self._side_protect_distance
        right_close = math.isfinite(right) and right < self._side_protect_distance
        if left_close and (not right_close or left <= right):
            return -1.0
        if right_close:
            return 1.0
        return None

    # ── Logging ───────────────────────────────────────────────────────────────

    def _log_decision(self, reason: str, snapshot: Snapshot | None = None):
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
        snapshot = snapshot or self._snapshot()

        nearest = min(
            v for v in (snapshot.front_raw, snapshot.left, snapshot.right, snapshot.rear)
            if math.isfinite(v)
        ) if any(
            math.isfinite(v)
            for v in (snapshot.front_raw, snapshot.left, snapshot.right, snapshot.rear)
        ) else math.inf

        nearest_cm = _fmt_cm(nearest)
        front_cm = _fmt_cm(snapshot.front_raw)
        left_cm = _fmt_cm(snapshot.left)
        right_cm = _fmt_cm(snapshot.right)

        if self._state in (DRIVE, NO_OBSTACLES):
            self.get_logger().info(
                f'[NAV] state={self._state} nearest={nearest_cm} '
                f'front={front_cm} left={left_cm} right={right_cm} reason={reason}'
            )
            return

        obs_type = 'dynamic' if snapshot.dynamic_candidate else 'static'
        dead_end = 'YES' if snapshot.dead_end else 'no'
        spacing_cm = _fmt_cm(snapshot.front)

        self.get_logger().warn(
            f'[OBSTACLE] state={self._state} type={obs_type} '
            f'nearest={nearest_cm} front={front_cm} spacing={spacing_cm} '
            f'left={left_cm} right={right_cm} '
            f'dead_end={dead_end} reason={reason}'
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

    # ── Battery safety ────────────────────────────────────────────────────────

    def _battery_stop_required(self, now: float) -> bool:
        if self._battery_voltage is None or self._last_battery_time is None:
            return self._require_battery_ok
        if now - self._last_battery_time > self._battery_stale_sec:
            return self._require_battery_ok
        return self._battery_voltage < self._min_battery_voltage

    def _log_battery_safety(self, now: float):
        if now - self._last_battery_warn_time < 5.0:
            return

        if self._battery_voltage is None or self._last_battery_time is None:
            if self._require_battery_ok:
                self._last_battery_warn_time = now
                self.get_logger().error(
                    '[BATTERY] no battery reading yet; stopping robot.'
                )
            return

        if now - self._last_battery_time > self._battery_stale_sec:
            if self._require_battery_ok:
                age = now - self._last_battery_time
                self._last_battery_warn_time = now
                self.get_logger().error(
                    f'[BATTERY] stale battery reading age={age:.1f}s; stopping robot.'
                )
            return

        if self._battery_voltage >= self._warn_battery_voltage:
            return

        self._last_battery_warn_time = now
        if self._battery_voltage < self._min_battery_voltage:
            self.get_logger().error(
                f'[BATTERY] voltage={self._battery_voltage:.2f}V below '
                f'min={self._min_battery_voltage:.2f}V; stopping robot.'
            )
        else:
            self.get_logger().warn(
                f'[BATTERY] voltage={self._battery_voltage:.2f}V below '
                f'warning={self._warn_battery_voltage:.2f}V; test gently.'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
