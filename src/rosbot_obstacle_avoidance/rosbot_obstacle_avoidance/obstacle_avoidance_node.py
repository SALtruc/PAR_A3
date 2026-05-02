"""
Project C reactive decision node.

Policy:
- drive forward when the fused obstacle representation is fresh and safe;
- slow down in narrow passages and steer toward the larger side clearance;
- if the front is blocked, rotate toward the clearer side and re-check the front;
- if a side is dangerously close, rotate away from that side with a locked direction;
- during one rotation search, keep the same rotation direction to avoid left-right oscillation;
- after each rotation step, drive only when the front is clear and both sides are safe;
- if no clear heading is found after max_rotation_attempts, back up and restart;
- hard-stop on ToF emergency.

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
OBSERVE = 'OBSERVE'
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
        # Drive-first policy thresholds
        self.declare_parameter('observe_distance', 0.25)  # 15-25cm: observe several frames
        self.declare_parameter('near_stop_distance', 0.15) # <=15cm: stop/backup/check
        self.declare_parameter('observe_frames', 5)
        self.declare_parameter('avoidance_distance', 0.50) # 25-50cm: dodge/steer, no full stop
        self.declare_parameter('max_dodge_angle_deg', 45.0)
        self.declare_parameter('rotation_step_deg', 45.0)
        self.declare_parameter('side_drive_slow_distance', 0.08)
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
        self._observe_distance = float(self.get_parameter('observe_distance').value)
        self._near_stop_distance = float(self.get_parameter('near_stop_distance').value)
        self._observe_frames = max(1, int(self.get_parameter('observe_frames').value))
        self._avoidance_distance = float(self.get_parameter('avoidance_distance').value)
        self._max_dodge_angle = math.radians(float(self.get_parameter('max_dodge_angle_deg').value))
        self._rotation_step_angle = math.radians(float(self.get_parameter('rotation_step_deg').value))
        self._side_drive_slow_distance = float(self.get_parameter('side_drive_slow_distance').value)
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
        self._rotation_90_sec = self._rotation_step_angle / max(self._rotation_angular_speed, 0.01)

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
        self._observe_count = 0
        self._observe_first_front = math.inf
        self._observe_min_front = math.inf
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
            f'rotation_step={math.degrees(self._rotation_step_angle):.0f}deg/{self._rotation_90_sec:.1f}s '
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
        """Priority policy: drive first; avoid only when the front path is not usable."""
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

        # Continue active states first.
        if self._state == BACKUP:
            if self._handle_backup(twist, snapshot, now):
                self._log_decision('backup', snapshot)
                self._publish_velocity(twist)
                return

        if self._state == OBSERVE:
            if self._handle_observe(twist, snapshot, now):
                self._log_decision('observe_frames', snapshot)
                self._publish_velocity(twist)
                return

        if self._state == DODGE:
            if self._handle_dodge(twist, snapshot, now):
                self._log_decision('dodge_limited_angle', snapshot)
                self._publish_velocity(twist)
                return

        if self._state == ROTATE:
            if self._handle_rotate(twist, snapshot, now):
                self._log_decision('rotate_search', snapshot)
                self._publish_velocity(twist)
                return

        # 1) Too close is different from sudden appearance.
        # <= 15cm after body-offset means stop/back up/check immediately.
        if self._too_close(snapshot):
            self._start_backup(snapshot, now)
            self._handle_backup(twist, snapshot, now)
            self._log_decision('too_close_backup', snapshot)
            self._publish_velocity(twist)
            return

        # 2) Dead-end: front + both sides blocked, cannot pass through.
        if self._dead_end_now(snapshot):
            self._start_backup(snapshot, now)
            self._handle_backup(twist, snapshot, now)
            self._log_decision('dead_end_backup', snapshot)
            self._publish_velocity(twist)
            return

        # 3) Sudden/dynamic object in the 15-25cm band: observe 4-5 frames.
        # If it disappears, keep driving. If it persists/closes, avoid.
        if self._should_observe(snapshot):
            self._start_observe(snapshot, now)
            self._handle_observe(twist, snapshot, now)
            self._log_decision('observe_start', snapshot)
            self._publish_velocity(twist)
            return

        # 4) Medium obstacle in front: do not full stop. Try small dodge/steer,
        # limited to about 45 degrees. Only rotate if there is no usable gap.
        if self._front_needs_avoid(snapshot):
            if self._can_dodge_forward(snapshot):
                self._start_dodge(snapshot, now)
                self._handle_dodge(twist, snapshot, now)
                self._log_decision('medium_front_dodge', snapshot)
            else:
                self._start_rotate(snapshot, now)
                self._handle_rotate(twist, snapshot, now)
                self._log_decision('front_blocked_rotate', snapshot)
            self._publish_velocity(twist)
            return

        # 5) Side close but front is usable: do NOT rotate in place. Keep moving
        # slowly and steer away. This handles corridors/narrow passages.
        if snapshot.side_escape is not None and self._front_is_usable(snapshot):
            self._transition(DRIVE)
            self._drive_straight(twist, snapshot, force_slow=True)
            self._log_decision('side_close_drive_away', snapshot)
            self._publish_velocity(twist)
            return

        # 6) Default: drive straight / roam. Front is clear enough for the robot.
        self._transition(DRIVE)
        self._drive_straight(twist, snapshot)
        self._log_decision('drive_first', snapshot)
        self._publish_velocity(twist)

    # ── OBSERVE: short-frame confirmation for sudden/dynamic objects ────────

    def _start_observe(self, snapshot: Snapshot, now: float):
        self._observe_count = 0
        self._observe_first_front = snapshot.front
        self._observe_min_front = snapshot.front
        self._transition(OBSERVE)

    def _handle_observe(self, twist: Twist, snapshot: Snapshot, now: float) -> bool:
        self._observe_count += 1
        self._observe_min_front = min(self._observe_min_front, snapshot.front)

        # If the object vanished / front path became usable, continue driving.
        if self._front_is_usable(snapshot):
            self._transition(DRIVE)
            return False

        # If it became too close during observation, back up immediately.
        if self._too_close(snapshot):
            self._start_backup(snapshot, now)
            return True

        # Collect 4-5 frames. Move very slowly instead of fully stopping.
        if self._observe_count < self._observe_frames:
            twist.linear.x = min(self._min_speed, 0.015)
            twist.angular.z = self._soft_avoid_angular(snapshot)
            return True

        # Persistent obstacle after observation: dodge if possible, otherwise rotate.
        if self._can_dodge_forward(snapshot):
            self._start_dodge(snapshot, now)
        else:
            self._start_rotate(snapshot, now)
        return True

    # ── DODGE: forward motion + limited steering, not a full in-place turn ─────

    def _start_dodge(self, snapshot: Snapshot, now: float):
        self._dodge_direction = self._choose_dodge_direction(snapshot)
        self._transition(DODGE, self._dodge_duration_sec)

    def _handle_dodge(self, twist: Twist, snapshot: Snapshot, now: float) -> bool:
        # Done: front path is now usable for the robot. Side can still be close;
        # DRIVE will continue steering away while moving forward.
        if self._front_is_usable(snapshot):
            self._transition(DRIVE)
            return False

        # Too close or dodge timeout: rotate/backup search.
        if self._too_close(snapshot):
            self._start_backup(snapshot, now)
            return True
        if now >= self._state_end_time:
            self._start_rotate(snapshot, now)
            return True

        # Keep moving slowly and steer no more than about 45 degrees.
        twist.linear.x = self._dodge_forward_speed
        twist.angular.z = _clamp(
            self._dodge_direction * self._max_angular_speed,
            -self._max_angular_speed,
            self._max_angular_speed,
        )
        return True

    # ── ROTATE: only for truly blocked headings / dead ends ───────────────────

    def _start_rotate(self, snapshot: Snapshot, now: float):
        """Start an escape rotation and lock the chosen direction."""
        if self._state != ROTATE:
            self._rotation_attempts = 0
            self._turn_direction = self._choose_escape_direction(snapshot)
        self._transition(ROTATE, self._rotation_90_sec)

    def _handle_rotate(self, twist: Twist, snapshot: Snapshot, now: float) -> bool:
        # If at any point the face/front path is usable, go forward immediately.
        # Do NOT require side_escape == None; side-close is handled while driving.
        if self._front_is_usable(snapshot):
            self._rotation_attempts = 0
            self._transition(DRIVE)
            return False

        if now < self._state_end_time:
            twist.linear.x = 0.0
            twist.angular.z = self._turn_direction * self._rotation_angular_speed
            return True

        self._rotation_attempts += 1

        # Continue the same direction for a few 45-degree sweeps.
        if self._rotation_attempts < self._max_rotation_attempts:
            self._state_end_time = now + self._rotation_90_sec
            self.get_logger().info(
                f'Rotate sweep {self._rotation_attempts}/{self._max_rotation_attempts} '
                f'continuing {"left" if self._turn_direction > 0 else "right"}; '
                f'front={_fmt_cm(snapshot.front_raw)} spacing={_fmt_cm(snapshot.front)} '
                f'left={_fmt_cm(snapshot.left)} right={_fmt_cm(snapshot.right)} '
                f'gap={self._gap_summary(snapshot)}'
            )
            twist.linear.x = 0.0
            twist.angular.z = self._turn_direction * self._rotation_angular_speed
            return True

        # Try opposite direction once from current pose before backing up.
        if self._rotation_attempts < self._max_rotation_attempts * 2:
            self._turn_direction = -self._turn_direction
            self._state_end_time = now + self._rotation_90_sec
            self.get_logger().info(
                f'Rotate switching to {"left" if self._turn_direction > 0 else "right"}; '
                f'front={_fmt_cm(snapshot.front_raw)} spacing={_fmt_cm(snapshot.front)}'
            )
            twist.linear.x = 0.0
            twist.angular.z = self._turn_direction * self._rotation_angular_speed
            return True

        self.get_logger().warn(
            'Rotate search failed to find usable front path — backing up and retrying'
        )
        self._start_backup(snapshot, now)
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        return True

    # ── Policy helper predicates ──────────────────────────────────────────────

    def _too_close(self, snapshot: Snapshot) -> bool:
        return math.isfinite(snapshot.front) and snapshot.front <= self._near_stop_distance

    def _front_is_usable(self, snapshot: Snapshot) -> bool:
        """True when the robot's current face/front path is wide and clear enough."""
        if not math.isfinite(snapshot.front):
            return True
        if snapshot.front >= self._clear_distance:
            return True
        # If perception found a central gap close to heading, treat it as usable.
        if snapshot.target is not None:
            return (
                abs(snapshot.target.angle) <= self._max_dodge_angle
                and snapshot.target.clearance >= self._clear_distance
            )
        return False

    def _front_needs_avoid(self, snapshot: Snapshot) -> bool:
        return (
            math.isfinite(snapshot.front)
            and self._observe_distance < snapshot.front < self._avoidance_distance
        )

    def _should_observe(self, snapshot: Snapshot) -> bool:
        if not math.isfinite(snapshot.front):
            return False
        return self._near_stop_distance < snapshot.front <= self._observe_distance

    def _dead_end_now(self, snapshot: Snapshot) -> bool:
        if snapshot.dead_end:
            return True
        return (
            math.isfinite(snapshot.front)
            and snapshot.front < self._observe_distance
            and snapshot.left < self._dodge_clearance
            and snapshot.right < self._dodge_clearance
        )

    def _can_dodge_forward(self, snapshot: Snapshot) -> bool:
        if self._too_close(snapshot):
            return False
        # Need at least one side with enough clearance to slide around.
        side_room = max(snapshot.left, snapshot.right)
        if not math.isfinite(side_room):
            side_room = self._dodge_clearance
        if side_room < self._dodge_clearance:
            return False
        if snapshot.target is None:
            return True
        return abs(snapshot.target.angle) <= self._max_dodge_angle

    def _choose_dodge_direction(self, snapshot: Snapshot) -> float:
        # Prefer the gap angle if it is within +-45 degrees.
        if snapshot.target is not None and abs(snapshot.target.angle) <= self._max_dodge_angle:
            if abs(snapshot.target.angle) < math.radians(5):
                return self._clearer_side(snapshot.left, snapshot.right)
            return 1.0 if snapshot.target.angle > 0 else -1.0
        if snapshot.side_escape is not None:
            return snapshot.side_escape
        return self._clearer_side(snapshot.left, snapshot.right)

    def _choose_escape_direction(self, snapshot: Snapshot) -> float:
        # Rotate toward the best gap if available, otherwise toward larger space.
        if snapshot.target is not None and math.isfinite(snapshot.target.angle):
            if abs(snapshot.target.angle) > math.radians(5):
                return 1.0 if snapshot.target.angle > 0 else -1.0
        if snapshot.side_escape is not None and not self._front_is_usable(snapshot):
            return snapshot.side_escape
        return self._clearer_side(snapshot.left, snapshot.right)

    def _soft_avoid_angular(self, snapshot: Snapshot) -> float:
        direction = self._choose_dodge_direction(snapshot)
        return _clamp(
            direction * self._drive_max_angular_speed,
            -self._drive_max_angular_speed,
            self._drive_max_angular_speed,
        )

    def _gap_summary(self, snapshot: Snapshot) -> str:
        if snapshot.target is None:
            return 'none'
        return (
            f'{math.degrees(snapshot.target.angle):.0f}deg/'
            f'{_fmt_cm(snapshot.target.clearance)}/w{snapshot.target.width}'
        )

    # ── BACKUP ────────────────────────────────────────────────────────────────

    def _needs_backup(self, snapshot: Snapshot) -> bool:
        if snapshot.dynamic_candidate:
            return False
        if not (snapshot.static_obstacle or snapshot.emergency):
            return False
        return math.isfinite(snapshot.front) and snapshot.front <= self._face_wall_distance

    def _start_backup(self, snapshot: Snapshot, now: float):
        self._rotation_attempts = 0  # fresh start after backing up
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

    def _drive_straight(self, twist: Twist, snapshot: Snapshot, force_slow: bool = False):
        twist.linear.x = self._max_speed
        twist.angular.z = 0.0

        if force_slow or snapshot.front < self._slow_distance:
            twist.linear.x = self._min_speed

        # If only one side is dangerously close, keep driving but steer away.
        if snapshot.side_escape is not None:
            twist.linear.x = min(twist.linear.x, self._min_speed)
            twist.angular.z = _clamp(
                snapshot.side_escape * self._drive_max_angular_speed,
                -self._drive_max_angular_speed,
                self._drive_max_angular_speed,
            )
            return

        # Corridor centering: turn slightly toward the side with more room.
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

    def _sensor_status_summary(self, snapshot: Snapshot) -> str:
        """Human-readable sensor availability for debug logs."""
        data = self._latest_obstacles or {}
        depth = data.get('depth', {})
        tof = data.get('tof', {})
        ages = data.get('ages', {})

        lidar_age = ages.get('scan')
        depth_age = ages.get('depth')
        tof_age = ages.get('tof')

        def age_text(value) -> str:
            try:
                value_f = float(value)
            except (TypeError, ValueError):
                return 'none'
            return f'{value_f:.2f}s' if math.isfinite(value_f) else 'none'

        return (
            f'lidar={"on" if snapshot.lidar_available else "off"}'
            f'(age={age_text(lidar_age)}) '
            f'depth={"on" if snapshot.depth_available else "off"}'
            f'(image={bool(depth.get("image_available", False))},'
            f'pc={bool(depth.get("pointcloud_available", False))},'
            f'age={age_text(depth_age)}) '
            f'tof={"on" if snapshot.tof_available else "off"}'
            f'(range={_fmt_cm(_finite_or_inf(tof.get("range")))},'
            f'age={age_text(tof_age)})'
        )

    def _detection_source_summary(self, snapshot: Snapshot) -> str:
        """Shows which sensor currently contributes to obstacle detection."""
        source = '+'.join(snapshot.source) if snapshot.source else 'none'
        lidar = 'hit' if snapshot.lidar_obstacle else 'clear'
        depth = 'hit' if snapshot.depth_obstacle else 'clear'
        tof = 'emergency' if snapshot.tof_emergency else 'clear'
        obs_type = 'dynamic' if snapshot.dynamic_candidate else 'static'
        return (
            f'source={source} '
            f'lidar={lidar}(front={_fmt_cm(snapshot.lidar_front)}) '
            f'depth={depth}(front={_fmt_cm(snapshot.depth_front)}) '
            f'tof={tof} type={obs_type}'
        )

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

        sensor_status = self._sensor_status_summary(snapshot)
        detection_source = self._detection_source_summary(snapshot)
        target = (
            f'gap(angle={math.degrees(snapshot.target.angle):.0f}deg,'
            f'clear={_fmt_cm(snapshot.target.clearance)},'
            f'width={snapshot.target.width})'
            if snapshot.target is not None
            else 'gap=none'
        )
        turn = 'left' if self._turn_direction > 0 else 'right'
        dodge = 'left' if self._dodge_direction > 0 else 'right'

        if self._state in (DRIVE, NO_OBSTACLES):
            self.get_logger().info(
                f'[NAV] state={self._state} nearest={nearest_cm} '
                f'front={front_cm} left={left_cm} right={right_cm} '
                f'reason={reason} {detection_source} {sensor_status} {target}'
            )
            return

        dead_end = 'YES' if snapshot.dead_end else 'no'
        spacing_cm = _fmt_cm(snapshot.front)

        self.get_logger().warn(
            f'[OBSTACLE] state={self._state} nearest={nearest_cm} '
            f'front={front_cm} spacing={spacing_cm} '
            f'left={left_cm} right={right_cm} '
            f'dead_end={dead_end} reason={reason} '
            f'turn={turn} dodge={dodge} attempts={self._rotation_attempts}/'
            f'{self._max_rotation_attempts} {detection_source} '
            f'{sensor_status} {target}'
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
