"""
Project C reactive decision node.

Simple robot-vacuum style policy:
- drive straight while the current sensor snapshot is clear;
- use S2 LIDAR as the primary obstacle check;
- when LIDAR is clear, use OAK-D depth as a secondary double-check;
- stop and confirm depth/dynamic obstacles for a few frames before avoiding;
- in a dead end, keep turning the chosen direction until sensors show a path.

There is no map, no route, and no long-term memory.
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
    depth_double_check: bool
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
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_stamped', True)
        self.declare_parameter('cmd_vel_frame_id', 'base_link')
        self.declare_parameter('state_topic', '/obstacle_avoidance_state')
        self.declare_parameter('control_hz', 20.0)

        self.declare_parameter('max_speed', 0.10)
        self.declare_parameter('min_speed', 0.05)
        self.declare_parameter('max_angular_speed', 0.35)
        self.declare_parameter('drive_max_angular_speed', 0.12)
        self.declare_parameter('gap_kp', 0.65)
        self.declare_parameter('corridor_kp', 0.14)

        self.declare_parameter('obstacle_distance', 0.30)
        self.declare_parameter('clear_distance', 0.40)
        self.declare_parameter('slow_distance', 0.55)
        self.declare_parameter('front_body_offset_m', 0.10)
        self.declare_parameter('dynamic_stop_distance', 0.80)
        self.declare_parameter('dynamic_check_frames', 4)
        self.declare_parameter('dynamic_clear_frames', 2)
        self.declare_parameter('obstacle_stale_sec', 1.0)
        self.declare_parameter('turn_out_sec', 0.90)
        self.declare_parameter('side_balance_distance', 0.80)
        self.declare_parameter('side_protect_distance', 0.30)
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
        self._slow_distance = float(self.get_parameter('slow_distance').value)
        self._front_body_offset_m = max(
            0.0,
            float(self.get_parameter('front_body_offset_m').value),
        )
        self._dynamic_stop_distance = float(
            self.get_parameter('dynamic_stop_distance').value
        )
        self._dynamic_check_frames = max(
            1,
            int(self.get_parameter('dynamic_check_frames').value),
        )
        self._dynamic_clear_frames = max(
            1,
            int(self.get_parameter('dynamic_clear_frames').value),
        )
        self._obstacle_stale_sec = float(self.get_parameter('obstacle_stale_sec').value)
        self._turn_out_sec = float(self.get_parameter('turn_out_sec').value)
        self._side_balance_distance = float(
            self.get_parameter('side_balance_distance').value
        )
        self._side_protect_distance = float(
            self.get_parameter('side_protect_distance').value
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
        self._dynamic_seen_frames = 0
        self._dynamic_clear_seen_frames = 0
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
            self._reset_dynamic_check()
            self._log_decision('tof_emergency_stop', snapshot)
            self._publish_velocity(twist)
            return

        if self._state == DYNAMIC_AVOID:
            if self._handle_dynamic_check(twist, snapshot, now):
                self._log_decision('dynamic_check', snapshot)
                self._publish_velocity(twist)
                return

        if self._state == TURN_OUT:
            if self._handle_deadend_turn(twist, snapshot, now):
                self._log_decision('deadend_turn', snapshot)
                self._publish_velocity(twist)
                return

        if snapshot.dynamic_candidate and snapshot.front < self._dynamic_stop_distance:
            self._transition(DYNAMIC_AVOID)
            self._dynamic_seen_frames = 1
            self._dynamic_clear_seen_frames = 0
            self._log_decision('dynamic_candidate_stop', snapshot)
            self._publish_velocity(twist)
            return

        if snapshot.dead_end:
            self._start_deadend_turn(snapshot, now)
            self._log_decision('deadend_start_turn', snapshot)
            self._publish_velocity(twist)
            return

        if snapshot.emergency or snapshot.static_obstacle or snapshot.side_escape is not None:
            self._transition(AVOID)
            self._reset_dynamic_check()
            self._drive_avoid(twist, snapshot, now)
            self._log_decision('avoid_static', snapshot)
            self._publish_velocity(twist)
            return

        self._transition(DRIVE)
        self._reset_dynamic_check()
        self._drive_straight(twist, snapshot)
        self._log_decision('drive_straight', snapshot)
        self._publish_velocity(twist)

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
        lidar_clear = (
            not lidar_available
            or not lidar_obstacle
            and lidar_front >= self._obstacle_distance
        )
        depth_obstacle = depth_available and (
            'depth' in source
            or depth_front < self._obstacle_distance
        )
        depth_double_check = bool(lidar_clear and depth_obstacle)

        static_obstacle = lidar_obstacle
        dynamic_candidate = bool(
            fused.get('dynamic_obstacle', False)
            or depth_double_check
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
            depth_double_check=depth_double_check,
            static_obstacle=static_obstacle,
            dynamic_candidate=dynamic_candidate,
            emergency=emergency,
            tof_emergency=tof_emergency,
            dead_end=dead_end,
            side_escape=side_escape,
        )

    def _handle_dynamic_check(
            self,
            twist: Twist,
            snapshot: Snapshot,
            now: float) -> bool:
        del twist
        if snapshot.dynamic_candidate:
            self._dynamic_seen_frames += 1
            self._dynamic_clear_seen_frames = 0
        else:
            self._dynamic_clear_seen_frames += 1

        if self._dynamic_clear_seen_frames >= self._dynamic_clear_frames:
            self._transition(DRIVE)
            self._reset_dynamic_check()
            return False

        if self._dynamic_seen_frames >= self._dynamic_check_frames:
            self._reset_dynamic_check()
            self._set_turn_direction(snapshot, now, force=True)
            self._transition(AVOID)
            self._drive_avoid(twist, snapshot, now)
            return True

        return True

    def _handle_deadend_turn(
            self,
            twist: Twist,
            snapshot: Snapshot,
            now: float) -> bool:
        if snapshot.side_escape is not None:
            self._turn_direction = snapshot.side_escape

        path_clear = (
            snapshot.front >= self._clear_distance
            and snapshot.side_escape is None
        )
        if path_clear and now >= self._state_end_time:
            self._transition(DRIVE)
            return False

        if now >= self._state_end_time:
            self._state_end_time = now + self._turn_out_sec

        twist.angular.z = self._turn_direction * self._max_angular_speed
        return True

    def _start_deadend_turn(self, snapshot: Snapshot, now: float):
        self._reset_dynamic_check()
        self._set_turn_direction(snapshot, now, force=True)
        self._transition(TURN_OUT, self._turn_out_sec)

    def _drive_avoid(self, twist: Twist, snapshot: Snapshot, now: float):
        direction = self._set_turn_direction(snapshot, now)
        twist.linear.x = 0.0

        if snapshot.side_escape is not None:
            twist.angular.z = snapshot.side_escape * self._max_angular_speed
            return

        if snapshot.target is not None and snapshot.front >= self._clear_distance:
            twist.angular.z = _clamp(
                self._gap_kp * snapshot.target.angle,
                -self._max_angular_speed,
                self._max_angular_speed,
            )
            return

        twist.angular.z = direction * self._max_angular_speed

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

    def _set_turn_direction(
            self,
            snapshot: Snapshot,
            now: float,
            force: bool = False) -> float:
        if not force and now < self._turn_direction_until:
            return self._turn_direction

        if snapshot.side_escape is not None:
            self._turn_direction = snapshot.side_escape
        elif (
                snapshot.target is not None
                and math.isfinite(snapshot.target.angle)
                and abs(snapshot.target.angle) > 0.12
                and snapshot.front >= self._obstacle_distance):
            # Only steer toward gap when front is not actively blocked;
            # gap angle is too noisy close-up and causes oscillation.
            self._turn_direction = 1.0 if snapshot.target.angle > 0.0 else -1.0
        else:
            self._turn_direction = self._clearer_side(snapshot.left, snapshot.right)

        self._turn_direction_until = now + self._turn_direction_hold_sec
        return self._turn_direction

    def _reset_dynamic_check(self):
        self._dynamic_seen_frames = 0
        self._dynamic_clear_seen_frames = 0

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

        if self._state in (DRIVE, NO_OBSTACLES):
            return

        obs_type = 'dynamic' if snapshot.dynamic_candidate else 'static'
        dead_end = 'YES' if snapshot.dead_end else 'no'
        spacing_cm = _fmt_cm(snapshot.front)
        raw_front_cm = _fmt_cm(snapshot.front_raw)
        left_cm = _fmt_cm(snapshot.left)
        right_cm = _fmt_cm(snapshot.right)

        self.get_logger().warn(
            f'[OBSTACLE] state={self._state} type={obs_type} '
            f'spacing={spacing_cm} raw_front={raw_front_cm} '
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


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
