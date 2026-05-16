"""
Project C - refined reactive obstacle avoidance for ROSbot 3 PRO.

Behaviour goal:
- Default: drive straight. No corridor centering, no gap following.
- If front is suspicious: stop and OBSERVE for a few frames.
- If the object disappears / depth confirms clear: drive straight again.
- If the object remains: dodge gently toward the clearer side.
- If it is too close or a true dead-end: backup, then rotate to recover.

States: DRIVE | OBSERVE | DODGE | SIDE_ESCAPE | EDGE_ESCAPE | BACKUP | ROTATE | STOPPED
"""

import json
import math
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import String


DRIVE = 'DRIVE'
OBSERVE = 'OBSERVE'
DODGE = 'DODGE'
BACKUP = 'BACKUP'
ROTATE = 'ROTATE'
STOPPED = 'STOPPED'
SIDE_ESCAPE = 'SIDE_ESCAPE'
EDGE_ESCAPE = 'EDGE_ESCAPE'


def _finite(value) -> float:
    try:
        value = float(value)
        return value if math.isfinite(value) else math.inf
    except (TypeError, ValueError):
        return math.inf


def _cm(value: float) -> str:
    return f'{value * 100:.0f}cm' if math.isfinite(value) else 'inf'


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def _split_topics(value) -> list[str]:
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return [str(item).strip() for item in value if str(item).strip()]
    return [part.strip() for part in str(value).split(',') if part.strip()]


def _angle_delta(value: float, reference: float) -> float:
    return math.atan2(math.sin(value - reference), math.cos(value - reference))


@dataclass
class Snap:
    front_lidar: float
    front_depth: float
    front_oak_low: float
    front_oak_low_count: int
    front_oak_sample_count: int
    front_oak_fallback_count: int
    front_tof: float
    left: float
    right: float
    rear: float
    dynamic: bool
    emergency: bool
    tof_emergency: bool
    lidar_ok: bool
    depth_ok: bool


class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Topics
        self.declare_parameter('obstacle_topic', '/obstacle_representation')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('battery_topic', '/battery')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('front_tof_topics', '/range/fl,/range/fr')
        self.declare_parameter('state_topic', '/obstacle_avoidance_state')
        self.declare_parameter('cmd_vel_stamped', True)
        self.declare_parameter('cmd_vel_frame_id', 'base_link')
        self.declare_parameter('control_hz', 20.0)

        # Motion
        self.declare_parameter('max_speed', 0.22)
        self.declare_parameter('observe_speed', 0.0)
        self.declare_parameter('dodge_forward_speed', 0.045)
        self.declare_parameter('dodge_angular_speed', 0.55)
        self.declare_parameter('rotation_angular_speed', 0.60)
        self.declare_parameter('backup_speed', 0.08)

        # Distances, metres
        self.declare_parameter('clear_distance', 0.45)
        self.declare_parameter('stop_distance', 0.25)
        self.declare_parameter('hard_backup_distance', 0.10)
        self.declare_parameter('front_tof_obstacle_distance', 0.12)
        self.declare_parameter('front_tof_hard_distance', 0.12)
        self.declare_parameter('dodge_clearance', 0.12)
        self.declare_parameter('rear_stop_distance', 0.20)
        self.declare_parameter('side_guard_distance', 0.08)
        self.declare_parameter('side_escape_distance', 0.12)
        self.declare_parameter('side_escape_angular_speed', 0.32)
        self.declare_parameter('side_escape_sec', 0.45)
        self.declare_parameter('edge_escape_enabled', True)
        self.declare_parameter('edge_escape_front_distance', 0.25)
        self.declare_parameter('edge_escape_clearance', 0.30)
        self.declare_parameter('edge_escape_angular_speed', 0.45)
        self.declare_parameter('edge_escape_sec', 1.00)
        self.declare_parameter('edge_escape_max_attempts', 3)
        self.declare_parameter('corner_backup_side_distance', 0.05)
        self.declare_parameter('corner_backup_front_distance', 0.35)
        self.declare_parameter('corner_backup_both_sides_distance', 0.10)
        self.declare_parameter('dynamic_observe_distance', 0.80)

        # Timings / behaviour limits
        self.declare_parameter('observe_frames', 8)
        self.declare_parameter('clear_observe_frames', 3)
        self.declare_parameter('front_release_distance', 0.45)
        self.declare_parameter('front_clear_exit_frames', 5)
        self.declare_parameter('backup_sec', 1.60)
        self.declare_parameter('dodge_step_deg', 60.0)
        self.declare_parameter('dodge_pivot_sec', 0.60)
        self.declare_parameter('rotation_step_deg', 95.0)
        self.declare_parameter('rotation_commit_sec', 0.65)
        self.declare_parameter('max_rotation_attempts', 3)
        self.declare_parameter('clearer_side_deadband', 0.05)
        self.declare_parameter('avoid_turn_direction', -1.0)

        # Static approach + dynamic timeout
        self.declare_parameter('creep_speed', 0.040)
        self.declare_parameter('dynamic_timeout_sec', 5.0)
        self.declare_parameter('dynamic_close_distance', 0.20)
        self.declare_parameter('surprise_backup_enabled', True)
        self.declare_parameter('surprise_backup_distance', 0.20)
        self.declare_parameter('surprise_backup_sec', 0.45)
        self.declare_parameter('surprise_backup_cooldown_sec', 1.20)

        # Battery / debug
        self.declare_parameter('require_battery_ok', False)
        self.declare_parameter('min_battery_voltage', 8.5)
        self.declare_parameter('warn_battery_voltage', 9.0)
        self.declare_parameter('battery_stale_sec', 3.0)
        self.declare_parameter('contact_recovery_enabled', True)
        self.declare_parameter('contact_cmd_speed_min', 0.025)
        self.declare_parameter('contact_odom_speed_max', 0.012)
        self.declare_parameter('contact_odom_angular_max', 0.08)
        self.declare_parameter('contact_stall_sec', 1.50)
        self.declare_parameter('contact_recovery_cooldown_sec', 1.8)
        self.declare_parameter('contact_odom_stale_sec', 0.5)
        self.declare_parameter('tilt_recovery_enabled', True)
        self.declare_parameter('tilt_backup_deg', 8.0)
        self.declare_parameter('tilt_stop_deg', 18.0)
        self.declare_parameter('tilt_stop_pause_sec', 0.30)
        self.declare_parameter('tilt_imu_stale_sec', 0.6)
        self.declare_parameter('debug_decisions', True)
        self.declare_parameter('debug_period_sec', 0.7)

        p = self.get_parameter
        obstacle_topic = p('obstacle_topic').value
        cmd_vel_topic = p('cmd_vel_topic').value
        battery_topic = p('battery_topic').value
        odom_topic = p('odom_topic').value
        imu_topic = p('imu_topic').value
        self._front_tof_topics = _split_topics(p('front_tof_topics').value)
        state_topic = p('state_topic').value
        hz = float(p('control_hz').value)

        self._stamped = _as_bool(p('cmd_vel_stamped').value)
        self._frame = str(p('cmd_vel_frame_id').value)

        self._max_speed = float(p('max_speed').value)
        self._observe_speed = float(p('observe_speed').value)
        self._dodge_forward = float(p('dodge_forward_speed').value)
        self._dodge_ang = float(p('dodge_angular_speed').value)
        self._rot_ang = float(p('rotation_angular_speed').value)
        self._backup_speed = float(p('backup_speed').value)

        self._clear = float(p('clear_distance').value)
        self._stop = float(p('stop_distance').value)
        self._hard_backup = max(
            0.0,
            min(float(p('hard_backup_distance').value), self._stop * 0.95),
        )
        self._front_tof_obstacle = max(0.0, float(p('front_tof_obstacle_distance').value))
        self._front_tof_hard = max(
            0.0,
            min(float(p('front_tof_hard_distance').value), self._front_tof_obstacle),
        )
        self._dodge_clear = float(p('dodge_clearance').value)
        self._rear_stop = float(p('rear_stop_distance').value)
        self._side_guard = float(p('side_guard_distance').value)
        self._side_escape = float(p('side_escape_distance').value)
        self._side_escape_ang = float(p('side_escape_angular_speed').value)
        self._side_escape_sec = float(p('side_escape_sec').value)
        self._edge_escape_enabled = _as_bool(p('edge_escape_enabled').value)
        self._edge_escape_front = max(0.0, float(p('edge_escape_front_distance').value))
        self._edge_escape_clear = max(0.0, float(p('edge_escape_clearance').value))
        self._edge_escape_ang = abs(float(p('edge_escape_angular_speed').value))
        self._edge_escape_sec = max(0.0, float(p('edge_escape_sec').value))
        self._edge_escape_max_attempts = max(1, int(p('edge_escape_max_attempts').value))
        self._corner_backup_side = max(0.0, float(p('corner_backup_side_distance').value))
        self._corner_backup_front = max(0.0, float(p('corner_backup_front_distance').value))
        self._corner_backup_both_sides = max(
            0.0,
            float(p('corner_backup_both_sides_distance').value),
        )
        self._dynamic_observe = float(p('dynamic_observe_distance').value)

        self._observe_frames = max(1, int(p('observe_frames').value))
        self._clear_observe_frames = max(1, int(p('clear_observe_frames').value))
        self._front_release = max(self._clear, float(p('front_release_distance').value))
        self._front_clear_exit_frames = max(1, int(p('front_clear_exit_frames').value))
        self._backup_sec = float(p('backup_sec').value)
        self._dodge_pivot_sec = max(0.0, float(p('dodge_pivot_sec').value))
        self._dodge_sec = math.radians(float(p('dodge_step_deg').value)) / max(abs(self._dodge_ang), 0.01)
        self._rotate_sec = math.radians(float(p('rotation_step_deg').value)) / max(abs(self._rot_ang), 0.01)
        self._rotate_commit_sec = max(0.0, float(p('rotation_commit_sec').value))
        self._max_rotations = max(1, int(p('max_rotation_attempts').value))
        self._clearer_side_deadband = max(0.0, float(p('clearer_side_deadband').value))
        fallback_turn = float(p('avoid_turn_direction').value)
        self._fallback_turn_dir = 1.0 if fallback_turn >= 0.0 else -1.0

        self._require_battery = _as_bool(p('require_battery_ok').value)
        self._min_battery = float(p('min_battery_voltage').value)
        self._warn_battery = float(p('warn_battery_voltage').value)
        self._battery_stale = float(p('battery_stale_sec').value)
        self._contact_recovery_enabled = _as_bool(p('contact_recovery_enabled').value)
        self._contact_cmd_speed_min = max(0.0, float(p('contact_cmd_speed_min').value))
        self._contact_odom_speed_max = max(0.0, float(p('contact_odom_speed_max').value))
        self._contact_odom_angular_max = max(0.0, float(p('contact_odom_angular_max').value))
        self._contact_stall_sec = max(0.0, float(p('contact_stall_sec').value))
        self._contact_cooldown_sec = max(0.0, float(p('contact_recovery_cooldown_sec').value))
        self._contact_odom_stale_sec = max(0.05, float(p('contact_odom_stale_sec').value))
        self._tilt_recovery_enabled = _as_bool(p('tilt_recovery_enabled').value)
        self._tilt_backup_rad = math.radians(max(0.0, float(p('tilt_backup_deg').value)))
        self._tilt_stop_rad = math.radians(max(0.0, float(p('tilt_stop_deg').value)))
        self._tilt_stop_pause = max(0.0, float(p('tilt_stop_pause_sec').value))
        self._tilt_imu_stale = max(0.05, float(p('tilt_imu_stale_sec').value))
        self._debug = _as_bool(p('debug_decisions').value)
        self._debug_period = float(p('debug_period_sec').value)
        self._creep_speed = float(p('creep_speed').value)
        self._dynamic_timeout = float(p('dynamic_timeout_sec').value)
        self._dynamic_close = float(p('dynamic_close_distance').value)
        self._surprise_backup_enabled = _as_bool(p('surprise_backup_enabled').value)
        self._surprise_backup_distance = max(0.0, float(p('surprise_backup_distance').value))
        self._surprise_backup_sec = max(0.0, float(p('surprise_backup_sec').value))
        self._surprise_backup_cooldown = max(
            0.0,
            float(p('surprise_backup_cooldown_sec').value),
        )

        self._state = STOPPED
        self._state_start = 0.0
        self._state_end = 0.0
        self._turn_dir = 1.0
        self._observe_count = 0
        self._front_clear_count = 0
        self._rotation_count = 0
        self._edge_escape_count = 0
        self._raw_obs = None
        self._raw_obs_time = None
        self._battery_v = None
        self._battery_time = None
        self._last_battery_log = 0.0
        self._last_debug_log = 0.0
        self._last_transition = None
        self._dynamic_first_seen: float | None = None  # monotonic time when dynamic first detected
        self._static_confirmed = False                 # True after observe_frames with obstacle
        self._backup_then_observe = False
        self._last_surprise_backup = -math.inf
        self._odom_linear = 0.0
        self._odom_angular = 0.0
        self._odom_time = None
        self._tilt_rad = 0.0
        self._imu_baseline: tuple[float, float] | None = None
        self._imu_time = None
        self._tilt_pause_until = 0.0
        self._tilt_backup_pending = False
        self._tilt_recovery_active = False
        self._motion_cmd_since = None
        self._contact_pending = False
        self._contact_cooldown_until = 0.0

        vel_type = TwistStamped if self._stamped else Twist
        self.create_subscription(String, obstacle_topic, self._on_obstacle, 10)
        self.create_subscription(BatteryState, battery_topic, self._on_battery, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(Imu, imu_topic, self._on_imu, 10)
        self._cmd_pub = self.create_publisher(vel_type, cmd_vel_topic, 10)
        self._state_pub = self.create_publisher(String, state_topic, 10)
        self.create_timer(1.0 / max(hz, 1.0), self._loop)

        self._print_detection_summary()

    # ---------------------------------------------------------------------
    # Startup detection summary
    # ---------------------------------------------------------------------

    def _print_detection_summary(self):
        sep = '=' * 56
        lines = [
            sep,
            'PROJECT C SAFETY NAV - ACTIVE SETTINGS',
            sep,
            f'front observe <= {_cm(self._clear)} | dodge <= {_cm(self._stop)} | hard backup <= {_cm(self._hard_backup)}',
            f'ToF emergency <= {_cm(self._front_tof_hard)} | side scrape <= {_cm(self._side_guard)} | rear stop <= {_cm(self._rear_stop)}',
            f'recovery: backup {self._backup_sec:.2f}s, rotate commit {self._rotate_commit_sec:.2f}s',
            'log: state/reason | obstacle + 4-side distances | OAK low-view',
            sep,
        ]
        for line in lines:
            self.get_logger().info(line)
        return

        sep = '=' * 60
        lines = [
            sep,
            'OBSTACLE AVOIDANCE - DETECTION POLICY SUMMARY',
            sep,
            f'  [THRESHOLDS]',
            f'    hard_backup        : {_cm(self._hard_backup)}  emergency backup trigger',
            f'    clear_distance     : {_cm(self._clear)}  ← LIDAR/depth suspicious zone',
            f'    stop_distance      : {_cm(self._stop)}  static obstacle dodge trigger',
            f'    dodge_clearance    : {_cm(self._dodge_clear)}  dead-end side clearance',
            f'    front_tof          : emergency bumper at {_cm(self._front_tof_hard)}',
            f'    front_release      : {_cm(self._front_release)} after {self._front_clear_exit_frames} clear frames',
            f'    edge_escape        : front<{_cm(self._edge_escape_front)} with side>{_cm(self._edge_escape_clear)}',
            f'    side_guard         : {_cm(self._side_guard)}  ← side scrape emergency',
            f'    side_escape        : {_cm(self._side_escape)}  ← side escape exit threshold',
            f'    rear_stop          : {_cm(self._rear_stop)}  ← blocks backup if rear blocked',
            f'    dynamic_observe    : {_cm(self._dynamic_observe)}  ← distance to trigger dynamic observe',
            f'    dynamic_close      : {_cm(self._dynamic_close)}  ← sudden dynamic appearance threshold',
            sep,
            f'  [BEHAVIOUR PHASES]',
            f'    Phase 1 – OBSERVE  : Stop (speed={self._observe_speed:.3f} m/s) for {self._observe_frames} frames.',
            f'                         If object clears → DRIVE straight.',
            f'                         Depth-confirms-clear after {self._clear_observe_frames} frames → DRIVE.',
            f'    Phase 2A – DYNAMIC : If dynamic detected hold 0 m/s up to {self._dynamic_timeout:.1f}s.',
            f'                         Object gone → DRIVE. Timed-out → treat as static.',
            f'    Phase 2B – STATIC  : Creep at {self._creep_speed:.3f} m/s toward obstacle.',
            f'                         Only dodge when front ≤ {_cm(self._stop)} (stop_distance).',
            f'    DODGE              : Pivot {self._dodge_pivot_sec:.1f}s, then arc forward {self._dodge_forward:.3f} m/s',
            f'                         angular {self._dodge_ang:.2f} rad/s, step {math.degrees(self._dodge_ang * self._dodge_sec):.0f}° max.',
            f'    EDGE_ESCAPE        : Pivot toward open side for {self._edge_escape_sec:.2f}s before backing up.',
            f'    BACKUP             : Reverse {self._backup_speed:.3f} m/s for {self._backup_sec:.2f}s then ROTATE.',
            f'    CONTACT RECOVERY   : cmd>{self._contact_cmd_speed_min:.3f} m/s but odom<{self._contact_odom_speed_max:.3f} m/s',
            f'                         for {self._contact_stall_sec:.2f}s -> BACKUP + ROTATE.',
            f'    TILT RECOVERY      : IMU tilt>{math.degrees(self._tilt_backup_rad):.0f}deg -> BACKUP, >{math.degrees(self._tilt_stop_rad):.0f}deg -> STOP {self._tilt_stop_pause:.2f}s then BACKUP.',
            f'    ROTATE             : commit {self._rotate_commit_sec:.2f}s, then check clear; {math.degrees(self._rot_ang * self._rotate_sec):.0f}° max step.',
            sep,
            f'  [PRIORITY ORDER in each loop tick]',
            f'    0. Front ToF emergency   → BACKUP',
            f'    1. Front <{_cm(self._hard_backup)}            → BACKUP immediately',
            f'    2. Dead-end              → BACKUP + ROTATE',
            f'    3. Front ≤{_cm(self._clear)}           → OBSERVE (stop & watch)',
            f'    4. Side scrape <{_cm(self._side_guard)} → SIDE_ESCAPE (if front clear)',
            f'    5. Default               → DRIVE straight, no corridor centering',
            sep,
            f'  [LOG LEGEND]',
            f'    [NAV] state=DRIVE reason=drive_straight       → going straight, all clear',
            f'    [NAV] state=OBSERVE reason=observe_start      → new obstacle, watching',
            f'    [NAV] state=OBSERVE reason=dynamic_wait       → dynamic object, waiting ≤{self._dynamic_timeout:.0f}s',
            f'    [NAV] state=OBSERVE reason=observe            → still counting frames',
            f'    [NAV] state=OBSERVE reason=observe (creep)    → static confirmed, creeping to {_cm(self._stop)}',
            f'    [NAV] state=DODGE   reason=dodge              → dodging to clearer side',
            f'    [NAV] state=BACKUP  reason=too_close_backup   → emergency backup',
            f'    [NAV] state=ROTATE  reason=rotate             → scanning for open heading',
            sep,
        ]
        for line in lines:
            self.get_logger().info(line)

    # ---------------------------------------------------------------------
    # Subscribers
    # ---------------------------------------------------------------------

    def _on_obstacle(self, msg: String):
        try:
            self._raw_obs = json.loads(msg.data)
            self._raw_obs_time = time.monotonic()
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid obstacle JSON ignored')

    def _on_battery(self, msg: BatteryState):
        if math.isfinite(msg.voltage) and msg.voltage > 0.0:
            self._battery_v = float(msg.voltage)
            self._battery_time = time.monotonic()

    def _on_odom(self, msg: Odometry):
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        wz = float(msg.twist.twist.angular.z)
        if math.isfinite(vx) and math.isfinite(vy):
            self._odom_linear = math.hypot(vx, vy)
        if math.isfinite(wz):
            self._odom_angular = abs(wz)
        self._odom_time = time.monotonic()

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        x = float(q.x)
        y = float(q.y)
        z = float(q.z)
        w = float(q.w)
        if not all(math.isfinite(v) for v in (x, y, z, w)):
            return

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        if self._imu_baseline is None:
            self._imu_baseline = (roll, pitch)

        base_roll, base_pitch = self._imu_baseline
        self._tilt_rad = max(
            abs(_angle_delta(roll, base_roll)),
            abs(_angle_delta(pitch, base_pitch)),
        )
        self._imu_time = time.monotonic()

    # ---------------------------------------------------------------------
    # Main FSM
    # ---------------------------------------------------------------------

    def _loop(self):
        now = time.monotonic()
        twist = Twist()

        if self._battery_blocked(now):
            self._set_state(STOPPED)
            self._log('battery_blocked')
            self._publish_cmd(twist)
            return
        self._battery_warn(now)

        if self._raw_obs is None or self._raw_obs_time is None or now - self._raw_obs_time > 1.0:
            self._set_state(STOPPED)
            self._log('no_obstacle_data')
            self._publish_cmd(twist)
            return

        snap = self._snap()
        front = self._effective_front(snap)

        if self._handle_tilt_recovery(twist, snap, now):
            self._publish_cmd(twist)
            return

        if self._tilt_backup(now) and self._state not in (BACKUP, STOPPED):
            self._start_backup()
            self._handle_backup(twist, snap, now)
            self._log('tilt_backup', snap)
            self._publish_cmd(twist)
            return

        # Track when dynamic obstacle was first seen (reset when gone).
        if snap.dynamic:
            if self._dynamic_first_seen is None:
                self._dynamic_first_seen = now
        else:
            self._dynamic_first_seen = None

        front_tof_hard = (
            snap.tof_emergency
            and math.isfinite(snap.front_tof)
            and snap.front_tof <= self._front_tof_hard
        )
        if front_tof_hard and self._state not in (BACKUP, ROTATE):
            self._start_backup()
            self._handle_backup(twist, snap, now)
            self._log('tof_emergency_backup', snap)
            self._publish_cmd(twist)
            return

        if self._contact_pending and self._state not in (BACKUP, ROTATE, STOPPED):
            self._contact_pending = False
            self._motion_cmd_since = None
            self._start_backup()
            self._handle_backup(twist, snap, now)
            self._log('contact_stall_backup', snap)
            self._publish_cmd(twist)
            return

        # Continue active manoeuvres first.
        if self._state == OBSERVE and self._handle_observe(twist, snap, front):
            self._log('observe', snap)
            self._publish_cmd(twist)
            return

        if self._state == DODGE and self._handle_dodge(twist, snap, front, now):
            self._log('dodge', snap)
            self._publish_cmd(twist)
            return

        if self._state == SIDE_ESCAPE and self._handle_side_escape(twist, snap, now):
            self._log('side_escape', snap)
            self._publish_cmd(twist)
            return

        if self._state == EDGE_ESCAPE and self._handle_edge_escape(twist, snap, now):
            self._log('edge_escape', snap)
            self._publish_cmd(twist)
            return

        if self._state == BACKUP and self._handle_backup(twist, snap, now):
            self._log('backup', snap)
            self._publish_cmd(twist)
            return

        if self._state == ROTATE and self._handle_rotate(twist, snap, front, now):
            self._log('rotate', snap)
            self._publish_cmd(twist)
            return

        # Priority 1: corner pinch. A side scrape plus a close front object
        # should back out before pivoting, otherwise the robot can swing into
        # table legs / wall corners.
        if self._corner_backup_needed(snap, front):
            self._start_backup()
            self._handle_backup(twist, snap, now)
            self._log('corner_backup', snap)
            self._publish_cmd(twist)
            return

        # Priority 2: sudden close obstacle. Give a short reverse pulse before
        # choosing a longer recovery path.
        if self._surprise_backup_needed(snap, front, now):
            self._start_backup(self._surprise_backup_sec, then_observe=True)
            self._last_surprise_backup = now
            self._handle_backup(twist, snap, now)
            self._log('surprise_backup', snap)
            self._publish_cmd(twist)
            return

        # Priority 3: near edge snag. If one side is clearly open, pivot out
        # before falling back to backup. This handles wall lips / hanging cloth.
        if self._edge_escape_needed(snap, front):
            self._start_edge_escape(snap)
            self._handle_edge_escape(twist, snap, now)
            self._log('edge_escape_start', snap)
            self._publish_cmd(twist)
            return

        # Priority 4: too close = backup, not dodge.
        if self._too_close(snap):
            self._start_backup()
            self._handle_backup(twist, snap, now)
            self._log('too_close_backup', snap)
            self._publish_cmd(twist)
            return

        # Priority 5: true dead-end = backup then rotate.
        if self._dead_end(snap, front):
            self._start_backup()
            self._handle_backup(twist, snap, now)
            self._log('dead_end_backup', snap)
            self._publish_cmd(twist)
            return

        # Priority 6: suspicious front object = observe first.
        if self._front_suspicious(snap, front):
            self._start_observe()
            self._handle_observe(twist, snap, front)
            self._log('observe_start', snap)
            self._publish_cmd(twist)
            return

        # Priority 7: side collision guard. This is not corridor centering;
        # it only prevents scraping/hitting when the forward path is clear.
        if self._side_danger(snap):
            self._start_side_escape(snap)
            self._handle_side_escape(twist, snap, now)
            self._log('side_guard_escape', snap)
            self._publish_cmd(twist)
            return

        # Default: straight drive. Side readings do not steer the robot.
        self._set_state(DRIVE)
        twist.linear.x = self._max_speed
        twist.angular.z = 0.0
        self._log('drive_straight', snap)
        self._publish_cmd(twist)

    # ---------------------------------------------------------------------
    # State handlers
    # ---------------------------------------------------------------------

    def _start_observe(self):
        if self._state != OBSERVE:
            self._observe_count = 0
            self._front_clear_count = 0
            self._static_confirmed = False
            self._set_state(OBSERVE)

    def _handle_observe(self, twist: Twist, snap: Snap, front: float) -> bool:
        self._observe_count += 1
        now = time.monotonic()

        if self._too_close(snap):
            self._static_confirmed = False
            self._start_backup()
            return self._handle_backup(twist, snap, now)

        if self._surprise_backup_needed(snap, front, now):
            self._static_confirmed = False
            self._start_backup(self._surprise_backup_sec, then_observe=True)
            self._last_surprise_backup = now
            return self._handle_backup(twist, snap, now)

        if self._corner_backup_needed(snap, front):
            self._static_confirmed = False
            self._start_backup()
            return self._handle_backup(twist, snap, now)

        if self._edge_escape_needed(snap, front):
            self._static_confirmed = False
            self._start_edge_escape(snap)
            return self._handle_edge_escape(twist, snap, now)

        # Thin/low objects can disappear for one LIDAR frame; require a stable
        # release distance before driving straight again.
        if not self._front_suspicious(snap, front):
            soft_clear = (not math.isfinite(front)) or front >= self._clear
            hard_clear = (not math.isfinite(front)) or front >= self._front_release
            if soft_clear:
                self._front_clear_count += 1
            else:
                self._front_clear_count = 0

            # Hard clear exits quickly; soft clear exits after the same stable
            # frame count. Without this, a steady 36-44 cm reading can park the
            # robot in OBSERVE forever.
            if hard_clear or self._front_clear_count >= self._front_clear_exit_frames:
                self._static_confirmed = False
                self._set_state(DRIVE)
                return False

            twist.linear.x = 0.0
            twist.angular.z = 0.0
            return True

        self._front_clear_count = 0

        # LIDAR suspicious but depth clearly open → short observe then drive.
        if self._depth_confirms_clear(snap) and self._observe_count >= self._clear_observe_frames:
            self._static_confirmed = False
            self._set_state(DRIVE)
            return False

        # Phase 1: still collecting observation frames → stop and watch.
        if self._observe_count < self._observe_frames:
            twist.linear.x = self._observe_speed
            twist.angular.z = 0.0
            return True

        # Phase 2: obstacle confirmed after observe_frames.
        self._static_confirmed = True

        # Dynamic object handling: hold position and wait up to dynamic_timeout_sec.
        # If the dynamic flag clears (person moved away), the top check exits first.
        if snap.dynamic and self._dynamic_first_seen is not None:
            elapsed = now - self._dynamic_first_seen
            if front > self._stop and elapsed < self._dynamic_timeout:
                # Dynamic object still near but not dangerously close: hold still.
                self._log('dynamic_wait', snap)
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                return True
            # Dynamic timed out (>5s) OR entered stop zone → treat as static blocker.

        # Static confirmed (or dynamic timed out/too close).
        # Dead-end check first.
        if self._dead_end(snap, front):
            self._static_confirmed = False
            self._start_backup()
            return True

        # If still outside stop_distance, creep forward slowly.
        # Robot approaches to stop_distance, then dodges.
        if front > self._stop:
            twist.linear.x = self._creep_speed
            twist.angular.z = 0.0
            return True

        # At stop_distance: initiate dodge toward the clearer side.
        self._static_confirmed = False
        self._start_dodge(snap)
        return self._handle_dodge(twist, snap, front, now)

    def _start_dodge(self, snap: Snap):
        self._turn_dir = self._clearer_side(snap.left, snap.right)
        self._set_state(DODGE, self._dodge_sec)

    def _handle_dodge(self, twist: Twist, snap: Snap, front: float, now: float) -> bool:
        if self._too_close(snap):
            self._start_backup()
            return self._handle_backup(twist, snap, now)

        if self._surprise_backup_needed(snap, front, now):
            self._start_backup(self._surprise_backup_sec, then_observe=True)
            self._last_surprise_backup = now
            return self._handle_backup(twist, snap, now)

        if self._corner_backup_needed(snap, front):
            self._start_backup()
            return self._handle_backup(twist, snap, now)

        if self._edge_escape_needed(snap, front):
            self._start_edge_escape(snap)
            return self._handle_edge_escape(twist, snap, now)

        elapsed = max(0.0, now - self._state_start)
        if elapsed < self._dodge_pivot_sec:
            # Pure pivot: turn in place. Do NOT exit early — a partial rotation
            # moves the obstacle out of the LIDAR front sector producing a
            # false-clear reading that would cause the robot to drive straight
            # into the obstacle.
            twist.linear.x = 0.0
            twist.angular.z = self._turn_dir * self._dodge_ang
            return True

        if now < self._state_end:
            # Arc phase: forward + angular. Commit to the full arc.
            # Do NOT exit early on a front-clear reading — the obstacle just
            # rotated out of the scan sector; it is still physically there.
            twist.linear.x = self._dodge_forward
            twist.angular.z = self._turn_dir * self._dodge_ang
            return True

        # Full arc completed. Re-enter OBSERVE to verify the new heading
        # before resuming straight drive. If the obstacle is still detectable
        # from the new heading, OBSERVE will initiate another dodge step —
        # the robot keeps arcing until the obstacle is fully cleared.
        self._start_observe()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        return True


    def _start_side_escape(self, snap: Snap):
        # If left is too close, rotate right. If right is too close, rotate left.
        left_close = math.isfinite(snap.left) and snap.left < self._side_guard
        right_close = math.isfinite(snap.right) and snap.right < self._side_guard

        if left_close and not right_close:
            self._turn_dir = -1.0
        elif right_close and not left_close:
            self._turn_dir = 1.0
        else:
            self._turn_dir = self._clearer_side(snap.left, snap.right)

        self._set_state(SIDE_ESCAPE, self._side_escape_sec)

    def _handle_side_escape(self, twist: Twist, snap: Snap, now: float) -> bool:
        # Hard front danger still has priority.
        front = self._effective_front(snap)
        if self._too_close(snap):
            self._start_backup()
            return self._handle_backup(twist, snap, now)

        if self._surprise_backup_needed(snap, front, now):
            self._start_backup(self._surprise_backup_sec, then_observe=True)
            self._last_surprise_backup = now
            return self._handle_backup(twist, snap, now)

        if self._corner_backup_needed(snap, front):
            self._start_backup()
            return self._handle_backup(twist, snap, now)

        if self._edge_escape_needed(snap, front):
            self._start_edge_escape(snap)
            return self._handle_edge_escape(twist, snap, now)

        left_safe = (not math.isfinite(snap.left)) or snap.left >= self._side_escape
        right_safe = (not math.isfinite(snap.right)) or snap.right >= self._side_escape

        if left_safe and right_safe:
            self._set_state(DRIVE)
            return False

        if now < self._state_end:
            twist.linear.x = 0.0
            twist.angular.z = self._turn_dir * self._side_escape_ang
            return True

        # Do not keep spinning forever. Try driving slowly after a small escape turn.
        self._set_state(DRIVE)
        return False

    def _start_edge_escape(self, snap: Snap):
        if self._state != EDGE_ESCAPE:
            self._edge_escape_count += 1
        self._turn_dir = self._clearer_side(snap.left, snap.right)
        self._set_state(EDGE_ESCAPE, self._edge_escape_sec)

    def _handle_edge_escape(self, twist: Twist, snap: Snap, now: float) -> bool:
        front = self._effective_front(snap)
        if self._corner_backup_needed(snap, front):
            self._start_backup()
            return self._handle_backup(twist, snap, now)

        if now < self._state_end:
            twist.linear.x = 0.0
            twist.angular.z = self._turn_dir * self._edge_escape_ang
            return True

        if self._edge_escape_needed(snap, front):
            if self._edge_escape_count >= self._edge_escape_max_attempts:
                self._start_backup()
                return True
            self._start_edge_escape(snap)
            twist.linear.x = 0.0
            twist.angular.z = self._turn_dir * self._edge_escape_ang
            return True

        self._edge_escape_count = 0
        self._start_observe()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        return True

    def _start_backup(self, duration: float | None = None, then_observe: bool = False):
        self._rotation_count = 0
        self._edge_escape_count = 0
        self._motion_cmd_since = None
        self._contact_pending = False
        self._backup_then_observe = then_observe
        self._set_state(BACKUP, self._backup_sec if duration is None else duration)

    def _handle_backup(self, twist: Twist, snap: Snap, now: float) -> bool:
        rear_blocked = math.isfinite(snap.rear) and snap.rear < self._rear_stop
        if now < self._state_end and not rear_blocked:
            twist.linear.x = -abs(self._backup_speed)
            twist.angular.z = 0.0
            return True

        if self._backup_then_observe:
            self._backup_then_observe = False
            self._start_observe()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            return True

        self._start_rotate(snap)
        return True

    def _start_rotate(self, snap: Snap):
        self._turn_dir = self._clearer_side(snap.left, snap.right)
        self._set_state(ROTATE, self._rotate_sec)

    def _handle_rotate(self, twist: Twist, snap: Snap, front: float, now: float) -> bool:
        if self._too_close(snap):
            self._start_backup()
            return self._handle_backup(twist, snap, now)

        if self._corner_backup_needed(snap, front):
            self._start_backup()
            return self._handle_backup(twist, snap, now)

        if self._edge_escape_needed(snap, front):
            self._start_edge_escape(snap)
            return self._handle_edge_escape(twist, snap, now)

        elapsed = max(0.0, now - self._state_start)
        if now < self._state_end and elapsed < self._rotate_commit_sec:
            twist.linear.x = 0.0
            twist.angular.z = self._turn_dir * self._rot_ang
            return True

        if front > self._clear or self._depth_confirms_clear(snap):
            self._set_state(DRIVE)
            return False

        if now < self._state_end:
            twist.linear.x = 0.0
            twist.angular.z = self._turn_dir * self._rot_ang
            return True

        self._rotation_count += 1
        if self._rotation_count >= self._max_rotations:
            self._start_backup()
            return True

        self._state_end = now + self._rotate_sec
        twist.angular.z = self._turn_dir * self._rot_ang
        return True

    # ---------------------------------------------------------------------
    # Decision helpers
    # ---------------------------------------------------------------------

    def _effective_front(self, snap: Snap) -> float:
        """Return the front distance used by the FSM.

        Important fix:
        If LIDAR reports a medium-close value but OAK depth clearly sees an open
        path, do NOT keep returning the LIDAR value forever. That caused the
        OBSERVE -> DRIVE -> OBSERVE loop.
        """
        lidar = snap.front_lidar
        depth = snap.front_depth
        tof = snap.front_tof

        if math.isfinite(tof) and tof <= self._front_tof_hard:
            return min(lidar, depth, tof)

        if snap.depth_ok and math.isfinite(depth):
            # Real close depth obstacle: trust depth.
            if depth <= self._stop:
                return depth

            # LIDAR says medium-close, but depth sees clear path: treat as clear.
            if depth >= self._clear and (not math.isfinite(lidar) or lidar > self._stop):
                return depth

            # Both sensors show something in the front area.
            if math.isfinite(lidar):
                return min(lidar, depth)
            return depth

        return lidar if math.isfinite(lidar) else math.inf

    def _front_suspicious(self, snap: Snap, front: float) -> bool:
        lidar_suspicious = (
            snap.lidar_ok
            and math.isfinite(snap.front_lidar)
            and snap.front_lidar <= self._clear
        )
        depth_suspicious = (
            snap.depth_ok
            and math.isfinite(snap.front_depth)
            and snap.front_depth <= self._clear
        )
        dynamic_suspicious = (
            snap.dynamic
            and math.isfinite(front)
            and front <= self._dynamic_observe
        )
        return lidar_suspicious or depth_suspicious or dynamic_suspicious

    def _depth_confirms_clear(self, snap: Snap) -> bool:
        return (
            snap.depth_ok
            and math.isfinite(snap.front_depth)
            and snap.front_depth >= self._clear
            and not snap.dynamic
        )

    def _too_close(self, snap: Snap) -> bool:
        # Hard safety: backup only when the front is closer than the dodge trigger.
        # At stop_distance a confirmed static obstacle should dodge, not reverse.
        lidar_too_close = (
            math.isfinite(snap.front_lidar)
            and snap.front_lidar <= self._hard_backup
        )
        depth_too_close = (
            math.isfinite(snap.front_depth)
            and snap.front_depth <= self._hard_backup
        )
        tof_too_close = (
            math.isfinite(snap.front_tof)
            and snap.front_tof <= self._front_tof_hard
        )
        return lidar_too_close or depth_too_close or tof_too_close

    def _side_danger(self, snap: Snap) -> bool:
        left_close = math.isfinite(snap.left) and snap.left < self._side_guard
        right_close = math.isfinite(snap.right) and snap.right < self._side_guard
        return left_close or right_close

    def _surprise_backup_needed(self, snap: Snap, front: float, now: float) -> bool:
        if not self._surprise_backup_enabled:
            return False
        if now - self._last_surprise_backup < self._surprise_backup_cooldown:
            return False
        if self._depth_confirms_clear(snap):
            return False
        close_front = math.isfinite(front) and front <= self._surprise_backup_distance
        sudden_dynamic = snap.dynamic and close_front
        sudden_static = self._state in (DRIVE, OBSERVE, DODGE) and close_front
        return sudden_dynamic or sudden_static

    def _corner_backup_needed(self, snap: Snap, front: float) -> bool:
        if self._depth_confirms_clear(snap):
            return False

        left = snap.left if math.isfinite(snap.left) else math.inf
        right = snap.right if math.isfinite(snap.right) else math.inf
        side_min = min(left, right)
        front_pinched = math.isfinite(front) and front <= self._corner_backup_front
        side_pinched = side_min <= self._corner_backup_side
        both_sides_pinched = (
            left <= self._corner_backup_both_sides
            and right <= self._corner_backup_both_sides
        )
        return (front_pinched and side_pinched) or both_sides_pinched

    def _edge_escape_needed(self, snap: Snap, front: float) -> bool:
        if not self._edge_escape_enabled:
            return False
        if self._edge_escape_count >= self._edge_escape_max_attempts:
            return False
        if not math.isfinite(front) or front > self._edge_escape_front:
            return False

        left_clear = snap.left if math.isfinite(snap.left) else math.inf
        right_clear = snap.right if math.isfinite(snap.right) else math.inf
        open_side = max(left_clear, right_clear)
        tight_side = min(left_clear, right_clear)

        if open_side < self._edge_escape_clear:
            return False

        # If the object is already in the hard-backup zone, only pivot when a
        # side is also tight. A centered near obstacle should still back up.
        if front <= self._hard_backup:
            return tight_side <= self._side_escape

        # For the softer 10-15 cm snag zone, allow a strongly asymmetric side
        # reading to pivot out of wall lips / hanging cloth.
        return tight_side <= self._side_escape or (open_side - tight_side) >= self._edge_escape_clear

    def _dead_end(self, snap: Snap, front: float) -> bool:
        front_blocked = front <= self._clear and not self._depth_confirms_clear(snap)
        left_blocked = math.isfinite(snap.left) and snap.left < self._dodge_clear
        right_blocked = math.isfinite(snap.right) and snap.right < self._dodge_clear
        return front_blocked and left_blocked and right_blocked

    def _clearer_side(self, left: float, right: float) -> float:
        left_clear = left if math.isfinite(left) else math.inf
        right_clear = right if math.isfinite(right) else math.inf
        if math.isinf(left_clear) and math.isinf(right_clear):
            return self._fallback_turn_dir
        if abs(left_clear - right_clear) <= self._clearer_side_deadband:
            return self._fallback_turn_dir
        return 1.0 if left_clear > right_clear else -1.0

    def _snap(self) -> Snap:
        data = self._raw_obs or {}
        fused = data.get('fused', {})
        lidar = data.get('lidar', {})
        depth = data.get('depth', {})
        tof = data.get('tof', {})

        source = list(fused.get('source', []))
        lidar_ok = bool(lidar.get('available', False))
        depth_ok = bool(depth.get('available', False))
        emergency = bool(fused.get('emergency', False))
        tof_emergency = bool(emergency and 'tof' in source)

        return Snap(
            front_lidar=_finite(lidar.get('front_control', fused.get('front_distance'))),
            front_depth=_finite(depth.get('front_min')),
            front_oak_low=_finite(depth.get('pointcloud_low_front_min')),
            front_oak_low_count=int(depth.get('pointcloud_low_front_count', 0) or 0),
            front_oak_sample_count=int(depth.get('pointcloud_sample_count', 0) or 0),
            front_oak_fallback_count=int(
                depth.get('pointcloud_low_fallback_count', 0) or 0
            ),
            front_tof=self._front_tof_distance(tof),
            left=_finite(fused.get('left_distance')),
            right=_finite(fused.get('right_distance')),
            rear=_finite(fused.get('rear_distance')),
            dynamic=bool(fused.get('dynamic_obstacle', False) or depth.get('motion', False)),
            emergency=emergency,
            tof_emergency=tof_emergency,
            lidar_ok=lidar_ok,
            depth_ok=depth_ok,
        )

    def _front_tof_distance(self, tof: dict) -> float:
        topics = tof.get('topics', {}) if isinstance(tof, dict) else {}
        if not isinstance(topics, dict):
            return math.inf

        readings = []
        for topic in self._front_tof_topics:
            value = _finite(topics.get(topic))
            if math.isfinite(value):
                readings.append(value)
        return min(readings) if readings else math.inf

    # ---------------------------------------------------------------------
    # ROS publish / logging
    # ---------------------------------------------------------------------

    def _set_state(self, state: str, duration: float = 0.0):
        now = time.monotonic()
        if state == DRIVE:
            self._edge_escape_count = 0
        if self._state != state:
            old = self._state
            self._state = state
            self._state_start = now
            self._last_transition = f'{old}->{state}'
            self.get_logger().info(f'FSM: {old} -> {state}')
            msg = String()
            msg.data = f'{time.time():.3f},{state}'
            self._state_pub.publish(msg)
        self._state_end = now + max(0.0, duration)

    def _track_contact_stall(self, twist: Twist):
        if not self._contact_recovery_enabled:
            return

        now = time.monotonic()
        if self._state in (BACKUP, ROTATE, SIDE_ESCAPE, STOPPED) or now < self._contact_cooldown_until:
            self._motion_cmd_since = None
            return

        if self._odom_time is None or now - self._odom_time > self._contact_odom_stale_sec:
            self._motion_cmd_since = None
            return

        forward_cmd = twist.linear.x >= self._contact_cmd_speed_min
        if not forward_cmd:
            self._motion_cmd_since = None
            return

        odom_stalled = (
            self._odom_linear <= self._contact_odom_speed_max
            and self._odom_angular <= self._contact_odom_angular_max
        )
        if not odom_stalled:
            self._motion_cmd_since = None
            return

        if self._motion_cmd_since is None:
            self._motion_cmd_since = now
            return

        if now - self._motion_cmd_since >= self._contact_stall_sec:
            self._contact_pending = True
            self._contact_cooldown_until = now + self._contact_cooldown_sec
            self._motion_cmd_since = None
            self.get_logger().warn(
                f'[CONTACT] cmd forward but odom is near zero for '
                f'{self._contact_stall_sec:.1f}s; backing up next tick'
            )

    def _imu_recent(self, now: float) -> bool:
        return self._imu_time is not None and now - self._imu_time <= self._tilt_imu_stale

    def _tilt_backup(self, now: float) -> bool:
        return (
            self._tilt_recovery_enabled
            and self._imu_recent(now)
            and self._tilt_rad >= self._tilt_backup_rad
        )

    def _tilt_stop(self, now: float) -> bool:
        return (
            self._tilt_recovery_enabled
            and self._imu_recent(now)
            and self._tilt_rad >= self._tilt_stop_rad
        )

    def _handle_tilt_recovery(self, twist: Twist, snap: Snap, now: float) -> bool:
        if not self._tilt_recovery_enabled or not self._imu_recent(now):
            self._tilt_backup_pending = False
            self._tilt_recovery_active = False
            return False

        severe_tilt = self._tilt_rad >= self._tilt_stop_rad
        if not severe_tilt:
            self._tilt_backup_pending = False
            if self._state not in (BACKUP, ROTATE):
                self._tilt_recovery_active = False
            return False

        if self._state in (BACKUP, ROTATE) and self._tilt_recovery_active:
            return False

        if not self._tilt_backup_pending:
            self._tilt_backup_pending = True
            self._tilt_pause_until = now + self._tilt_stop_pause
            self._set_state(STOPPED)
            self._log('tilt_stop_pause', snap)
            return True

        if now < self._tilt_pause_until:
            self._set_state(STOPPED)
            self._log('tilt_stop_pause', snap)
            return True

        self._tilt_backup_pending = False
        self._tilt_recovery_active = True
        self._start_backup()
        self._handle_backup(twist, snap, now)
        self._log('tilt_stop_backup', snap)
        return True

    def _publish_cmd(self, twist: Twist):
        self._track_contact_stall(twist)

        if not self._stamped:
            self._cmd_pub.publish(twist)
            return

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        msg.twist = twist
        self._cmd_pub.publish(msg)

    def _log(self, reason: str, snap: Snap | None = None):
        if not self._debug:
            return

        now = time.monotonic()
        if self._last_transition is None and now - self._last_debug_log < self._debug_period:
            return
        self._last_debug_log = now
        self._last_transition = None

        if snap is None:
            self.get_logger().info(f'[NAV] state={self._state} reason={reason}')
            return

        front = self._effective_front(snap)
        dyn_elapsed = ''
        if snap.dynamic and self._dynamic_first_seen is not None:
            dyn_elapsed = f' dyn_elapsed={time.monotonic() - self._dynamic_first_seen:.1f}s'
        tilt_info = ''
        if self._imu_recent(now):
            tilt_info = f' tilt={math.degrees(self._tilt_rad):.0f}deg'
        creep_info = (
            f' [CREEP->{_cm(self._stop)}]'
            if (self._state == OBSERVE and self._static_confirmed
                and snap is not None and not snap.dynamic and front > self._stop)
            else ''
        )
        obstacle = (
            front <= self._clear
            or snap.left <= self._side_guard
            or snap.right <= self._side_guard
            or snap.rear <= self._rear_stop
            or snap.dynamic
            or snap.tof_emergency
        )
        oak_low_hit = (
            math.isfinite(snap.front_oak_low)
            and snap.front_oak_low <= self._clear
            and snap.front_oak_low_count > 0
        )
        lidar_front_hit = (
            math.isfinite(snap.front_lidar)
            and snap.front_lidar <= self._clear
        )
        oak_low_status = 'LOW_OBSTACLE' if oak_low_hit else 'clear'
        lidar_miss = 'yes' if oak_low_hit and not lidar_front_hit else 'no'
        line = (
            f'[NAV] state={self._state} reason={reason}{creep_info} | '
            f'obstacle={"YES" if obstacle else "NO"} '
            f'front={_cm(front)} left={_cm(snap.left)} '
            f'right={_cm(snap.right)} rear={_cm(snap.rear)} | '
            f'oak_low={oak_low_status} dist={_cm(snap.front_oak_low)} '
            f'pts={snap.front_oak_low_count} pc={snap.front_oak_sample_count} '
            f'fallback={snap.front_oak_fallback_count} lidar_miss={lidar_miss} | '
            f'dynamic={"YES" if snap.dynamic else "NO"}{dyn_elapsed}{tilt_info} '
            f'turn={"L" if self._turn_dir > 0 else "R"} obs={self._observe_count}'
        )
        if self._state in (OBSERVE, DODGE, SIDE_ESCAPE, EDGE_ESCAPE, BACKUP, ROTATE):
            self.get_logger().warn(line)
        else:
            self.get_logger().info(line)

    # ---------------------------------------------------------------------
    # Battery
    # ---------------------------------------------------------------------

    def _battery_blocked(self, now: float) -> bool:
        if not self._require_battery:
            return False
        if self._battery_v is None or self._battery_time is None:
            return True
        if now - self._battery_time > self._battery_stale:
            return True
        return self._battery_v < self._min_battery

    def _battery_warn(self, now: float):
        if now - self._last_battery_log < 5.0:
            return
        if self._battery_v is None:
            if self._require_battery:
                self._last_battery_log = now
                self.get_logger().error('[BAT] no reading - stopped')
            return
        if self._battery_v < self._min_battery:
            self._last_battery_log = now
            self.get_logger().error(f'[BAT] {self._battery_v:.2f}V < min {self._min_battery:.2f}V')
        elif self._battery_v < self._warn_battery:
            self._last_battery_log = now
            self.get_logger().warn(f'[BAT] {self._battery_v:.2f}V < warn {self._warn_battery:.2f}V')


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


if __name__ == '__main__':
    main()
