"""
Navigation FSM Node
-------------------
Command-driven finite state machine for QR navigation.

Immediate commands interrupt the current action. Queued commands use the AND_
prefix, for example AND_TURN_LEFT, and run after the current finite action.
When GO is received, or while the robot is driving, a fused LIDAR/depth front
obstacle check can trigger a side-step avoidance routine. The robot chooses the
clearer side from LIDAR sectors, drives around the obstacle, then resumes.

Extended features:
  - IMU yaw feedback for accurate turns (timer fallback when IMU unavailable).
  - ToF emergency stop: hard-stops below tof_emergency_dist regardless of state.
  - Depth camera secondary obstacle detection fused with LIDAR (Project C).
  - RECOVERING state uses wall-following (Project C) instead of blind slow drive.
"""

import math
import time
from collections import deque

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan, Range
from std_msgs.msg import String


DRIVING = 'DRIVING'
TURNING = 'TURNING'
STOPPED = 'STOPPED'
RECOVERING = 'RECOVERING'
AVOIDING = 'AVOIDING'

BASE_COMMANDS = {
    'TURN_LEFT', 'TURN_RIGHT', 'STOP', 'GO',
    'SPEED_UP', 'SPEED_DOWN', 'U_TURN',
}

DEFAULT_CRUISE_SPEED = 0.20
DEFAULT_SLOW_SPEED = 0.10
DEFAULT_SPEED_STEP = 0.05
DEFAULT_MAX_SPEED = 0.40
DEFAULT_MIN_SPEED = 0.05
DEFAULT_TURN_SPEED = 0.50
DEFAULT_TURN_90_SEC = (math.pi / 2) / DEFAULT_TURN_SPEED
DEFAULT_TURN_180_SEC = math.pi / DEFAULT_TURN_SPEED
DEFAULT_RECOVERY_SEC = 10.0

DEFAULT_OBSTACLE_DISTANCE = 0.30
DEFAULT_OBSTACLE_FRONT_ANGLE_DEG = 15.0
DEFAULT_OBSTACLE_CONFIRM_SEC = 0.35
DEFAULT_OBSTACLE_MIN_POINTS = 5
DEFAULT_OBSTACLE_PERCENTILE = 10.0
DEFAULT_OBSTACLE_SAFETY_ENABLED = True
DEFAULT_AVOID_TURN_SEC = DEFAULT_TURN_90_SEC
DEFAULT_AVOID_FORWARD_SEC = 1.5
DEFAULT_AVOID_PASS_SEC = 1.2
DEFAULT_AVOID_RETURN_SEC = DEFAULT_AVOID_FORWARD_SEC
DEFAULT_AVOID_RETURN_TO_PATH = True
DEFAULT_AVOID_TURN_DIRECTION = 1.0
DEFAULT_CONTINUOUS_OBSTACLE_AVOIDANCE = True
DEFAULT_OBSTACLE_STOP_ONLY = True
DEFAULT_AVOID_SIDE_SECTOR_DEG = 70.0
DEFAULT_AVOID_RETRY_LIMIT = 3
DEFAULT_SENSOR_STALE_SEC = 1.0

# ToF safety
DEFAULT_TOF_EMERGENCY_DIST = 0.15   # meters; hard stop below this
DEFAULT_TOF_TOPIC = '/range'

# IMU-assisted turns
DEFAULT_IMU_TOPIC = '/imu/data'
DEFAULT_USE_IMU_FOR_TURNS = True

# Depth camera secondary obstacle sensor (Project C integration)
DEFAULT_DEPTH_TOPIC = '/camera/depth/image_rect_raw'
DEFAULT_DEPTH_OBSTACLE_DIST = 0.50   # meters; center-patch min depth threshold
DEFAULT_DEPTH_CENTER_FRACTION = 0.33  # fraction of image width/height for center ROI

# Wall-following in RECOVERING (Project C integration)
DEFAULT_WALL_FOLLOW_KP = 0.4         # proportional gain on left/right distance error
DEFAULT_WALL_SECTOR_DEG = 60.0       # half-width of left/right LIDAR sectors

# ROSbot snap controllers commonly subscribe to geometry_msgs/TwistStamped on
# /cmd_vel. Keep this configurable so simulator or older stacks can use Twist.
DEFAULT_CMD_VEL_STAMPED = True
DEFAULT_CMD_VEL_FRAME_ID = 'base_link'
DEFAULT_STOP_AFTER_TURN = True


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def _as_float(value) -> float:
    return float(value)


def _as_int(value) -> int:
    return int(float(value))


class NavigationFSMNode(Node):

    def __init__(self):
        super().__init__('navigation_fsm')

        self.declare_parameter('cruise_speed', DEFAULT_CRUISE_SPEED)
        self.declare_parameter('slow_speed', DEFAULT_SLOW_SPEED)
        self.declare_parameter('speed_step', DEFAULT_SPEED_STEP)
        self.declare_parameter('max_speed', DEFAULT_MAX_SPEED)
        self.declare_parameter('min_speed', DEFAULT_MIN_SPEED)
        self.declare_parameter('turn_speed', DEFAULT_TURN_SPEED)
        self.declare_parameter('turn_90_sec', DEFAULT_TURN_90_SEC)
        self.declare_parameter('turn_180_sec', DEFAULT_TURN_180_SEC)
        self.declare_parameter('recovery_sec', DEFAULT_RECOVERY_SEC)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('start_state', DRIVING)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('obstacle_distance', DEFAULT_OBSTACLE_DISTANCE)
        self.declare_parameter(
            'obstacle_front_angle_deg',
            DEFAULT_OBSTACLE_FRONT_ANGLE_DEG,
        )
        self.declare_parameter('obstacle_confirm_sec', DEFAULT_OBSTACLE_CONFIRM_SEC)
        self.declare_parameter('obstacle_min_points', DEFAULT_OBSTACLE_MIN_POINTS)
        self.declare_parameter('obstacle_percentile', DEFAULT_OBSTACLE_PERCENTILE)
        self.declare_parameter(
            'obstacle_safety_enabled',
            DEFAULT_OBSTACLE_SAFETY_ENABLED,
        )
        self.declare_parameter('avoid_turn_sec', DEFAULT_AVOID_TURN_SEC)
        self.declare_parameter('avoid_forward_sec', DEFAULT_AVOID_FORWARD_SEC)
        self.declare_parameter('avoid_pass_sec', DEFAULT_AVOID_PASS_SEC)
        self.declare_parameter('avoid_return_sec', DEFAULT_AVOID_RETURN_SEC)
        self.declare_parameter('avoid_return_to_path', DEFAULT_AVOID_RETURN_TO_PATH)
        self.declare_parameter('avoid_turn_direction', DEFAULT_AVOID_TURN_DIRECTION)
        self.declare_parameter(
            'continuous_obstacle_avoidance',
            DEFAULT_CONTINUOUS_OBSTACLE_AVOIDANCE,
        )
        self.declare_parameter('obstacle_stop_only', DEFAULT_OBSTACLE_STOP_ONLY)
        self.declare_parameter('avoid_side_sector_deg', DEFAULT_AVOID_SIDE_SECTOR_DEG)
        self.declare_parameter('avoid_retry_limit', DEFAULT_AVOID_RETRY_LIMIT)
        self.declare_parameter('sensor_stale_sec', DEFAULT_SENSOR_STALE_SEC)
        self.declare_parameter('tof_topic', DEFAULT_TOF_TOPIC)
        self.declare_parameter('tof_emergency_dist', DEFAULT_TOF_EMERGENCY_DIST)
        self.declare_parameter('imu_topic', DEFAULT_IMU_TOPIC)
        self.declare_parameter('use_imu_for_turns', DEFAULT_USE_IMU_FOR_TURNS)
        self.declare_parameter('depth_topic', DEFAULT_DEPTH_TOPIC)
        self.declare_parameter('depth_obstacle_dist', DEFAULT_DEPTH_OBSTACLE_DIST)
        self.declare_parameter('depth_center_fraction', DEFAULT_DEPTH_CENTER_FRACTION)
        self.declare_parameter('wall_follow_kp', DEFAULT_WALL_FOLLOW_KP)
        self.declare_parameter('wall_sector_deg', DEFAULT_WALL_SECTOR_DEG)
        self.declare_parameter('cmd_vel_stamped', DEFAULT_CMD_VEL_STAMPED)
        self.declare_parameter('cmd_vel_frame_id', DEFAULT_CMD_VEL_FRAME_ID)
        self.declare_parameter('stop_after_turn', DEFAULT_STOP_AFTER_TURN)

        self.cruise_speed = _as_float(self.get_parameter('cruise_speed').value)
        self.slow_speed = _as_float(self.get_parameter('slow_speed').value)
        self.speed_step = _as_float(self.get_parameter('speed_step').value)
        self.max_speed = _as_float(self.get_parameter('max_speed').value)
        self.min_speed = _as_float(self.get_parameter('min_speed').value)
        self.turn_speed = _as_float(self.get_parameter('turn_speed').value)
        self.turn_90_sec = _as_float(self.get_parameter('turn_90_sec').value)
        self.turn_180_sec = _as_float(self.get_parameter('turn_180_sec').value)
        self.recovery_sec = _as_float(self.get_parameter('recovery_sec').value)
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        start_state = self.get_parameter('start_state').value
        scan_topic = self.get_parameter('scan_topic').value

        self.obstacle_distance = _as_float(
            self.get_parameter('obstacle_distance').value
        )
        self.obstacle_front_angle = math.radians(
            _as_float(self.get_parameter('obstacle_front_angle_deg').value)
        )
        self.obstacle_confirm_sec = _as_float(
            self.get_parameter('obstacle_confirm_sec').value
        )
        self.obstacle_min_points = max(
            1,
            _as_int(self.get_parameter('obstacle_min_points').value),
        )
        self.obstacle_percentile = max(
            0.0,
            min(100.0, _as_float(self.get_parameter('obstacle_percentile').value)),
        )
        self.obstacle_safety_enabled = _as_bool(
            self.get_parameter('obstacle_safety_enabled').value
        )
        self.avoid_turn_sec = _as_float(self.get_parameter('avoid_turn_sec').value)
        self.avoid_forward_sec = _as_float(
            self.get_parameter('avoid_forward_sec').value
        )
        self.avoid_pass_sec = _as_float(self.get_parameter('avoid_pass_sec').value)
        self.avoid_return_sec = _as_float(
            self.get_parameter('avoid_return_sec').value
        )
        self.avoid_return_to_path = _as_bool(
            self.get_parameter('avoid_return_to_path').value
        )
        self.default_avoid_turn_direction = _as_float(
            self.get_parameter('avoid_turn_direction').value
        )
        self.continuous_obstacle_avoidance = _as_bool(
            self.get_parameter('continuous_obstacle_avoidance').value
        )
        self.obstacle_stop_only = _as_bool(
            self.get_parameter('obstacle_stop_only').value
        )
        self.avoid_side_sector_rad = math.radians(
            _as_float(self.get_parameter('avoid_side_sector_deg').value)
        )
        self.avoid_retry_limit = _as_int(self.get_parameter('avoid_retry_limit').value)
        self.sensor_stale_sec = _as_float(self.get_parameter('sensor_stale_sec').value)

        self.tof_emergency_dist = _as_float(
            self.get_parameter('tof_emergency_dist').value
        )
        self.use_imu_for_turns = _as_bool(self.get_parameter('use_imu_for_turns').value)
        self.cmd_vel_stamped = _as_bool(self.get_parameter('cmd_vel_stamped').value)
        self.cmd_vel_frame_id = self.get_parameter('cmd_vel_frame_id').value
        self.stop_after_turn = _as_bool(self.get_parameter('stop_after_turn').value)
        self.depth_obstacle_dist = _as_float(
            self.get_parameter('depth_obstacle_dist').value
        )
        self.depth_center_fraction = _as_float(
            self.get_parameter('depth_center_fraction').value
        )
        self.wall_follow_kp = _as_float(self.get_parameter('wall_follow_kp').value)
        self.wall_sector_rad = math.radians(
            _as_float(self.get_parameter('wall_sector_deg').value)
        )

        self._current_speed = self.cruise_speed
        self._state = start_state
        self._prev_state = start_state
        self._turn_direction = 0.0
        self._turn_end_time = 0.0
        self._turn_target_rad = 0.0   # IMU-mode: target accumulated yaw
        self._turn_next_state = STOPPED
        self._last_command_time = time.monotonic()

        self._queued_actions = deque()
        self._latest_scan: LaserScan | None = None
        self._last_scan_time: float | None = None
        self._avoid_steps: deque[tuple[str, float]] = deque()
        self._avoid_step: str | None = None
        self._avoid_step_end_time = 0.0
        self._avoid_turn_direction = self.default_avoid_turn_direction
        self._avoid_retry_count = 0
        self._obstacle_first_seen_time: float | None = None
        self._obstacle_confirmed = False

        # IMU yaw tracking
        self._current_imu_yaw: float | None = None
        self._imu_yaw_start: float | None = None
        self._imu_active = False  # set True on first IMU message

        # ToF safety
        self._tof_range: float = math.inf
        self._last_tof_time: float | None = None
        self._tof_emergency_active = False

        # Depth camera (secondary obstacle sensor, Project C integration)
        self._depth_front_dist: float = math.inf
        self._last_depth_time: float | None = None
        self._bridge = CvBridge()

        self.cmd_sub = self.create_subscription(
            String, '/qr_command', self._on_command, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._on_scan, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self._on_imu, 10
        )
        self.tof_sub = self.create_subscription(
            Range, self.get_parameter('tof_topic').value, self._on_tof, 10
        )
        self.depth_sub = self.create_subscription(
            Image, self.get_parameter('depth_topic').value, self._on_depth, 10
        )
        self.state_pub = self.create_publisher(String, '/fsm_state', 10)
        self.evt_pub = self.create_publisher(String, '/qr_event', 10)
        vel_msg_type = TwistStamped if self.cmd_vel_stamped else Twist
        self.vel_pub = self.create_publisher(vel_msg_type, cmd_vel_topic, 10)

        self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f'Navigation FSM ready. Initial state: {self._state}. '
            f'cmd_vel -> {cmd_vel_topic} ({vel_msg_type.__name__}), '
            f'scan -> {scan_topic}, '
            f'IMU turns: {self.use_imu_for_turns}, '
            'QR turns finish in STOPPED, '
            f'obstacle safety: {self.obstacle_safety_enabled}, '
            f'obstacle response: {"STOP" if self.obstacle_stop_only else "AVOID"}'
        )

    def _on_command(self, msg: String):
        cmd = msg.data.strip().upper()
        self._last_command_time = time.monotonic()
        self.get_logger().info(f'FSM received command: {cmd} (state={self._state})')

        if cmd.startswith('AND_'):
            queued_cmd = cmd[4:]
            if queued_cmd not in BASE_COMMANDS:
                self.get_logger().warn(f'Unknown queued command ignored: {cmd!r}')
                return

            self._queued_actions.append(queued_cmd)
            self._publish_event('QUEUE', queued_cmd)
            self.get_logger().info(
                f'Queued action: {queued_cmd} (queue_len={len(self._queued_actions)})'
            )
            self._maybe_start_next_queued_action()
            return

        self._execute_command(cmd)

    def _execute_command(self, cmd: str):
        if cmd == 'STOP':
            self._avoid_steps.clear()
            self._avoid_retry_count = 0
            self._transition(STOPPED)

        elif cmd == 'GO':
            if self._state == TURNING and self._turn_next_state == STOPPED:
                if 'GO' not in self._queued_actions:
                    self._queued_actions.appendleft('GO')
                    self._publish_event('QUEUE', 'GO')
                    self.get_logger().info('Queued GO until the active turn completes.')
                return

            if self._state == AVOIDING:
                self.get_logger().info('GO ignored while obstacle avoidance is active.')
                return

            if self._front_obstacle_detected():
                self._handle_obstacle(reason='GO')
            elif self._state in (STOPPED, RECOVERING, TURNING, AVOIDING):
                self._transition(DRIVING)

        elif cmd == 'TURN_LEFT':
            self._start_turn(+1.0, self.turn_90_sec, next_state=STOPPED)

        elif cmd == 'TURN_RIGHT':
            self._start_turn(-1.0, self.turn_90_sec, next_state=STOPPED)

        elif cmd == 'U_TURN':
            self._start_turn(+1.0, self.turn_180_sec, next_state=STOPPED)

        elif cmd == 'SPEED_UP':
            self._current_speed = min(
                self._current_speed + self.speed_step, self.max_speed
            )
            self.get_logger().info(f'Speed -> {self._current_speed:.2f} m/s')

        elif cmd == 'SPEED_DOWN':
            self._current_speed = max(
                self._current_speed - self.speed_step, self.min_speed
            )
            self.get_logger().info(f'Speed -> {self._current_speed:.2f} m/s')

    def _maybe_start_next_queued_action(self, allow_when_stopped: bool = False):
        if self._state in (TURNING, AVOIDING):
            return
        if not self._queued_actions:
            return
        if (
                self._state == STOPPED
                and self._queued_actions[0] != 'GO'
                and not allow_when_stopped):
            return

        next_cmd = self._queued_actions.popleft()
        self._publish_event('DEQUEUE', next_cmd)
        self.get_logger().info(
            f'Running queued action: {next_cmd} (remaining={len(self._queued_actions)})'
        )
        self._execute_command(next_cmd)

    def _start_turn(self, direction: float, duration: float, next_state: str = STOPPED):
        self._avoid_steps.clear()
        self._turn_direction = direction
        self._turn_end_time = time.monotonic() + duration
        self._turn_next_state = next_state
        # IMU mode: record start yaw and target angle
        if self.use_imu_for_turns and self._imu_active:
            self._imu_yaw_start = self._current_imu_yaw
            self._turn_target_rad = duration * self.turn_speed  # angle = time × ω
        else:
            self._imu_yaw_start = None
        self._transition(TURNING)

    def _turn_complete(self, now: float) -> bool:
        """True when the current turn should end (IMU primary, timer fallback)."""
        if (self.use_imu_for_turns
                and self._imu_active
                and self._imu_yaw_start is not None):
            delta = self._yaw_delta(self._imu_yaw_start, self._current_imu_yaw)
            # Check accumulated rotation in the commanded direction
            if self._turn_direction * delta >= self._turn_target_rad:
                return True
            # Safety: also respect timer as upper bound
            return now >= self._turn_end_time + 1.0
        return now >= self._turn_end_time

    @staticmethod
    def _yaw_from_imu(imu: Imu) -> float:
        """Extract yaw (Z-axis rotation) from IMU quaternion."""
        q = imu.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _yaw_delta(start: float, current: float) -> float:
        """Signed shortest-path angle difference current - start, in (-π, π]."""
        d = current - start
        while d > math.pi:
            d -= 2 * math.pi
        while d <= -math.pi:
            d += 2 * math.pi
        return d

    def _sensor_recent(self, stamp: float | None) -> bool:
        return stamp is not None and time.monotonic() - stamp <= self.sensor_stale_sec

    def _on_imu(self, msg: Imu):
        self._current_imu_yaw = self._yaw_from_imu(msg)
        if not self._imu_active:
            self._imu_active = True
            self.get_logger().info('IMU online – using yaw feedback for turns.')

    def _on_tof(self, msg: Range):
        """VL53L0X ToF range callback – feeds the emergency stop layer."""
        self._last_tof_time = time.monotonic()
        if math.isfinite(msg.range) and msg.min_range < msg.range < msg.max_range:
            self._tof_range = msg.range
        else:
            self._tof_range = math.inf

    def _on_depth(self, msg: Image):
        """OAK-D depth image callback – secondary front-obstacle sensor (Project C)."""
        try:
            depth_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception:
            return

        self._last_depth_time = time.monotonic()
        h, w = depth_img.shape[:2]
        cx = w // 2
        cy = h // 2
        half_w = int(w * self.depth_center_fraction / 2)
        half_h = int(h * self.depth_center_fraction / 2)
        roi = depth_img[cy - half_h:cy + half_h, cx - half_w:cx + half_w]

        valid = roi[np.isfinite(roi) & (roi > 0)]
        if valid.size == 0:
            self._depth_front_dist = math.inf
            return

        # OAK-D often publishes 16UC1 depth in mm, but some pipelines publish
        # floating-point metres. Use encoding/dtype so both cases work.
        depth_value = float(np.percentile(valid, 5))
        encoding = msg.encoding.lower()
        if '32f' in encoding or '64f' in encoding or np.issubdtype(valid.dtype, np.floating):
            min_depth_m = depth_value
        else:
            min_depth_m = depth_value / 1000.0
        self._depth_front_dist = min_depth_m

    def _on_scan(self, msg: LaserScan):
        self._latest_scan = msg
        self._last_scan_time = time.monotonic()

    def _front_obstacle_detected(self, log: bool = True) -> bool:
        """Fuse LIDAR + depth camera for front-obstacle check (Project C sensor fusion)."""
        if not self.obstacle_safety_enabled:
            self._obstacle_first_seen_time = None
            self._obstacle_confirmed = False
            return False

        lidar_dist, lidar_close_points = self._lidar_front_distance()
        depth_dist = (
            self._depth_front_dist
            if self._sensor_recent(self._last_depth_time)
            else math.inf
        )

        # Require multiple close LIDAR rays and a short confirmation window so
        # one noisy ray or a wall edge does not kick the robot into avoidance.
        blocked_lidar = (
            lidar_dist < self.obstacle_distance
            and lidar_close_points >= self.obstacle_min_points
        )
        blocked_depth = depth_dist < self.depth_obstacle_dist
        raw_blocked = blocked_lidar or blocked_depth

        now = time.monotonic()
        if raw_blocked:
            if self._obstacle_first_seen_time is None:
                self._obstacle_first_seen_time = now
            confirmed = (
                self.obstacle_confirm_sec <= 0.0
                or now - self._obstacle_first_seen_time >= self.obstacle_confirm_sec
            )
        else:
            self._obstacle_first_seen_time = None
            self._obstacle_confirmed = False
            return False

        if confirmed:
            source = []
            if blocked_lidar:
                source.append(
                    f'LIDAR={lidar_dist:.2f}m/{lidar_close_points}pts'
                )
            if blocked_depth:
                source.append(f'depth={depth_dist:.2f}m')
            if log and not self._obstacle_confirmed:
                action = (
                    'Stopping.'
                    if self.obstacle_stop_only
                    else 'Starting avoidance.'
                )
                self.get_logger().warn(
                    f'Obstacle detected [{", ".join(source)}]. {action}'
                )
            self._obstacle_confirmed = True
            return True
        return False

    def _lidar_front_distance(self) -> tuple[float, int]:
        """Return robust front-cone distance and close-ray count."""
        scan = self._latest_scan
        if scan is None or not self._sensor_recent(self._last_scan_time):
            return math.inf, 0
        values = []
        close_points = 0
        angle = scan.angle_min
        for value in scan.ranges:
            if -self.obstacle_front_angle <= angle <= self.obstacle_front_angle:
                if math.isfinite(value) and scan.range_min <= value <= scan.range_max:
                    values.append(value)
                    if value < self.obstacle_distance:
                        close_points += 1
            angle += scan.angle_increment
        if not values:
            return math.inf, 0
        distance = float(np.percentile(values, self.obstacle_percentile))
        return distance, close_points

    def _sector_mean(self, scan: LaserScan, angle_lo: float, angle_hi: float) -> float:
        """Mean valid range in an angular sector (radians). Returns inf if no data."""
        values = []
        angle = scan.angle_min
        for value in scan.ranges:
            if angle_lo <= angle <= angle_hi:
                if math.isfinite(value) and scan.range_min <= value <= scan.range_max:
                    values.append(value)
            angle += scan.angle_increment
        return float(np.mean(values)) if values else math.inf

    def _wall_follow_twist(self) -> Twist:
        """Wall-centering twist for RECOVERING state (Project C element)."""
        twist = Twist()
        twist.linear.x = self.slow_speed
        scan = self._latest_scan
        if scan is None or not self._sensor_recent(self._last_scan_time):
            return twist
        left_dist = self._sector_mean(
            scan,
            math.pi / 2 - self.wall_sector_rad,
            math.pi / 2 + self.wall_sector_rad,
        )
        right_dist = self._sector_mean(
            scan,
            -(math.pi / 2 + self.wall_sector_rad),
            -(math.pi / 2 - self.wall_sector_rad),
        )
        if math.isfinite(left_dist) and math.isfinite(right_dist):
            # Positive error → right wall closer → steer left
            error = left_dist - right_dist
            twist.angular.z = max(-0.3, min(0.3, self.wall_follow_kp * error))
        return twist

    def _choose_avoid_direction(self) -> float:
        """Choose the side with more LIDAR clearance; +1 turns left, -1 right."""
        scan = self._latest_scan
        if scan is None or not self._sensor_recent(self._last_scan_time):
            return 1.0 if self.default_avoid_turn_direction >= 0.0 else -1.0

        left_clearance = self._sector_mean(
            scan,
            math.radians(20.0),
            self.avoid_side_sector_rad,
        )
        right_clearance = self._sector_mean(
            scan,
            -self.avoid_side_sector_rad,
            math.radians(-20.0),
        )

        if not math.isfinite(left_clearance) and not math.isfinite(right_clearance):
            return 1.0 if self.default_avoid_turn_direction >= 0.0 else -1.0
        if not math.isfinite(left_clearance):
            return -1.0
        if not math.isfinite(right_clearance):
            return 1.0
        return 1.0 if left_clearance >= right_clearance else -1.0

    def _handle_obstacle(self, reason: str):
        if self.obstacle_stop_only:
            self._avoid_steps.clear()
            self._avoid_retry_count = 0
            self._publish_event('AVOID', f'stopped_{reason}')
            self._transition(STOPPED)
            return

        self._start_avoidance(reason=reason)

    def _start_avoidance(self, reason: str = 'front_obstacle', retry: bool = False):
        if not retry:
            self._avoid_retry_count = 0

        self._avoid_turn_direction = self._choose_avoid_direction()
        avoid_steps = [
            ('turn_away', self.avoid_turn_sec),
            ('shift_out', self.avoid_forward_sec),
            ('turn_parallel', self.avoid_turn_sec),
        ]
        if self.avoid_return_to_path:
            avoid_steps.extend([
                ('pass_obstacle', self.avoid_pass_sec),
                ('turn_in', self.avoid_turn_sec),
                ('shift_back', self.avoid_return_sec),
                ('turn_align', self.avoid_turn_sec),
            ])
        self._avoid_steps = deque(avoid_steps)
        direction_label = 'left' if self._avoid_turn_direction > 0.0 else 'right'
        self.get_logger().warn(
            f'Starting obstacle avoidance ({reason}); turning {direction_label} first.'
        )
        self._start_next_avoid_step()
        self._transition(AVOIDING)

    def _start_next_avoid_step(self):
        if not self._avoid_steps:
            self._avoid_step = None
            if self._front_obstacle_detected(log=False):
                self._avoid_retry_count += 1
                if self._avoid_retry_count <= self.avoid_retry_limit:
                    self._publish_event('AVOID', f'retry_{self._avoid_retry_count}')
                    self.get_logger().warn(
                        f'Obstacle still ahead after avoidance; retry '
                        f'{self._avoid_retry_count}/{self.avoid_retry_limit}.'
                    )
                    self._start_avoidance(reason='retry', retry=True)
                    return

                self.get_logger().error(
                    'Obstacle still ahead after avoidance retry limit; stopping.'
                )
                self._avoid_retry_count = 0
                self._transition(STOPPED)
                return

            self._avoid_retry_count = 0
            self._last_command_time = time.monotonic()
            self._transition(DRIVING)
            self._maybe_start_next_queued_action()
            return

        self._avoid_step, duration = self._avoid_steps.popleft()
        self._avoid_step_end_time = time.monotonic() + duration
        self._publish_event('AVOID', self._avoid_step)

    def _avoidance_twist(self, now: float) -> Twist:
        if now >= self._avoid_step_end_time:
            self._start_next_avoid_step()

        twist = Twist()
        if self._avoid_step in ('turn_away', 'turn_align'):
            twist.angular.z = self._avoid_turn_direction * self.turn_speed
        elif self._avoid_step in ('shift_out', 'pass_obstacle', 'shift_back'):
            twist.linear.x = self.slow_speed
        elif self._avoid_step in ('turn_parallel', 'turn_in'):
            twist.angular.z = -self._avoid_turn_direction * self.turn_speed
        return twist

    def _transition(self, new_state: str):
        if self._state == new_state:
            return

        self.get_logger().info(f'FSM: {self._state} -> {new_state}')
        self._prev_state = self._state
        self._state = new_state
        self._publish_event('STATE', new_state)

        state_msg = String()
        state_msg.data = new_state
        self.state_pub.publish(state_msg)

    def _publish_event(self, event_type: str, value: str):
        evt = String()
        evt.data = f'{time.time():.3f},{event_type},{value}'
        self.evt_pub.publish(evt)

    def _publish_velocity(self, twist: Twist):
        if not self.cmd_vel_stamped:
            self.vel_pub.publish(twist)
            return

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.cmd_vel_frame_id)
        msg.twist = twist
        self.vel_pub.publish(msg)

    def _control_loop(self):
        now = time.monotonic()
        twist = Twist()

        # ── ToF emergency stop layer (Project C safety requirement) ───────
        tof_recent = self._sensor_recent(self._last_tof_time)
        tof_triggered = (
            (tof_recent and self._tof_range < self.tof_emergency_dist)
            or (self._tof_emergency_active and not tof_recent)
        )
        if tof_triggered and not self._tof_emergency_active:
            self._tof_emergency_active = True
            self._publish_event('STATE', 'TOF_EMERGENCY')
            self.get_logger().error(
                f'ToF emergency stop: range={self._tof_range:.3f}m '
                f'< threshold={self.tof_emergency_dist:.3f}m'
            )
        elif not tof_triggered and self._tof_emergency_active:
            self._tof_emergency_active = False
            self._publish_event('STATE', 'TOF_CLEAR')
            self.get_logger().info('ToF clear – resuming normal operation.')

        if self._tof_emergency_active:
            self._publish_velocity(Twist())  # hard stop, bypass all FSM logic
            return

        # ── Normal FSM ────────────────────────────────────────────────────
        if self._state == DRIVING:
            self._maybe_start_next_queued_action()
            if self._state == DRIVING:
                if (
                        self.continuous_obstacle_avoidance
                        and self._front_obstacle_detected()):
                    self._handle_obstacle(reason='driving')
                else:
                    twist.linear.x = self._current_speed

                if (
                        self._state == DRIVING
                        and now - self._last_command_time > self.recovery_sec):
                    self._transition(RECOVERING)

        elif self._state == TURNING:
            if self._turn_complete(now):
                self._transition(self._turn_next_state)
                self._maybe_start_next_queued_action(
                    allow_when_stopped=self._turn_next_state == STOPPED
                )
                if self._state == DRIVING:
                    twist.linear.x = self._current_speed
            else:
                twist.angular.z = self._turn_direction * self.turn_speed

        elif self._state == STOPPED:
            self._maybe_start_next_queued_action()

        elif self._state == RECOVERING:
            # Wall-following in recovery (Project C): centre between walls instead
            # of blindly driving forward, so the robot stays in the corridor.
            self._maybe_start_next_queued_action()
            if self._state == RECOVERING:
                twist = self._wall_follow_twist()
                if now - self._last_command_time < self.recovery_sec:
                    self._transition(DRIVING)

        elif self._state == AVOIDING:
            twist = self._avoidance_twist(now)

        self._publish_velocity(twist)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationFSMNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
