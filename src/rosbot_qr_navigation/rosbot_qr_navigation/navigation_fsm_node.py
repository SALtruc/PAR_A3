"""
Navigation FSM Node
-------------------
Command-driven finite state machine for QR navigation.

Immediate commands interrupt the current action. Queued commands use the AND_
prefix, for example AND_TURN_LEFT, and run after the current finite action.
When GO is received and a LaserScan obstacle is close in front, the robot runs
a simple timed avoidance routine, then resumes straight driving.

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
from geometry_msgs.msg import Twist
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

DEFAULT_OBSTACLE_DISTANCE = 0.45
DEFAULT_OBSTACLE_FRONT_ANGLE_DEG = 30.0
DEFAULT_AVOID_TURN_SEC = DEFAULT_TURN_90_SEC
DEFAULT_AVOID_FORWARD_SEC = 1.5
DEFAULT_AVOID_TURN_DIRECTION = 1.0

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
        self.declare_parameter('avoid_turn_sec', DEFAULT_AVOID_TURN_SEC)
        self.declare_parameter('avoid_forward_sec', DEFAULT_AVOID_FORWARD_SEC)
        self.declare_parameter('avoid_turn_direction', DEFAULT_AVOID_TURN_DIRECTION)
        self.declare_parameter('tof_topic', DEFAULT_TOF_TOPIC)
        self.declare_parameter('tof_emergency_dist', DEFAULT_TOF_EMERGENCY_DIST)
        self.declare_parameter('imu_topic', DEFAULT_IMU_TOPIC)
        self.declare_parameter('use_imu_for_turns', DEFAULT_USE_IMU_FOR_TURNS)
        self.declare_parameter('depth_topic', DEFAULT_DEPTH_TOPIC)
        self.declare_parameter('depth_obstacle_dist', DEFAULT_DEPTH_OBSTACLE_DIST)
        self.declare_parameter('depth_center_fraction', DEFAULT_DEPTH_CENTER_FRACTION)
        self.declare_parameter('wall_follow_kp', DEFAULT_WALL_FOLLOW_KP)
        self.declare_parameter('wall_sector_deg', DEFAULT_WALL_SECTOR_DEG)

        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.slow_speed = self.get_parameter('slow_speed').value
        self.speed_step = self.get_parameter('speed_step').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.turn_90_sec = self.get_parameter('turn_90_sec').value
        self.turn_180_sec = self.get_parameter('turn_180_sec').value
        self.recovery_sec = self.get_parameter('recovery_sec').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        start_state = self.get_parameter('start_state').value
        scan_topic = self.get_parameter('scan_topic').value

        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.obstacle_front_angle = math.radians(
            self.get_parameter('obstacle_front_angle_deg').value
        )
        self.avoid_turn_sec = self.get_parameter('avoid_turn_sec').value
        self.avoid_forward_sec = self.get_parameter('avoid_forward_sec').value
        self.avoid_turn_direction = self.get_parameter('avoid_turn_direction').value

        self.tof_emergency_dist = self.get_parameter('tof_emergency_dist').value
        self.use_imu_for_turns = self.get_parameter('use_imu_for_turns').value
        self.depth_obstacle_dist = self.get_parameter('depth_obstacle_dist').value
        self.depth_center_fraction = self.get_parameter('depth_center_fraction').value
        self.wall_follow_kp = self.get_parameter('wall_follow_kp').value
        self.wall_sector_rad = math.radians(self.get_parameter('wall_sector_deg').value)

        self._current_speed = self.cruise_speed
        self._state = start_state
        self._prev_state = start_state
        self._turn_direction = 0.0
        self._turn_end_time = 0.0
        self._turn_target_rad = 0.0   # IMU-mode: target accumulated yaw
        self._last_command_time = time.monotonic()

        self._queued_actions = deque()
        self._latest_scan: LaserScan | None = None
        self._avoid_steps: deque[tuple[str, float]] = deque()
        self._avoid_step: str | None = None
        self._avoid_step_end_time = 0.0

        # IMU yaw tracking
        self._current_imu_yaw: float | None = None
        self._imu_yaw_start: float | None = None
        self._imu_active = False  # set True on first IMU message

        # ToF safety
        self._tof_range: float = math.inf
        self._tof_emergency_active = False

        # Depth camera (secondary obstacle sensor, Project C integration)
        self._depth_front_dist: float = math.inf
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
        self.vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f'Navigation FSM ready. Initial state: {self._state}. '
            f'cmd_vel -> {cmd_vel_topic}, scan -> {scan_topic}, '
            f'IMU turns: {self.use_imu_for_turns}'
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
            self._transition(STOPPED)

        elif cmd == 'GO':
            if self._front_obstacle_detected():
                self._start_avoidance()
            elif self._state in (STOPPED, RECOVERING, TURNING, AVOIDING):
                self._transition(DRIVING)

        elif cmd == 'TURN_LEFT':
            self._start_turn(+1.0, self.turn_90_sec)

        elif cmd == 'TURN_RIGHT':
            self._start_turn(-1.0, self.turn_90_sec)

        elif cmd == 'U_TURN':
            self._start_turn(+1.0, self.turn_180_sec)

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

    def _maybe_start_next_queued_action(self):
        if self._state in (TURNING, AVOIDING):
            return
        if not self._queued_actions:
            return
        if self._state == STOPPED and self._queued_actions[0] != 'GO':
            return

        next_cmd = self._queued_actions.popleft()
        self._publish_event('DEQUEUE', next_cmd)
        self.get_logger().info(
            f'Running queued action: {next_cmd} (remaining={len(self._queued_actions)})'
        )
        self._execute_command(next_cmd)

    def _start_turn(self, direction: float, duration: float):
        self._avoid_steps.clear()
        self._turn_direction = direction
        self._turn_end_time = time.monotonic() + duration
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

    def _on_imu(self, msg: Imu):
        self._current_imu_yaw = self._yaw_from_imu(msg)
        if not self._imu_active:
            self._imu_active = True
            self.get_logger().info('IMU online – using yaw feedback for turns.')

    def _on_tof(self, msg: Range):
        """VL53L0X ToF range callback – feeds the emergency stop layer."""
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

        h, w = depth_img.shape[:2]
        cx = w // 2
        cy = h // 2
        half_w = int(w * self.depth_center_fraction / 2)
        half_h = int(h * self.depth_center_fraction / 2)
        roi = depth_img[cy - half_h:cy + half_h, cx - half_w:cx + half_w]

        valid = roi[roi > 0]
        if valid.size == 0:
            self._depth_front_dist = math.inf
            return

        # Depth is in mm for OAK-D; convert to metres
        min_depth_m = float(np.percentile(valid, 5)) / 1000.0
        self._depth_front_dist = min_depth_m

    def _on_scan(self, msg: LaserScan):
        self._latest_scan = msg

    def _front_obstacle_detected(self) -> bool:
        """Fuse LIDAR + depth camera for front-obstacle check (Project C sensor fusion)."""
        lidar_dist = self._lidar_front_min()
        depth_dist = self._depth_front_dist

        # Obstacle confirmed if either sensor triggers
        blocked_lidar = lidar_dist < self.obstacle_distance
        blocked_depth = depth_dist < self.depth_obstacle_dist

        if blocked_lidar or blocked_depth:
            source = []
            if blocked_lidar:
                source.append(f'LIDAR={lidar_dist:.2f}m')
            if blocked_depth:
                source.append(f'depth={depth_dist:.2f}m')
            self.get_logger().warn(
                f'Obstacle detected [{", ".join(source)}]. Starting avoidance.'
            )
            return True
        return False

    def _lidar_front_min(self) -> float:
        """Return minimum range in the front cone from the latest LaserScan."""
        scan = self._latest_scan
        if scan is None:
            return math.inf
        min_r = math.inf
        angle = scan.angle_min
        for value in scan.ranges:
            if -self.obstacle_front_angle <= angle <= self.obstacle_front_angle:
                if math.isfinite(value) and scan.range_min <= value <= scan.range_max:
                    min_r = min(min_r, value)
            angle += scan.angle_increment
        return min_r

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
        if scan is None:
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

    def _start_avoidance(self):
        self._avoid_steps = deque([
            ('turn_away', self.avoid_turn_sec),
            ('forward', self.avoid_forward_sec),
            ('turn_back', self.avoid_turn_sec),
        ])
        self._start_next_avoid_step()
        self._transition(AVOIDING)

    def _start_next_avoid_step(self):
        if not self._avoid_steps:
            self._avoid_step = None
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
        if self._avoid_step == 'turn_away':
            twist.angular.z = self.avoid_turn_direction * self.turn_speed
        elif self._avoid_step == 'forward':
            twist.linear.x = self.slow_speed
        elif self._avoid_step == 'turn_back':
            twist.angular.z = -self.avoid_turn_direction * self.turn_speed
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

    def _control_loop(self):
        now = time.monotonic()
        twist = Twist()

        # ── ToF emergency stop layer (Project C safety requirement) ───────
        tof_triggered = self._tof_range < self.tof_emergency_dist
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
            self.vel_pub.publish(Twist())  # hard stop, bypass all FSM logic
            return

        # ── Normal FSM ────────────────────────────────────────────────────
        if self._state == DRIVING:
            self._maybe_start_next_queued_action()
            if self._state == DRIVING:
                twist.linear.x = self._current_speed
                if now - self._last_command_time > self.recovery_sec:
                    self._transition(RECOVERING)

        elif self._state == TURNING:
            if self._turn_complete(now):
                self._transition(DRIVING)
                self._maybe_start_next_queued_action()
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

        self.vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationFSMNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
