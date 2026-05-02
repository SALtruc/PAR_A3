"""
Project C - refined reactive obstacle avoidance for ROSbot 3 PRO.

Behaviour goal:
- Default: drive straight. No corridor centering, no gap following.
- If front is suspicious: stop and OBSERVE for a few frames.
- If the object disappears / depth confirms clear: drive straight again.
- If the object remains: dodge gently toward the clearer side.
- If it is too close or a true dead-end: backup, then rotate to recover.

States: DRIVE | OBSERVE | DODGE | BACKUP | ROTATE | STOPPED
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
OBSERVE = 'OBSERVE'
DODGE = 'DODGE'
BACKUP = 'BACKUP'
ROTATE = 'ROTATE'
STOPPED = 'STOPPED'


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


@dataclass
class Snap:
    front_lidar: float
    front_depth: float
    left: float
    right: float
    rear: float
    dynamic: bool
    emergency: bool
    lidar_ok: bool
    depth_ok: bool


class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Topics
        self.declare_parameter('obstacle_topic', '/obstacle_representation')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('battery_topic', '/battery')
        self.declare_parameter('state_topic', '/obstacle_avoidance_state')
        self.declare_parameter('cmd_vel_stamped', True)
        self.declare_parameter('cmd_vel_frame_id', 'base_link')
        self.declare_parameter('control_hz', 20.0)

        # Motion
        self.declare_parameter('max_speed', 0.10)
        self.declare_parameter('observe_speed', 0.0)
        self.declare_parameter('dodge_forward_speed', 0.045)
        self.declare_parameter('dodge_angular_speed', 0.25)
        self.declare_parameter('rotation_angular_speed', 0.35)
        self.declare_parameter('backup_speed', 0.07)

        # Distances, metres
        self.declare_parameter('clear_distance', 0.28)
        self.declare_parameter('stop_distance', 0.15)
        self.declare_parameter('dodge_clearance', 0.25)
        self.declare_parameter('rear_stop_distance', 0.20)

        # Timings / behaviour limits
        self.declare_parameter('observe_frames', 8)
        self.declare_parameter('backup_sec', 0.70)
        self.declare_parameter('dodge_step_deg', 30.0)
        self.declare_parameter('rotation_step_deg', 70.0)
        self.declare_parameter('max_rotation_attempts', 3)

        # Battery / debug
        self.declare_parameter('require_battery_ok', False)
        self.declare_parameter('min_battery_voltage', 8.5)
        self.declare_parameter('warn_battery_voltage', 9.0)
        self.declare_parameter('battery_stale_sec', 3.0)
        self.declare_parameter('debug_decisions', True)
        self.declare_parameter('debug_period_sec', 0.7)

        p = self.get_parameter
        obstacle_topic = p('obstacle_topic').value
        cmd_vel_topic = p('cmd_vel_topic').value
        battery_topic = p('battery_topic').value
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
        self._dodge_clear = float(p('dodge_clearance').value)
        self._rear_stop = float(p('rear_stop_distance').value)

        self._observe_frames = max(1, int(p('observe_frames').value))
        self._backup_sec = float(p('backup_sec').value)
        self._dodge_sec = math.radians(float(p('dodge_step_deg').value)) / max(abs(self._dodge_ang), 0.01)
        self._rotate_sec = math.radians(float(p('rotation_step_deg').value)) / max(abs(self._rot_ang), 0.01)
        self._max_rotations = max(1, int(p('max_rotation_attempts').value))

        self._require_battery = _as_bool(p('require_battery_ok').value)
        self._min_battery = float(p('min_battery_voltage').value)
        self._warn_battery = float(p('warn_battery_voltage').value)
        self._battery_stale = float(p('battery_stale_sec').value)
        self._debug = _as_bool(p('debug_decisions').value)
        self._debug_period = float(p('debug_period_sec').value)

        self._state = STOPPED
        self._state_end = 0.0
        self._turn_dir = 1.0
        self._observe_count = 0
        self._rotation_count = 0
        self._raw_obs = None
        self._raw_obs_time = None
        self._battery_v = None
        self._battery_time = None
        self._last_battery_log = 0.0
        self._last_debug_log = 0.0
        self._last_transition = None

        vel_type = TwistStamped if self._stamped else Twist
        self.create_subscription(String, obstacle_topic, self._on_obstacle, 10)
        self.create_subscription(BatteryState, battery_topic, self._on_battery, 10)
        self._cmd_pub = self.create_publisher(vel_type, cmd_vel_topic, 10)
        self._state_pub = self.create_publisher(String, state_topic, 10)
        self.create_timer(1.0 / max(hz, 1.0), self._loop)

        self.get_logger().info(
            f'Refined obstacle avoidance ready: clear={_cm(self._clear)} '
            f'stop={_cm(self._stop)} observe={self._observe_frames} '
            f'dodge_step={float(p("dodge_step_deg").value):.0f}deg '
            f'rotate_step={float(p("rotation_step_deg").value):.0f}deg'
        )

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

        if snap.emergency:
            self._set_state(STOPPED)
            self._log('tof_or_emergency_stop', snap)
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

        if self._state == BACKUP and self._handle_backup(twist, snap, now):
            self._log('backup', snap)
            self._publish_cmd(twist)
            return

        if self._state == ROTATE and self._handle_rotate(twist, snap, front, now):
            self._log('rotate', snap)
            self._publish_cmd(twist)
            return

        # Priority 1: too close = backup, not dodge.
        if self._too_close(snap):
            self._start_backup()
            self._handle_backup(twist, snap, now)
            self._log('too_close_backup', snap)
            self._publish_cmd(twist)
            return

        # Priority 2: true dead-end = backup then rotate.
        if self._dead_end(snap, front):
            self._start_backup()
            self._handle_backup(twist, snap, now)
            self._log('dead_end_backup', snap)
            self._publish_cmd(twist)
            return

        # Priority 3: suspicious front object = observe first.
        if front <= self._clear:
            self._start_observe()
            self._handle_observe(twist, snap, front)
            self._log('observe_start', snap)
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
            self._set_state(OBSERVE)

    def _handle_observe(self, twist: Twist, snap: Snap, front: float) -> bool:
        self._observe_count += 1

        if self._too_close(snap):
            self._start_backup()
            return True

        # Object disappeared, or LIDAR is suspicious but depth says clear.
        if front > self._clear or self._depth_confirms_clear(snap):
            self._set_state(DRIVE)
            return False

        if self._observe_count < self._observe_frames:
            twist.linear.x = self._observe_speed
            twist.angular.z = 0.0
            return True

        # Still blocked after observation.
        if self._dead_end(snap, front):
            self._start_backup()
            return True

        self._start_dodge(snap)
        return True

    def _start_dodge(self, snap: Snap):
        self._turn_dir = self._clearer_side(snap.left, snap.right)
        self._set_state(DODGE, self._dodge_sec)

    def _handle_dodge(self, twist: Twist, snap: Snap, front: float, now: float) -> bool:
        if self._too_close(snap):
            self._start_backup()
            return True

        # Exit dodge once front is usable again.
        if front > self._clear or self._depth_confirms_clear(snap):
            self._set_state(DRIVE)
            return False

        if now < self._state_end:
            twist.linear.x = self._dodge_forward
            twist.angular.z = self._turn_dir * self._dodge_ang
            return True

        # Finished one gentle dodge step. Continue straight in the new heading.
        self._set_state(DRIVE)
        return False

    def _start_backup(self):
        self._rotation_count = 0
        self._set_state(BACKUP, self._backup_sec)

    def _handle_backup(self, twist: Twist, snap: Snap, now: float) -> bool:
        rear_blocked = math.isfinite(snap.rear) and snap.rear < self._rear_stop
        if now < self._state_end and not rear_blocked:
            twist.linear.x = -abs(self._backup_speed)
            twist.angular.z = 0.0
            return True

        self._start_rotate(snap)
        return True

    def _start_rotate(self, snap: Snap):
        self._turn_dir = self._clearer_side(snap.left, snap.right)
        self._set_state(ROTATE, self._rotate_sec)

    def _handle_rotate(self, twist: Twist, snap: Snap, front: float, now: float) -> bool:
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

    def _depth_confirms_clear(self, snap: Snap) -> bool:
        return (
            snap.depth_ok
            and math.isfinite(snap.front_depth)
            and snap.front_depth >= self._clear
            and not snap.dynamic
        )

    def _too_close(self, snap: Snap) -> bool:
        # Hard safety: any confirmed front reading under stop distance.
        lidar_too_close = math.isfinite(snap.front_lidar) and snap.front_lidar <= self._stop
        depth_too_close = math.isfinite(snap.front_depth) and snap.front_depth <= self._stop

        # If depth is available and clearly open, ignore medium LIDAR noise.
        if self._depth_confirms_clear(snap):
            return False

        return lidar_too_close or depth_too_close

    def _dead_end(self, snap: Snap, front: float) -> bool:
        front_blocked = front <= self._clear and not self._depth_confirms_clear(snap)
        left_blocked = math.isfinite(snap.left) and snap.left < self._dodge_clear
        right_blocked = math.isfinite(snap.right) and snap.right < self._dodge_clear
        return front_blocked and left_blocked and right_blocked

    @staticmethod
    def _clearer_side(left: float, right: float) -> float:
        left_clear = left if math.isfinite(left) else math.inf
        right_clear = right if math.isfinite(right) else math.inf
        return 1.0 if left_clear >= right_clear else -1.0

    def _snap(self) -> Snap:
        data = self._raw_obs or {}
        fused = data.get('fused', {})
        lidar = data.get('lidar', {})
        depth = data.get('depth', {})

        source = list(fused.get('source', []))
        lidar_ok = bool(lidar.get('available', False))
        depth_ok = bool(depth.get('available', False))

        return Snap(
            front_lidar=_finite(lidar.get('front_control', fused.get('front_distance'))),
            front_depth=_finite(depth.get('front_min')),
            left=_finite(fused.get('left_distance')),
            right=_finite(fused.get('right_distance')),
            rear=_finite(fused.get('rear_distance')),
            dynamic=bool(fused.get('dynamic_obstacle', False) or depth.get('motion', False)),
            emergency=bool(fused.get('emergency', False) and ('tof' in source or True)),
            lidar_ok=lidar_ok,
            depth_ok=depth_ok,
        )

    # ---------------------------------------------------------------------
    # ROS publish / logging
    # ---------------------------------------------------------------------

    def _set_state(self, state: str, duration: float = 0.0):
        if self._state != state:
            old = self._state
            self._state = state
            self._last_transition = f'{old}->{state}'
            self.get_logger().info(f'FSM: {old} -> {state}')
            msg = String()
            msg.data = f'{time.time():.3f},{state}'
            self._state_pub.publish(msg)
        self._state_end = time.monotonic() + max(0.0, duration)

    def _publish_cmd(self, twist: Twist):
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
        line = (
            f'[NAV] state={self._state} reason={reason} '
            f'front_lidar={_cm(snap.front_lidar)} '
            f'front_depth={_cm(snap.front_depth)} '
            f'front_eff={_cm(front)} '
            f'left={_cm(snap.left)} right={_cm(snap.right)} rear={_cm(snap.rear)} '
            f'dynamic={snap.dynamic} turn={"L" if self._turn_dir > 0 else "R"} '
            f'obs={self._observe_count}'
        )
        if self._state in (OBSERVE, DODGE, BACKUP, ROTATE):
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
