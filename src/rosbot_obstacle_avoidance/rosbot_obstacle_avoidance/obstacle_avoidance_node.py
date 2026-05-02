"""
ROSbot simplified obstacle avoidance.

States: DRIVE | OBSERVE | ROTATE | BACKUP | STOPPED

Policy:
  front > clear_dist (20 cm)  → DRIVE straight, sides ignored completely
  stop_dist < front <= clear  → OBSERVE N frames
    · if front clears          → DRIVE
    · after N frames, depth clear but LIDAR blocked → DRIVE (false positive)
    · after N frames, both blocked → ROTATE toward clearer side
  front <= stop_dist (15 cm)  → BACKUP immediately
  dead-end (front blocked + both sides < dodge_clearance) → BACKUP

ROTATE: spin toward the side with more clearance; exit the moment front clears.
LIDAR primary; if depth clearly open (>= clear_dist) while LIDAR close → OBSERVE, not panic.
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


DRIVE   = 'DRIVE'
OBSERVE = 'OBSERVE'
ROTATE  = 'ROTATE'
BACKUP  = 'BACKUP'
STOPPED = 'STOPPED'


def _finite(v) -> float:
    try:
        f = float(v)
        return f if math.isfinite(f) else math.inf
    except (TypeError, ValueError):
        return math.inf


def _cm(v: float) -> str:
    return f'{v * 100:.0f}cm' if math.isfinite(v) else 'inf'


def _as_bool(v) -> bool:
    if isinstance(v, bool):
        return v
    if isinstance(v, str):
        return v.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(v)


@dataclass
class Snap:
    front_lidar: float
    front_depth: float
    left: float
    right: float
    rear: float
    dynamic: bool
    tof_emergency: bool
    lidar_ok: bool
    depth_ok: bool


class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.declare_parameter('obstacle_topic',          '/obstacle_representation')
        self.declare_parameter('cmd_vel_topic',           '/cmd_vel')
        self.declare_parameter('battery_topic',           '/battery')
        self.declare_parameter('state_topic',             '/obstacle_avoidance_state')
        self.declare_parameter('cmd_vel_stamped',         True)
        self.declare_parameter('cmd_vel_frame_id',        'base_link')
        self.declare_parameter('control_hz',              20.0)

        self.declare_parameter('max_speed',               0.10)
        self.declare_parameter('min_speed',               0.05)
        self.declare_parameter('rotation_angular_speed',  0.45)

        self.declare_parameter('clear_distance',          0.20)
        self.declare_parameter('stop_distance',           0.15)
        self.declare_parameter('dodge_clearance',         0.25)
        self.declare_parameter('rear_stop_distance',      0.20)

        self.declare_parameter('observe_frames',          5)
        self.declare_parameter('backup_speed',            0.07)
        self.declare_parameter('backup_sec',              0.80)
        self.declare_parameter('rotation_step_deg',       90.0)
        self.declare_parameter('max_rotation_attempts',   4)

        self.declare_parameter('require_battery_ok',      True)
        self.declare_parameter('min_battery_voltage',     11.1)
        self.declare_parameter('warn_battery_voltage',    11.4)
        self.declare_parameter('battery_stale_sec',       3.0)
        self.declare_parameter('debug_decisions',         True)
        self.declare_parameter('debug_period_sec',        1.0)

        p = self.get_parameter
        obs_t   = p('obstacle_topic').value
        cmd_t   = p('cmd_vel_topic').value
        bat_t   = p('battery_topic').value
        state_t = p('state_topic').value

        self._stamped  = _as_bool(p('cmd_vel_stamped').value)
        self._frame    = str(p('cmd_vel_frame_id').value)
        hz             = float(p('control_hz').value)

        self._max_spd  = float(p('max_speed').value)
        self._min_spd  = float(p('min_speed').value)
        self._rot_spd  = float(p('rotation_angular_speed').value)

        self._clear    = float(p('clear_distance').value)
        self._stop     = float(p('stop_distance').value)
        self._dodge_cl = float(p('dodge_clearance').value)
        self._rear_stp = float(p('rear_stop_distance').value)

        self._obs_n    = max(1, int(p('observe_frames').value))
        self._bk_spd   = float(p('backup_speed').value)
        self._bk_sec   = float(p('backup_sec').value)

        rot_rad        = math.radians(float(p('rotation_step_deg').value))
        self._rot_sec  = rot_rad / max(self._rot_spd, 0.01)
        self._max_rot  = int(p('max_rotation_attempts').value)

        self._req_bat  = _as_bool(p('require_battery_ok').value)
        self._min_bat  = float(p('min_battery_voltage').value)
        self._warn_bat = float(p('warn_battery_voltage').value)
        self._bat_stl  = float(p('battery_stale_sec').value)
        self._debug    = _as_bool(p('debug_decisions').value)
        self._dbg_per  = float(p('debug_period_sec').value)

        self._state      = STOPPED
        self._state_end  = 0.0
        self._turn_dir   = 1.0
        self._rot_tries  = 0
        self._obs_count  = 0
        self._bat_v      = None
        self._bat_t      = None
        self._bat_warn_t = 0.0
        self._raw_obs    = None
        self._raw_t      = None
        self._dbg_t      = 0.0
        self._dbg_trans  = None

        vel_type = TwistStamped if self._stamped else Twist
        self.create_subscription(String,       obs_t, self._on_obs, 10)
        self.create_subscription(BatteryState, bat_t, self._on_bat, 10)
        self._vel_pub   = self.create_publisher(vel_type, cmd_t,   10)
        self._state_pub = self.create_publisher(String,   state_t, 10)
        self.create_timer(1.0 / max(hz, 1.0), self._loop)

        self.get_logger().info(
            f'Obstacle avoidance ready: '
            f'clear={self._clear*100:.0f}cm stop={self._stop*100:.0f}cm '
            f'obs_frames={self._obs_n} rot_step={math.degrees(rot_rad):.0f}deg/{self._rot_sec:.1f}s'
        )

    # ── Subscribers ──────────────────────────────────────────────────────────

    def _on_obs(self, msg: String):
        try:
            self._raw_obs = json.loads(msg.data)
            self._raw_t   = time.monotonic()
        except json.JSONDecodeError:
            pass

    def _on_bat(self, msg: BatteryState):
        if math.isfinite(msg.voltage) and msg.voltage > 0.0:
            self._bat_v = float(msg.voltage)
            self._bat_t = time.monotonic()

    # ── Main loop ─────────────────────────────────────────────────────────────

    def _loop(self):
        twist = Twist()
        now   = time.monotonic()

        if self._bat_blocked(now):
            self._bat_log(now)
            self._set(STOPPED)
            self._pub(twist)
            return
        self._bat_log(now)

        if self._raw_obs is None or self._raw_t is None or now - self._raw_t > 1.0:
            self._set(STOPPED)
            self._log('no_data')
            self._pub(twist)
            return

        snap  = self._snap()
        front = self._front(snap)

        if snap.tof_emergency:
            self._set(STOPPED)
            self._log('tof_emergency', snap)
            self._pub(twist)
            return

        # Continue timed / counted states
        if self._state == BACKUP and self._do_backup(twist, snap, now):
            self._log('backup', snap)
            self._pub(twist)
            return

        if self._state == OBSERVE and self._do_observe(twist, snap, front):
            self._log('observe', snap)
            self._pub(twist)
            return

        if self._state == ROTATE and self._do_rotate(twist, snap, front, now):
            self._log('rotate', snap)
            self._pub(twist)
            return

        # 1. Confirmed too close → backup immediately
        if self._too_close(snap):
            self._start_backup()
            self._do_backup(twist, snap, now)
            self._log('too_close_backup', snap)
            self._pub(twist)
            return

        # 2. Dead-end: front blocked + both sides tight
        if self._dead_end(snap, front):
            self._start_backup()
            self._do_backup(twist, snap, now)
            self._log('dead_end_backup', snap)
            self._pub(twist)
            return

        # 3. Object in observe band → watch before deciding
        if front <= self._clear:
            self._start_observe()
            self._do_observe(twist, snap, front)
            self._log('observe_start', snap)
            self._pub(twist)
            return

        # 4. Front clear → drive straight, ignore sides
        self._set(DRIVE)
        twist.linear.x  = self._max_spd
        twist.angular.z = 0.0
        self._log('drive', snap)
        self._pub(twist)

    # ── State handlers ────────────────────────────────────────────────────────

    def _start_backup(self):
        self._rot_tries = 0
        self._set(BACKUP, self._bk_sec)

    def _do_backup(self, twist: Twist, snap: Snap, now: float) -> bool:
        rear_blocked = math.isfinite(snap.rear) and snap.rear < self._rear_stp
        if not rear_blocked and now < self._state_end:
            twist.linear.x  = -abs(self._bk_spd)
            twist.angular.z = 0.0
            return True
        self._start_rotate(snap)
        return True

    def _start_observe(self):
        if self._state != OBSERVE:
            self._obs_count = 0
            self._set(OBSERVE)

    def _do_observe(self, twist: Twist, snap: Snap, front: float) -> bool:
        self._obs_count += 1

        if self._too_close(snap):
            self._start_backup()
            return True

        if front > self._clear:
            self._set(DRIVE)
            return False

        if self._obs_count < self._obs_n:
            twist.linear.x  = 0.0
            twist.angular.z = 0.0
            return True

        # After N frames: if depth still shows clear while LIDAR blocked → false positive, drive
        if snap.depth_ok and math.isfinite(snap.front_depth) and snap.front_depth >= self._clear:
            self._set(DRIVE)
            return False

        self._start_rotate(snap)
        return True

    def _start_rotate(self, snap: Snap):
        if self._state != ROTATE:
            self._rot_tries = 0
            self._turn_dir  = self._clearer_side(snap.left, snap.right)
        self._set(ROTATE, self._rot_sec)

    def _do_rotate(self, twist: Twist, snap: Snap, front: float, now: float) -> bool:
        if front >= self._clear:
            self._rot_tries = 0
            self._set(DRIVE)
            return False

        if now < self._state_end:
            twist.linear.x  = 0.0
            twist.angular.z = self._turn_dir * self._rot_spd
            return True

        self._rot_tries += 1
        if self._rot_tries < self._max_rot:
            self._state_end = now + self._rot_sec
            twist.angular.z = self._turn_dir * self._rot_spd
            self.get_logger().info(
                f'Rotate sweep {self._rot_tries}/{self._max_rot} '
                f'{"left" if self._turn_dir > 0 else "right"} '
                f'front={_cm(front)} left={_cm(snap.left)} right={_cm(snap.right)}'
            )
            return True

        self.get_logger().warn('Rotation exhausted — backing up')
        self._start_backup()
        return True

    # ── Predicates ────────────────────────────────────────────────────────────

    def _front(self, snap: Snap) -> float:
        """Effective front distance for FSM.

        If LIDAR is blocked but depth clearly shows open space, trust depth to
        avoid false positives from side reflections / transient objects.
        After observing N frames we double-check depth before acting.
        """
        lf = snap.front_lidar
        df = snap.front_depth
        if snap.depth_ok and math.isfinite(df):
            if math.isfinite(lf):
                return min(lf, df)
            return df
        return lf if math.isfinite(lf) else math.inf

    def _too_close(self, snap: Snap) -> bool:
        """Backup only when danger is confirmed — depth clear means no backup."""
        if snap.depth_ok and math.isfinite(snap.front_depth):
            if snap.front_depth >= self._clear:
                return False  # depth says open → trust it
            return snap.front_depth <= self._stop
        return math.isfinite(snap.front_lidar) and snap.front_lidar <= self._stop

    def _dead_end(self, snap: Snap, front: float) -> bool:
        front_blocked = front <= self._clear
        left_blocked  = math.isfinite(snap.left)  and snap.left  < self._dodge_cl
        right_blocked = math.isfinite(snap.right) and snap.right < self._dodge_cl
        return front_blocked and left_blocked and right_blocked

    @staticmethod
    def _clearer_side(left: float, right: float) -> float:
        """Return +1.0 (CCW/left) or -1.0 (CW/right): rotate toward more space."""
        lr = left  if math.isfinite(left)  else math.inf
        rr = right if math.isfinite(right) else math.inf
        return 1.0 if lr >= rr else -1.0

    # ── Snapshot ──────────────────────────────────────────────────────────────

    def _snap(self) -> Snap:
        data  = self._raw_obs or {}
        fused = data.get('fused', {})
        lidar = data.get('lidar', {})
        depth = data.get('depth', {})

        lidar_ok = bool(lidar.get('available', False))
        depth_ok = bool(depth.get('available', False))

        fl = _finite(lidar.get('front_control'))
        fd = _finite(depth.get('front_min'))

        source  = list(fused.get('source', []))
        dynamic = bool(
            fused.get('dynamic_obstacle', False)
            or (not lidar_ok and depth_ok and 'depth' in source)
        )
        emergency = bool(fused.get('emergency', False))

        return Snap(
            front_lidar   = fl,
            front_depth   = fd,
            left          = _finite(fused.get('left_distance')),
            right         = _finite(fused.get('right_distance')),
            rear          = _finite(fused.get('rear_distance')),
            dynamic       = dynamic,
            tof_emergency = bool(emergency and 'tof' in source),
            lidar_ok      = lidar_ok,
            depth_ok      = depth_ok,
        )

    # ── FSM / publish helpers ─────────────────────────────────────────────────

    def _set(self, state: str, duration: float = 0.0):
        if self._state != state:
            self.get_logger().info(f'FSM: {self._state} -> {state}')
            msg      = String()
            msg.data = f'{time.time():.3f},{state}'
            self._state_pub.publish(msg)
            self._dbg_trans = f'{self._state}->{state}'
            self._state     = state
        self._state_end = time.monotonic() + duration

    def _pub(self, twist: Twist):
        if not self._stamped:
            self._vel_pub.publish(twist)
            return
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        msg.twist = twist
        self._vel_pub.publish(msg)

    # ── Logging ───────────────────────────────────────────────────────────────

    def _log(self, reason: str, snap: Snap | None = None):
        if not self._debug:
            return
        now   = time.monotonic()
        trans = self._dbg_trans
        if trans is None and now - self._dbg_t < self._dbg_per:
            return
        self._dbg_t     = now
        self._dbg_trans = None

        if snap is None:
            self.get_logger().info(f'[NAV] state={self._state} reason={reason}')
            return

        front_eff = self._front(snap)
        msg = (
            f'[NAV] state={self._state} '
            f'front_lidar={_cm(snap.front_lidar)} '
            f'front_depth={_cm(snap.front_depth)} '
            f'front_eff={_cm(front_eff)} '
            f'left={_cm(snap.left)} right={_cm(snap.right)} '
            f'turn={"L" if self._turn_dir > 0 else "R"} '
            f'reason={reason} obs={self._obs_count}'
        )
        if self._state in (DRIVE, STOPPED):
            self.get_logger().info(msg)
        else:
            self.get_logger().warn(msg)

    # ── Battery ───────────────────────────────────────────────────────────────

    def _bat_blocked(self, now: float) -> bool:
        if self._bat_v is None or self._bat_t is None:
            return self._req_bat
        if now - self._bat_t > self._bat_stl:
            return self._req_bat
        return self._bat_v < self._min_bat

    def _bat_log(self, now: float):
        if now - self._bat_warn_t < 5.0:
            return
        if self._bat_v is None:
            if self._req_bat:
                self._bat_warn_t = now
                self.get_logger().error('[BAT] no reading — robot stopped')
            return
        if self._bat_v < self._min_bat:
            self._bat_warn_t = now
            self.get_logger().error(
                f'[BAT] {self._bat_v:.2f}V < min {self._min_bat:.2f}V — stopped'
            )
        elif self._bat_v < self._warn_bat:
            self._bat_warn_t = now
            self.get_logger().warn(
                f'[BAT] {self._bat_v:.2f}V < warn {self._warn_bat:.2f}V — test gently'
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
