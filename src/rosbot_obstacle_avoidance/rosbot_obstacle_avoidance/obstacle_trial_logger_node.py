"""CSV trial logger for Project C evaluation metrics."""

import csv
import json
import math
import os
import time
from datetime import datetime

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


RESPONSE_STATES = {'OBSERVE', 'DODGE', 'ROTATE', 'BACKUP', 'EMERGENCY'}


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def _finite_or_none(value):
    if value is None:
        return None
    try:
        value_f = float(value)
    except (TypeError, ValueError):
        return None
    return value_f if math.isfinite(value_f) else None


class ObstacleTrialLoggerNode(Node):

    def __init__(self):
        super().__init__('obstacle_trial_logger')

        self.declare_parameter('obstacle_topic', '/obstacle_representation')
        self.declare_parameter('state_topic', '/obstacle_avoidance_state')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('collision_topic', '/collision_event')
        self.declare_parameter('summary_topic', '/obstacle_trial_summary')
        self.declare_parameter('log_dir', '~/rosbot_obstacle_logs')
        self.declare_parameter('summary_period_sec', 5.0)
        self.declare_parameter('log_obstacle_samples', True)

        obstacle_topic = self.get_parameter('obstacle_topic').value
        state_topic = self.get_parameter('state_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        collision_topic = self.get_parameter('collision_topic').value
        summary_topic = self.get_parameter('summary_topic').value
        summary_period = float(self.get_parameter('summary_period_sec').value)
        self._log_obstacle_samples = _as_bool(
            self.get_parameter('log_obstacle_samples').value
        )

        log_dir = os.path.expanduser(self.get_parameter('log_dir').value)
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._path = os.path.join(log_dir, f'project_c_trial_{stamp}.csv')
        self._csv_file = open(self._path, 'w', newline='', encoding='utf-8')
        self._writer = csv.DictWriter(
            self._csv_file,
            fieldnames=[
                'stamp',
                'event',
                'value',
                'state',
                'front_distance',
                'left_distance',
                'right_distance',
                'best_gap_angle',
                'dynamic_obstacle',
                'emergency',
                'dead_end',
                'source',
                'depth_available',
                'depth_image_available',
                'depth_image_front_min',
                'depth_image_low_front_min',
                'depth_motion',
                'depth_motion_score',
                'pointcloud_available',
                'pointcloud_front_min',
                'pointcloud_low_front_min',
                'pointcloud_low_front_count',
                'pointcloud_sample_count',
                'path_length_m',
                'coverage_area_m2',
                'collision_count',
                'collision_rate_per_min',
                'dead_end_count',
                'recovery_success_count',
                'recovery_success_rate',
                'dynamic_latency_s',
            ],
        )
        self._writer.writeheader()

        self._state = 'UNKNOWN'
        self._latest_fused = {}
        self._latest_depth = {}
        self._latest_source = []
        self._trial_start_time = time.time()
        self._collision_count = 0
        self._dead_end_count = 0
        self._recovery_success_count = 0
        self._recovery_active = False
        self._dynamic_active = False
        self._dynamic_seen_time: float | None = None
        self._latencies: list[float] = []
        self._prev_odom_xy: tuple[float, float] | None = None
        self._path_length = 0.0
        self._min_x: float | None = None
        self._max_x: float | None = None
        self._min_y: float | None = None
        self._max_y: float | None = None

        self.create_subscription(String, obstacle_topic, self._on_obstacles, 10)
        self.create_subscription(String, state_topic, self._on_state, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(String, collision_topic, self._on_collision, 10)
        self._summary_pub = self.create_publisher(String, summary_topic, 10)
        self.create_timer(max(summary_period, 1.0), self._publish_summary)

        self.get_logger().info(f'Project C trial logger writing to: {self._path}')

    def _on_obstacles(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        fused = data.get('fused', {})
        depth = data.get('depth', {})
        self._latest_fused = fused
        self._latest_depth = depth
        self._latest_source = list(fused.get('source', []))

        dynamic = bool(fused.get('dynamic_obstacle', False))
        dead_end = bool(fused.get('dead_end', False))
        emergency = bool(fused.get('emergency', False))
        now = time.time()

        if dynamic and not self._dynamic_active:
            self._dynamic_active = True
            self._dynamic_seen_time = now
            self._write_row('DYNAMIC_SEEN')
        elif not dynamic:
            self._dynamic_active = False
            if (
                    self._dynamic_seen_time is not None
                    and now - self._dynamic_seen_time > 3.0):
                self._dynamic_seen_time = None

        if dead_end and not self._recovery_active:
            self._dead_end_count += 1
            self._recovery_active = True
            self._write_row('DEAD_END')

        if emergency:
            self._write_row('EMERGENCY')

        if self._log_obstacle_samples:
            self._write_row('SAMPLE')

    def _on_state(self, msg: String):
        parts = msg.data.split(',', 1)
        state = parts[1] if len(parts) == 2 else msg.data
        self._state = state

        latency = None
        if (
                self._dynamic_seen_time is not None
                and state in RESPONSE_STATES):
            latency = time.time() - self._dynamic_seen_time
            self._latencies.append(latency)
            self._dynamic_seen_time = None

        if self._recovery_active and state == 'DRIVE':
            self._recovery_success_count += 1
            self._recovery_active = False

        self._write_row('STATE', value=state, dynamic_latency=latency)

    def _on_collision(self, msg: String):
        self._collision_count += 1
        self._write_row('COLLISION', value=msg.data)

    def _on_odom(self, msg: Odometry):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if self._prev_odom_xy is not None:
            px, py = self._prev_odom_xy
            self._path_length += math.hypot(x - px, y - py)
        self._prev_odom_xy = (x, y)

        self._min_x = x if self._min_x is None else min(self._min_x, x)
        self._max_x = x if self._max_x is None else max(self._max_x, x)
        self._min_y = y if self._min_y is None else min(self._min_y, y)
        self._max_y = y if self._max_y is None else max(self._max_y, y)

    def _publish_summary(self):
        msg = String()
        msg.data = json.dumps(self._summary(), separators=(',', ':'))
        self._summary_pub.publish(msg)
        self._write_row('SUMMARY')

    def _summary(self) -> dict:
        elapsed_min = max((time.time() - self._trial_start_time) / 60.0, 1e-6)
        recovery_rate = (
            self._recovery_success_count / self._dead_end_count
            if self._dead_end_count
            else None
        )
        mean_latency = (
            sum(self._latencies) / len(self._latencies)
            if self._latencies
            else None
        )
        return {
            'elapsed_sec': time.time() - self._trial_start_time,
            'collision_count': self._collision_count,
            'collision_rate_per_min': self._collision_count / elapsed_min,
            'dead_end_count': self._dead_end_count,
            'recovery_success_count': self._recovery_success_count,
            'recovery_success_rate': recovery_rate,
            'dynamic_response_count': len(self._latencies),
            'mean_dynamic_latency_s': mean_latency,
            'path_length_m': self._path_length,
            'coverage_area_m2': self._coverage_area(),
            'log_path': self._path,
        }

    def _coverage_area(self) -> float:
        if None in (self._min_x, self._max_x, self._min_y, self._max_y):
            return 0.0
        return max(0.0, (self._max_x - self._min_x) * (self._max_y - self._min_y))

    def _write_row(self, event: str, value: str = '', dynamic_latency=None):
        fused = self._latest_fused
        depth = self._latest_depth
        summary = self._summary()
        dead_end_count = self._dead_end_count
        recovery_rate = (
            self._recovery_success_count / dead_end_count
            if dead_end_count
            else None
        )
        row = {
            'stamp': f'{time.time():.3f}',
            'event': event,
            'value': value,
            'state': self._state,
            'front_distance': _finite_or_none(fused.get('front_distance')),
            'left_distance': _finite_or_none(fused.get('left_distance')),
            'right_distance': _finite_or_none(fused.get('right_distance')),
            'best_gap_angle': _finite_or_none(fused.get('best_gap_angle')),
            'dynamic_obstacle': bool(fused.get('dynamic_obstacle', False)),
            'emergency': bool(fused.get('emergency', False)),
            'dead_end': bool(fused.get('dead_end', False)),
            'source': '+'.join(self._latest_source),
            'depth_available': bool(depth.get('available', False)),
            'depth_image_available': bool(depth.get('image_available', False)),
            'depth_image_front_min': _finite_or_none(depth.get('image_front_min')),
            'depth_image_low_front_min': _finite_or_none(
                depth.get('image_low_front_min')
            ),
            'depth_motion': bool(depth.get('motion', False)),
            'depth_motion_score': _finite_or_none(depth.get('motion_score')),
            'pointcloud_available': bool(depth.get('pointcloud_available', False)),
            'pointcloud_front_min': _finite_or_none(
                depth.get('pointcloud_front_min')
            ),
            'pointcloud_low_front_min': _finite_or_none(
                depth.get('pointcloud_low_front_min')
            ),
            'pointcloud_low_front_count': int(
                depth.get('pointcloud_low_front_count', 0) or 0
            ),
            'pointcloud_sample_count': int(
                depth.get('pointcloud_sample_count', 0) or 0
            ),
            'path_length_m': f'{self._path_length:.3f}',
            'coverage_area_m2': f'{self._coverage_area():.3f}',
            'collision_count': self._collision_count,
            'collision_rate_per_min': f'{summary["collision_rate_per_min"]:.3f}',
            'dead_end_count': dead_end_count,
            'recovery_success_count': self._recovery_success_count,
            'recovery_success_rate': (
                f'{recovery_rate:.3f}' if recovery_rate is not None else ''
            ),
            'dynamic_latency_s': (
                f'{dynamic_latency:.3f}' if dynamic_latency is not None else ''
            ),
        }
        self._writer.writerow(row)
        self._csv_file.flush()

    def destroy_node(self):
        try:
            self._csv_file.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleTrialLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
