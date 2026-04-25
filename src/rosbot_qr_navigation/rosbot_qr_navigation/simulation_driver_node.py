"""
Simulation Driver Node
----------------------
Publishes a timed sequence of fake QR detections and prints the resulting
FSM state, velocity commands, and event log messages.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


DEFAULT_SCRIPT = (
    '1.0:GO,'
    '3.0:SPEED_UP,'
    '5.0:TURN_LEFT,'
    '10.0:STOP,'
    '13.0:GO,'
    '16.0:TURN_RIGHT,'
    '21.0:U_TURN,'
    '29.0:SPEED_DOWN'
)


class SimulationDriverNode(Node):

    def __init__(self):
        super().__init__('simulation_driver')

        self.declare_parameter('script', DEFAULT_SCRIPT)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('stop_after_sec', 35.0)
        self.declare_parameter('print_cmd_vel_hz', 2.0)
        self.declare_parameter('obstacle_start_sec', -1.0)
        self.declare_parameter('obstacle_end_sec', -1.0)
        self.declare_parameter('fake_obstacle_distance', 0.30)

        self._events = self._parse_script(self.get_parameter('script').value)
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        self._stop_after_sec = float(self.get_parameter('stop_after_sec').value)
        print_hz = float(self.get_parameter('print_cmd_vel_hz').value)
        self._obstacle_start_sec = float(self.get_parameter('obstacle_start_sec').value)
        self._obstacle_end_sec = float(self.get_parameter('obstacle_end_sec').value)
        self._fake_obstacle_distance = float(
            self.get_parameter('fake_obstacle_distance').value
        )

        self._start_time = time.monotonic()
        self._next_event = 0
        self._last_twist_print = 0.0
        self._twist_print_period = 1.0 / max(print_hz, 0.1)

        self._qr_pub = self.create_publisher(String, '/qr_detected', 10)
        self._scan_pub = self.create_publisher(LaserScan, scan_topic, 10)
        self.create_subscription(String, '/fsm_state', self._on_state, 10)
        self.create_subscription(String, '/qr_event', self._on_event, 10)
        self.create_subscription(Twist, cmd_vel_topic, self._on_twist, 10)

        self.create_timer(0.05, self._tick)

        pretty_script = ', '.join(f'{delay:.1f}s:{cmd}' for delay, cmd in self._events)
        self.get_logger().info(f'Simulation script: {pretty_script}')
        self.get_logger().info(
            f'Watching velocity topic: {cmd_vel_topic}; publishing fake scan: {scan_topic}'
        )

    @staticmethod
    def _parse_script(script: str) -> list[tuple[float, str]]:
        events = []
        for item in script.split(','):
            item = item.strip()
            if not item:
                continue

            delay_text, command = item.split(':', 1)
            events.append((float(delay_text), command.strip().upper()))

        return sorted(events, key=lambda event: event[0])

    def _tick(self):
        elapsed = time.monotonic() - self._start_time
        self._publish_scan(elapsed)

        while self._next_event < len(self._events):
            delay, command = self._events[self._next_event]
            if elapsed < delay:
                break

            msg = String()
            msg.data = command
            self._qr_pub.publish(msg)
            self.get_logger().info(f'FAKE QR -> {command}')
            self._next_event += 1

        if elapsed >= self._stop_after_sec:
            self.get_logger().info('Simulation complete.')
            rclpy.shutdown()

    def _publish_scan(self, elapsed: float):
        obstacle_active = (
            self._obstacle_start_sec >= 0.0
            and self._obstacle_start_sec <= elapsed <= self._obstacle_end_sec
        )

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'sim_laser'
        scan.angle_min = -math.pi / 2
        scan.angle_max = math.pi / 2
        scan.angle_increment = math.pi / 180
        scan.range_min = 0.05
        scan.range_max = 5.0
        scan.ranges = [2.0] * 181

        if obstacle_active:
            center = len(scan.ranges) // 2
            for i in range(center - 10, center + 11):
                scan.ranges[i] = self._fake_obstacle_distance

        self._scan_pub.publish(scan)

    def _on_state(self, msg: String):
        self.get_logger().info(f'FSM STATE <- {msg.data}')

    def _on_event(self, msg: String):
        self.get_logger().info(f'EVENT <- {msg.data}')

    def _on_twist(self, msg: Twist):
        now = time.monotonic()
        if now - self._last_twist_print < self._twist_print_period:
            return

        self._last_twist_print = now
        self.get_logger().info(
            f'CMD_VEL <- linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimulationDriverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
