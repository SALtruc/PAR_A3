"""Timed traffic light state publisher for Project B dry tests."""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_SCRIPT = '1.0:RED,4.0:GREEN,8.0:YELLOW,11.0:RED'


class TrafficLightSimNode(Node):

    def __init__(self):
        super().__init__('traffic_light_sim')
        self.declare_parameter('state_topic', '/traffic_light_state')
        self.declare_parameter('script', DEFAULT_SCRIPT)
        self.declare_parameter('stop_after_sec', 14.0)

        self._pub = self.create_publisher(
            String,
            self.get_parameter('state_topic').value,
            10,
        )
        self._events = self._parse_script(self.get_parameter('script').value)
        self._stop_after_sec = float(self.get_parameter('stop_after_sec').value)
        self._start_time = time.monotonic()
        self._next_event = 0
        self.create_timer(0.05, self._tick)

    @staticmethod
    def _parse_script(script: str) -> list[tuple[float, str]]:
        events = []
        for item in script.split(','):
            item = item.strip()
            if not item:
                continue
            delay, state = item.split(':', 1)
            events.append((float(delay), state.strip().upper()))
        return sorted(events, key=lambda event: event[0])

    def _tick(self):
        elapsed = time.monotonic() - self._start_time
        while self._next_event < len(self._events):
            delay, state = self._events[self._next_event]
            if elapsed < delay:
                break
            msg = String()
            msg.data = f'{time.time():.3f},{state},1.000,-1,-1,0.0'
            self._pub.publish(msg)
            self.get_logger().info(f'TRAFFIC_LIGHT -> {state}')
            self._next_event += 1

        if elapsed >= self._stop_after_sec:
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
