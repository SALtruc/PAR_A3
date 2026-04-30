"""
Project B traffic light controller.

Maps stable traffic light states to robot motion:
  RED/UNKNOWN -> stop
  YELLOW      -> slow crawl
  GREEN       -> drive forward
"""

import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from std_msgs.msg import String


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class TrafficLightControllerNode(Node):

    def __init__(self):
        super().__init__('traffic_light_controller')

        self.declare_parameter('state_topic', '/traffic_light_state')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_stamped', True)
        self.declare_parameter('cmd_vel_frame_id', 'base_link')
        self.declare_parameter('green_speed', 0.18)
        self.declare_parameter('yellow_speed', 0.06)
        self.declare_parameter('state_timeout_sec', 1.0)

        state_topic = self.get_parameter('state_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self._cmd_vel_stamped = _as_bool(self.get_parameter('cmd_vel_stamped').value)
        self._cmd_vel_frame_id = self.get_parameter('cmd_vel_frame_id').value
        self._green_speed = float(self.get_parameter('green_speed').value)
        self._yellow_speed = float(self.get_parameter('yellow_speed').value)
        self._state_timeout_sec = float(self.get_parameter('state_timeout_sec').value)

        self._state = 'UNKNOWN'
        self._last_state_time = 0.0

        vel_type = TwistStamped if self._cmd_vel_stamped else Twist
        self._vel_pub = self.create_publisher(vel_type, cmd_vel_topic, 10)
        self.create_subscription(String, state_topic, self._on_state, 10)
        self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f'Traffic light controller ready. state={state_topic}, '
            f'cmd_vel={cmd_vel_topic} ({vel_type.__name__})'
        )

    def _on_state(self, msg: String):
        parts = msg.data.split(',')
        state = parts[1].strip().upper() if len(parts) >= 2 else msg.data.strip().upper()
        if state not in {'RED', 'YELLOW', 'GREEN', 'UNKNOWN'}:
            self.get_logger().warn(f'Ignoring invalid traffic light state: {msg.data!r}')
            return

        if state != self._state:
            self.get_logger().info(f'Traffic light transition: {self._state} -> {state}')
        self._state = state
        self._last_state_time = time.monotonic()

    def _control_loop(self):
        state = self._state
        if time.monotonic() - self._last_state_time > self._state_timeout_sec:
            state = 'UNKNOWN'

        twist = Twist()
        if state == 'GREEN':
            twist.linear.x = self._green_speed
        elif state == 'YELLOW':
            twist.linear.x = self._yellow_speed
        else:
            twist.linear.x = 0.0
        self._publish_velocity(twist)

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
    node = TrafficLightControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
