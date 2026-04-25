"""
Event Logger Node
-----------------
Subscribes to /qr_event (produced by CommandInterpreter and FSM) and writes
every event to a timestamped CSV file for offline evaluation.

CSV columns:
    wall_clock_iso  – ISO-8601 timestamp
    ros_time        – float seconds embedded in event string
    event_type      – COMMAND | STATE | DETECTION
    value           – the command name or state name

The log directory is ~/rosbot_qr_logs/ by default (configurable via param).
A new file is created each time the node starts: qr_events_<YYYYMMDD_HHMMSS>.csv
"""

import csv
import os
import time
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class EventLoggerNode(Node):

    def __init__(self):
        super().__init__('event_logger')

        self.declare_parameter(
            'log_dir',
            os.path.expanduser('~/rosbot_qr_logs')
        )
        log_dir = os.path.expanduser(self.get_parameter('log_dir').value)
        os.makedirs(log_dir, exist_ok=True)

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_path = os.path.join(log_dir, f'qr_events_{stamp}.csv')

        self._file = open(log_path, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(['wall_clock_iso', 'ros_time', 'event_type', 'value'])
        self._file.flush()

        self.sub = self.create_subscription(
            String, '/qr_event', self._on_event, 10
        )

        self.get_logger().info(f'Event Logger writing to: {log_path}')

    # ------------------------------------------------------------------
    def _on_event(self, msg: String):
        """
        Expected format: '<ros_time>,<event_type>,<value>'
        e.g.  '1745600000.123,COMMAND,TURN_LEFT'
        """
        parts = msg.data.split(',', 2)
        if len(parts) != 3:
            self.get_logger().warn(f'Malformed event string: {msg.data!r}')
            return

        ros_time, event_type, value = parts
        wall_clock = datetime.now(timezone.utc).isoformat()

        self._writer.writerow([wall_clock, ros_time, event_type, value])
        self._file.flush()

    # ------------------------------------------------------------------
    def destroy_node(self):
        self._file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EventLoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
