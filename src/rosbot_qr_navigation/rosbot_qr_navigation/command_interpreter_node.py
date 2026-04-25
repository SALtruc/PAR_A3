"""
Command Interpreter Node
------------------------
Sits between QR Detector and Navigation FSM.

Responsibilities:
  1. Receive raw command strings from /qr_detected.
  2. Apply priority rules when two commands arrive simultaneously.
  3. Validate commands against the allowed set.
  4. Forward the resolved command to /qr_command (consumed by FSM).
  5. Publish executed command events to /qr_event for the logger.

Priority order (highest → lowest):
  STOP > U_TURN > TURN_LEFT / TURN_RIGHT > GO > SPEED_UP / SPEED_DOWN

Simultaneous detection window: if two different commands arrive within
SIMULTANEOUS_WINDOW_SEC, the higher-priority one wins and the lower is
dropped (with a warning log).
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

BASE_COMMANDS = {
    'TURN_LEFT', 'TURN_RIGHT', 'STOP', 'GO',
    'SPEED_UP', 'SPEED_DOWN', 'U_TURN',
}
QUEUED_COMMANDS = {
    'AND_TURN_LEFT',
    'AND_TURN_RIGHT',
    'AND_STOP',
    'AND_GO',
    'AND_SPEED_UP',
    'AND_SPEED_DOWN',
    'AND_U_TURN',
}
VALID_COMMANDS = BASE_COMMANDS | QUEUED_COMMANDS

# Lower number = higher priority
PRIORITY: dict[str, int] = {
    'STOP':       0,
    'U_TURN':     1,
    'TURN_LEFT':  2,
    'TURN_RIGHT': 2,
    'GO':         3,
    'SPEED_UP':   4,
    'SPEED_DOWN': 4,
}
PRIORITY.update({
    'AND_STOP':       10,
    'AND_U_TURN':     11,
    'AND_TURN_LEFT':  12,
    'AND_TURN_RIGHT': 12,
    'AND_GO':         13,
    'AND_SPEED_UP':   14,
    'AND_SPEED_DOWN': 14,
})

SIMULTANEOUS_WINDOW_SEC = 0.3  # commands within this window treated as simultaneous


class CommandInterpreterNode(Node):

    def __init__(self):
        super().__init__('command_interpreter')

        self.declare_parameter('simultaneous_window_sec', SIMULTANEOUS_WINDOW_SEC)
        self._window = self.get_parameter('simultaneous_window_sec').value

        self.sub = self.create_subscription(
            String, '/qr_detected', self._on_qr, 10
        )
        self.cmd_pub = self.create_publisher(String, '/qr_command', 10)
        self.evt_pub = self.create_publisher(String, '/qr_event', 10)

        # Pending buffer: list of (timestamp, command) within current window
        self._pending: list[tuple[float, str]] = []
        self._flush_timer = self.create_timer(
            max(self._window / 2.0, 0.05), self._flush_pending
        )

        self.get_logger().info('Command Interpreter ready.')

    # ------------------------------------------------------------------
    def _on_qr(self, msg: String):
        cmd = msg.data.strip().upper()
        if cmd not in VALID_COMMANDS:
            self.get_logger().warn(f'Unknown command ignored: {cmd!r}')
            return

        now = time.monotonic()
        self._pending.append((now, cmd))
        self.get_logger().debug(f'Buffered: {cmd}')

    # ------------------------------------------------------------------
    def _flush_pending(self):
        if not self._pending:
            return

        now = time.monotonic()
        oldest_ts = min(ts for ts, _ in self._pending)
        if now - oldest_ts < self._window:
            return

        window = self._pending
        self._pending = []

        immediate = [(ts, cmd) for ts, cmd in window if not cmd.startswith('AND_')]
        queued = [(ts, cmd) for ts, cmd in window if cmd.startswith('AND_')]

        if immediate:
            chosen = min(immediate, key=lambda x: PRIORITY[x[1]])[1]
            dropped = [cmd for _, cmd in immediate if cmd != chosen]
            if dropped:
                self.get_logger().warn(
                    f'Simultaneous immediate QR codes detected. Chose [{chosen}] '
                    f'(priority {PRIORITY[chosen]}), dropped {dropped}.'
                )
            self._dispatch(chosen)

        for _, cmd in sorted(queued, key=lambda x: x[0]):
            self._dispatch(cmd)

    # ------------------------------------------------------------------
    def _dispatch(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)

        # Also emit a timestamped event for the logger
        evt = String()
        evt.data = f'{time.time():.3f},COMMAND,{cmd}'
        self.evt_pub.publish(evt)

        self.get_logger().info(f'Dispatched command: {cmd}')


def main(args=None):
    rclpy.init(args=args)
    node = CommandInterpreterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
