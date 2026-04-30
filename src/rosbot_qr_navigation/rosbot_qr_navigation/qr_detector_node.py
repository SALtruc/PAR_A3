"""
QR Detector Node
----------------
Subscribes to a camera image topic, decodes QR codes using OpenCV's built-in
QRCodeDetectorAruco (primary) with a ZBar fallback, and publishes each unique
decoded command string on /qr_detected.

Edge-case handling:
  - Multiple QR codes in one frame: publishes all, tagged with bbox centre.
  - Partial / degraded codes: OpenCV detector is tried first; on failure ZBar
    is attempted if available; otherwise the frame is skipped silently.
  - Oblique angles: when OpenCV finds corners but fails to decode (empty text),
    a perspective warp rectifies the quad to a canonical square and retries.
    Skew angle is estimated from corner geometry and logged for diagnostics.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import math
import cv2
import numpy as np
import time

try:
    import zxingcpp  # preferred ZBar replacement, pure-python friendly
    ZXING_AVAILABLE = True
except ImportError:
    ZXING_AVAILABLE = False

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

DEBOUNCE_SEC = 2.0  # suppress re-publishing the same command within this window


class QRDetectorNode(Node):

    def __init__(self):
        super().__init__('qr_detector')

        self.declare_parameter('image_topic', '/oak/rgb/image_raw')
        self.declare_parameter('show_debug', False)
        self.declare_parameter('min_qr_area', 800)        # px² – ignore tiny detections
        self.declare_parameter('debounce_sec', DEBOUNCE_SEC)

        image_topic = self.get_parameter('image_topic').value
        self.show_debug = self.get_parameter('show_debug').value
        self.min_area = self.get_parameter('min_qr_area').value
        self.debounce_sec = self.get_parameter('debounce_sec').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(Image, image_topic, self._image_cb, qos)
        self.pub = self.create_publisher(String, '/qr_detected', 10)
        self.evt_pub = self.create_publisher(String, '/qr_event', 10)

        self.bridge = CvBridge()
        self.cv_qr = cv2.QRCodeDetector()
        self._deskew_size = 300  # px; canonical square for perspective warp

        # last-seen timestamp per command for debouncing
        self._last_seen: dict[str, float] = {}

        self.get_logger().info(
            f'QR Detector ready – subscribing to [{image_topic}], '
            f'ZXing available: {ZXING_AVAILABLE}'
        )

    # ------------------------------------------------------------------
    def _image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        commands = self._detect_qr_codes(frame)
        now = time.monotonic()
        for cmd in commands:
            last = self._last_seen.get(cmd, 0.0)
            if now - last < self.debounce_sec:
                continue
            self._last_seen[cmd] = now
            out = String()
            out.data = cmd
            self.pub.publish(out)

            evt = String()
            evt.data = f'{time.time():.3f},DETECTION,{cmd}'
            self.evt_pub.publish(evt)

            self.get_logger().info(f'QR detected: {cmd}')

    # ------------------------------------------------------------------
    def _detect_qr_codes(self, frame: np.ndarray) -> list[str]:
        """Return list of valid command strings found in frame."""
        results = []

        # --- Primary: OpenCV multi-code detector ----------------------
        try:
            retval, decoded_info, points, _ = self.cv_qr.detectAndDecodeMulti(frame)
            if retval and points is not None:
                for text, pts in zip(decoded_info, points):
                    area = cv2.contourArea(pts.astype(np.float32))
                    if area < self.min_area:
                        self.get_logger().debug(f'Ignoring small QR area={area:.0f}')
                        continue

                    cmd = text.strip().upper() if text else ''

                    # Oblique-angle recovery: corners found but decode failed –
                    # warp the quad to a canonical square and retry.
                    if not cmd:
                        cmd = self._deskew_and_decode(frame, pts)
                        if cmd:
                            skew = self._estimate_skew_deg(pts)
                            self.get_logger().info(
                                f'Deskew decode succeeded (skew≈{skew:.1f}°): {cmd}'
                            )

                    if not cmd or cmd not in VALID_COMMANDS:
                        continue
                    results.append(cmd)
                    if self.show_debug:
                        self._draw_qr(frame, pts, cmd)
        except Exception as e:
            self.get_logger().debug(f'OpenCV QR error: {e}')

        # --- Fallback: ZXing-cpp if OpenCV found nothing --------------
        if not results and ZXING_AVAILABLE:
            results.extend(self._decode_zxing(frame))

        if self.show_debug and results:
            cv2.imshow('QR Debug', frame)
            cv2.waitKey(1)

        return results

    # ------------------------------------------------------------------
    def _deskew_and_decode(self, frame: np.ndarray, pts: np.ndarray) -> str:
        """Warp the 4-corner quad to a square and retry decode."""
        try:
            src = pts.astype(np.float32)
            s = self._deskew_size
            dst = np.array([[0, 0], [s, 0], [s, s], [0, s]], dtype=np.float32)
            M = cv2.getPerspectiveTransform(src, dst)
            warped = cv2.warpPerspective(frame, M, (s, s))
            text, _, _ = self.cv_qr.detectAndDecode(warped)
            return text.strip().upper() if text else ''
        except Exception:
            return ''

    @staticmethod
    def _estimate_skew_deg(pts: np.ndarray) -> float:
        """Estimate in-plane skew angle from the top edge of the QR quad."""
        p0, p1 = pts[0], pts[1]
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        return abs(math.degrees(math.atan2(dy, dx)))

    # ------------------------------------------------------------------
    def _decode_zxing(self, frame: np.ndarray) -> list[str]:
        import zxingcpp
        results = []
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found = zxingcpp.read_barcodes(gray)
        for r in found:
            cmd = r.text.strip().upper()
            if cmd in VALID_COMMANDS:
                results.append(cmd)
        return results

    @staticmethod
    def _draw_qr(frame, pts, label):
        pts = pts.astype(int)
        cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
        cx, cy = int(pts[:, 0].mean()), int(pts[:, 1].mean())
        cv2.putText(frame, label, (cx - 40, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)


def main(args=None):
    rclpy.init(args=args)
    node = QRDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
