"""
Project B traffic light detector.

Detects red, yellow, and green traffic-light-like blobs from an RGB image using
HSV thresholding, morphology, circularity filtering, and temporal stability.
The detector intentionally publishes UNKNOWN when confidence is low so false
positive tests fail safely.
"""

import math
import time
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


VALID_STATES = {'RED', 'YELLOW', 'GREEN', 'UNKNOWN'}


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


@dataclass
class Candidate:
    state: str
    confidence: float
    x: int
    y: int
    area: float
    circularity: float


class TrafficLightDetectorNode(Node):

    def __init__(self):
        super().__init__('traffic_light_detector')

        self.declare_parameter('image_topic', '/oak/rgb/image_raw')
        self.declare_parameter('output_topic', '/traffic_light_state')
        self.declare_parameter('show_debug', False)
        self.declare_parameter('roi_top_fraction', 0.75)
        self.declare_parameter('min_blob_area', 90.0)
        self.declare_parameter('max_blob_area_fraction', 0.08)
        self.declare_parameter('min_circularity', 0.45)
        self.declare_parameter('min_confidence', 0.45)
        self.declare_parameter('stable_frames', 3)
        self.declare_parameter('unknown_timeout_sec', 0.8)

        image_topic = self.get_parameter('image_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.show_debug = _as_bool(self.get_parameter('show_debug').value)
        self.roi_top_fraction = float(self.get_parameter('roi_top_fraction').value)
        self.min_blob_area = float(self.get_parameter('min_blob_area').value)
        self.max_blob_area_fraction = float(
            self.get_parameter('max_blob_area_fraction').value
        )
        self.min_circularity = float(self.get_parameter('min_circularity').value)
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.stable_frames = int(self.get_parameter('stable_frames').value)
        self.unknown_timeout_sec = float(self.get_parameter('unknown_timeout_sec').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._bridge = CvBridge()
        self._pub = self.create_publisher(String, output_topic, 10)
        self.create_subscription(Image, image_topic, self._on_image, qos)

        self._candidate_state = 'UNKNOWN'
        self._candidate_count = 0
        self._stable_state = 'UNKNOWN'
        self._last_detection_time = 0.0
        self._last_publish_time = 0.0

        self.get_logger().info(
            f'Traffic light detector ready. image={image_topic}, output={output_topic}'
        )

    def _on_image(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge error: {exc}')
            return

        candidate = self._detect(frame)
        now = time.monotonic()
        if candidate is None:
            if now - self._last_detection_time > self.unknown_timeout_sec:
                self._update_state('UNKNOWN', 0.0, -1, -1, 0.0)
            if self.show_debug:
                cv2.imshow('Traffic Light Debug', frame)
                cv2.waitKey(1)
            return

        self._last_detection_time = now
        self._update_state(
            candidate.state,
            candidate.confidence,
            candidate.x,
            candidate.y,
            candidate.area,
        )

        if self.show_debug:
            color = {
                'RED': (0, 0, 255),
                'YELLOW': (0, 255, 255),
                'GREEN': (0, 255, 0),
            }.get(candidate.state, (255, 255, 255))
            cv2.circle(frame, (candidate.x, candidate.y), 14, color, 2)
            cv2.putText(
                frame,
                f'{candidate.state} {candidate.confidence:.2f}',
                (candidate.x + 16, candidate.y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
            )
            cv2.imshow('Traffic Light Debug', frame)
            cv2.waitKey(1)

    def _update_state(self, observed: str, confidence: float, x: int, y: int, area: float):
        if observed == self._candidate_state:
            self._candidate_count += 1
        else:
            self._candidate_state = observed
            self._candidate_count = 1

        if observed == 'UNKNOWN' or self._candidate_count >= self.stable_frames:
            self._stable_state = observed
            self._publish_state(confidence, x, y, area)

    def _publish_state(self, confidence: float, x: int, y: int, area: float):
        now = time.monotonic()
        if now - self._last_publish_time < 0.1:
            return
        self._last_publish_time = now

        msg = String()
        msg.data = (
            f'{time.time():.3f},{self._stable_state},{confidence:.3f},'
            f'{x},{y},{area:.1f}'
        )
        self._pub.publish(msg)
        self.get_logger().info(
            f'Traffic light: {self._stable_state} '
            f'(confidence={confidence:.2f}, area={area:.0f})'
        )

    def _detect(self, frame: np.ndarray) -> Candidate | None:
        h, w = frame.shape[:2]
        roi_h = max(1, min(h, int(h * self.roi_top_fraction)))
        roi = frame[:roi_h, :]

        blurred = cv2.GaussianBlur(roi, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        hsv[:, :, 2] = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(
            hsv[:, :, 2]
        )

        masks = {
            'RED': self._mask_red(hsv),
            'YELLOW': cv2.inRange(hsv, (15, 70, 80), (38, 255, 255)),
            'GREEN': cv2.inRange(hsv, (40, 55, 60), (95, 255, 255)),
        }

        candidates: list[Candidate] = []
        max_area = float(h * w) * self.max_blob_area_fraction
        for state, mask in masks.items():
            cleaned = self._clean_mask(mask)
            candidates.extend(self._candidates_from_mask(state, cleaned, hsv, max_area))

        if not candidates:
            return None

        best = max(candidates, key=lambda item: item.confidence)
        if best.confidence < self.min_confidence:
            return None
        return best

    @staticmethod
    def _mask_red(hsv: np.ndarray) -> np.ndarray:
        low_red = cv2.inRange(hsv, (0, 70, 80), (10, 255, 255))
        high_red = cv2.inRange(hsv, (170, 70, 80), (180, 255, 255))
        return cv2.bitwise_or(low_red, high_red)

    @staticmethod
    def _clean_mask(mask: np.ndarray) -> np.ndarray:
        kernel = np.ones((5, 5), np.uint8)
        opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)

    def _candidates_from_mask(
            self,
            state: str,
            mask: np.ndarray,
            hsv: np.ndarray,
            max_area: float) -> list[Candidate]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.min_blob_area or area > max_area:
                continue

            perimeter = float(cv2.arcLength(contour, True))
            if perimeter <= 0.0:
                continue

            x, y, bw, bh = cv2.boundingRect(contour)
            aspect = bw / max(float(bh), 1.0)
            if aspect < 0.45 or aspect > 1.8:
                continue

            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            if circularity < self.min_circularity:
                continue

            mask_roi = mask[y:y + bh, x:x + bw]
            hsv_roi = hsv[y:y + bh, x:x + bw]
            selected = hsv_roi[mask_roi > 0]
            if selected.size == 0:
                continue

            sat = float(np.mean(selected[:, 1])) / 255.0
            val = float(np.mean(selected[:, 2])) / 255.0
            fill = min(area / max(float(bw * bh), 1.0), 1.0)
            area_score = min(area / (self.min_blob_area * 8.0), 1.0)
            confidence = (
                0.35 * min(circularity, 1.0)
                + 0.25 * sat
                + 0.20 * val
                + 0.10 * fill
                + 0.10 * area_score
            )

            moments = cv2.moments(contour)
            if moments['m00'] == 0:
                cx = x + bw // 2
                cy = y + bh // 2
            else:
                cx = int(moments['m10'] / moments['m00'])
                cy = int(moments['m01'] / moments['m00'])

            candidates.append(
                Candidate(state, confidence, cx, cy, area, circularity)
            )
        return candidates


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
