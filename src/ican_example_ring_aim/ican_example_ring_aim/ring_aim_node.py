import math

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


class RingAimNode(Node):
    def __init__(self):
        super().__init__("ring_aim_node")
        self.declare_parameter("image_topic", "/camera/image")
        self.declare_parameter("center_topic", "/ring_target/center_offset")
        self.declare_parameter("debug_image_topic", "/ring_target/debug_image")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("min_radius_px", 8.0)

        self.bridge = CvBridge()
        self.min_radius_px = float(self.get_parameter("min_radius_px").value)
        self.latest_center = None
        self.latest_stamp = None

        image_topic = str(self.get_parameter("image_topic").value)
        center_topic = str(self.get_parameter("center_topic").value)
        debug_topic = str(self.get_parameter("debug_image_topic").value)
        self.publish_debug = bool(self.get_parameter("publish_debug_image").value)

        self.center_pub = self.create_publisher(PointStamped, center_topic, 10)
        self.debug_pub = self.create_publisher(Image, debug_topic, 10) if self.publish_debug else None
        self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
        self.create_service(Trigger, "/ring_target/get_center", self.on_get_center)
        self.get_logger().info(f"Ring aim example: {image_topic} -> {center_topic}")

    def on_image(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        detection = self.detect_ring(image)

        if detection is not None:
            cx, cy, radius, confidence = detection
            height, width = image.shape[:2]
            out = PointStamped()
            out.header = msg.header
            out.header.frame_id = "camera_screen"
            out.point.x = float(cx - width * 0.5)
            out.point.y = float(cy - height * 0.5)
            out.point.z = float(confidence)
            self.center_pub.publish(out)
            self.latest_center = out
            self.latest_stamp = self.get_clock().now()

        if self.debug_pub is not None:
            debug = image.copy()
            height, width = debug.shape[:2]
            cv2.drawMarker(debug, (width // 2, height // 2), (255, 255, 255), cv2.MARKER_CROSS, 18, 2)
            if detection is not None:
                cx, cy, radius, _ = detection
                cv2.circle(debug, (int(cx), int(cy)), int(radius), (0, 255, 255), 2)
                cv2.drawMarker(debug, (int(cx), int(cy)), (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
            debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)

    def detect_ring(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        red = cv2.bitwise_or(
            cv2.inRange(hsv, np.array([0, 55, 45]), np.array([15, 255, 255])),
            cv2.inRange(hsv, np.array([165, 55, 45]), np.array([179, 255, 255])),
        )
        blue = cv2.inRange(hsv, np.array([85, 40, 35]), np.array([130, 255, 255]))
        yellow = cv2.inRange(hsv, np.array([18, 45, 70]), np.array([45, 255, 255]))

        components = []
        for mask, weight in ((red, 1.0), (blue, 0.8), (yellow, 0.5)):
            item = self.best_circle(mask)
            if item is not None:
                cx, cy, radius, area, circularity = item
                score = max(0.0, circularity) * math.sqrt(max(area, 1.0)) * weight
                components.append((np.array([cx, cy]), radius, score, circularity))

        if not components:
            return None

        largest = max(components, key=lambda item: item[1])
        coherent = [item for item in components if np.linalg.norm(item[0] - largest[0]) < max(10.0, largest[1] * 0.35)]
        score_sum = sum(item[2] for item in coherent)
        center = sum((item[0] * item[2] for item in coherent), np.zeros(2)) / max(score_sum, 1e-6)
        radius = max(item[1] for item in coherent)
        confidence = min(1.0, 0.35 + 0.18 * len(coherent) + largest[3] * 0.35)
        return float(center[0]), float(center[1]), float(radius), float(confidence)

    def best_circle(self, mask):
        clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), dtype=np.uint8))
        clean = cv2.morphologyEx(clean, cv2.MORPH_CLOSE, np.ones((5, 5), dtype=np.uint8))
        contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 80.0:
                continue
            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 1e-6:
                continue
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            if radius < self.min_radius_px or circularity < 0.25:
                continue
            score = area * circularity
            if best is None or score > best[0]:
                best = (score, cx, cy, radius, area, circularity)
        if best is None:
            return None
        _, cx, cy, radius, area, circularity = best
        return cx, cy, radius, area, circularity

    def on_get_center(self, _request, response):
        if self.latest_center is None:
            response.success = False
            response.message = "no ring target detected"
            return response
        point = self.latest_center.point
        response.success = True
        response.message = f"x={point.x:.2f} y={point.y:.2f} confidence={point.z:.3f}"
        return response


def main():
    rclpy.init()
    node = RingAimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
