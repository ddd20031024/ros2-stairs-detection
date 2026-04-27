import math
import os

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from ultralytics import YOLO


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('model_file', '')
        self.declare_parameter('fov_deg', 60.0)
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('min_valid_distance', 0.02)
        self.declare_parameter('show_debug_window', True)
        self.declare_parameter('log_interval_sec', 1.0)

        self.bridge = CvBridge()
        self.latest_scan = None
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.model_file = str(self.get_parameter('model_file').value)
        self.fov = float(self.get_parameter('fov_deg').value)
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.min_valid_distance = float(self.get_parameter('min_valid_distance').value)
        self.show_debug_window = bool(self.get_parameter('show_debug_window').value)
        self.log_interval_sec = float(self.get_parameter('log_interval_sec').value)
        self._last_detection_log_ns = 0

        pkg_share = get_package_share_directory('yolo_lidar_fusion')
        model_dir = os.path.join(pkg_share, 'model')
        candidate_names = (self.model_file, 'best.pt', 'best_seg.pt', 'best_seg_seg.pt')
        self.model_path = next(
            (os.path.join(model_dir, name) for name in candidate_names if name and os.path.exists(os.path.join(model_dir, name))),
            None,
        )

        if self.model_path is None:
            raise FileNotFoundError(
                f'No YOLO model file found in {model_dir}. Expected one of: {", ".join(candidate_names)}'
            )

        self.get_logger().info(f'Loading YOLO model from: {self.model_path}')
        self.model = YOLO(self.model_path)

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10,
        )
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )

        self.get_logger().info(
            f'Fusion node started. image_topic={self.image_topic}, scan_topic={self.scan_topic}, '
            f'fov_deg={self.fov}, conf={self.conf_threshold}'
        )

    def scan_callback(self, msg):
        self.latest_scan = msg

    def image_callback(self, msg):
        if self.latest_scan is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Failed to convert image: {exc}')
            return

        _, width = frame.shape[:2]

        try:
            results = self.model.predict(source=frame, conf=self.conf_threshold, verbose=False)
        except Exception as exc:
            self.get_logger().error(f'YOLO inference failed: {exc}')
            return

        if not results:
            return

        result = results[0]
        annotated = result.plot()

        if len(result.boxes) > 0:
            # Choose the highest-confidence detection for distance query stability.
            best_idx = int(result.boxes.conf.argmax().item())
            box = result.boxes[best_idx].xyxy[0].cpu().numpy()
            x_center = int((box[0] + box[2]) / 2)
            y_center = int((box[1] + box[3]) / 2)

            angle_deg = ((width / 2) - x_center) / width * self.fov
            angle_rad = math.radians(angle_deg)

            scan = self.latest_scan
            if scan.angle_increment <= 0.0 or len(scan.ranges) == 0:
                return

            # Normalize target angle into the scanner's angle window when possible.
            normalized_angle = angle_rad
            while normalized_angle < scan.angle_min:
                normalized_angle += 2.0 * math.pi
            while normalized_angle > scan.angle_max:
                normalized_angle -= 2.0 * math.pi

            index = int((normalized_angle - scan.angle_min) / scan.angle_increment)

            # Some scanners publish full 360 deg data with wrapped index semantics.
            # For those cases, allow wrapped indexing when range covers nearly 2*pi.
            if index < 0 or index >= len(scan.ranges):
                scan_span = abs(scan.angle_max - scan.angle_min)
                if scan_span >= (2.0 * math.pi - 0.1):
                    index = index % len(scan.ranges)
                else:
                    return

            if 0 <= index < len(scan.ranges):
                distance = scan.ranges[index]
                if not math.isinf(distance) and not math.isnan(distance) and distance > self.min_valid_distance:
                    cv2.putText(
                        annotated,
                        f'Dist: {distance:.2f}m',
                        (x_center, max(0, y_center - 20)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 0),
                        2,
                    )
                    now_ns = self.get_clock().now().nanoseconds
                    if now_ns - self._last_detection_log_ns >= int(self.log_interval_sec * 1e9):
                        self.get_logger().info(
                            f'检测到目标: x={x_center}, angle={math.degrees(normalized_angle):.1f} deg, dist={distance:.3f} m'
                        )
                        self._last_detection_log_ns = now_ns

        if self.show_debug_window:
            cv2.imshow('Fusion View', annotated)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        show_window = node.show_debug_window
        node.destroy_node()
        if show_window:
            cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()