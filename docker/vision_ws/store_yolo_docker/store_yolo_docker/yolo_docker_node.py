import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class YoloDockerNode(Node):
    """Single-process YOLOv8n inference node for product detection."""

    def __init__(self) -> None:
        super().__init__('yolo_docker_node')
        self.declare_parameter('weights_path', '/config/weights/products_yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.40)

        weights_path = self.get_parameter('weights_path').get_parameter_value().string_value
        self.conf_threshold = (
            self.get_parameter('conf_threshold').get_parameter_value().double_value
        )

        self.bridge = CvBridge()
        self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        self.model: YOLO | None = None

        if Path(weights_path).exists():
            self.model = YOLO(weights_path)
            dummy = np.zeros((640, 640, 3), dtype=np.uint8)
            self.model.predict(dummy, device=self.device, verbose=False)
            self.get_logger().info(
                f'Loaded YOLO weights={weights_path} on {self.device}, classes={self.model.names}'
            )
        else:
            self.get_logger().error(f'YOLO weights not found: {weights_path}')

        self.det_pub = self.create_publisher(String, '/store/yolo_detections', 10)
        self.vest_pub = self.create_publisher(String, '/store/vision/vest_detected', 10)
        self.health_pub = self.create_publisher(String, '/store/node_health_raw', 10)

        self.create_subscription(Image, '/camera/color/image_raw', self.on_color_image, 10)
        self.create_subscription(Image, '/webcam/image_raw', self.on_webcam_image, 10)
        self.create_timer(1.0, self.publish_heartbeat)

    @staticmethod
    def _estimate_yaw_deg(crop_bgr: np.ndarray) -> float:
        """Estimate gripper rz (deg) to grasp the short axis, normalized to [-90, 90]."""
        if crop_bgr.size == 0:
            return 0.0
        gray = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0.0
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < 100:
            return 0.0
        (_, _), (rw, rh), angle = cv2.minAreaRect(largest)
        # Align gripper fingers across the short side of the bounding rectangle.
        if rw < rh:
            angle += 90.0
        while angle > 90.0:
            angle -= 180.0
        while angle <= -90.0:
            angle += 180.0
        return round(float(angle), 2)

    def on_color_image(self, msg: Image) -> None:
        if self.model is None:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model.predict(
            frame, conf=self.conf_threshold, device=self.device, verbose=False
        )
        res = results[0]
        h, w = frame.shape[:2]

        detections = []
        if res.boxes is not None:
            for box in res.boxes:
                cls_id = int(box.cls[0].item())
                cls_name = str(self.model.names.get(cls_id, cls_id))
                conf = round(float(box.conf[0].item()), 4)
                x1, y1, x2, y2 = [int(round(v)) for v in box.xyxy[0].tolist()]
                x1_c, y1_c = max(0, x1), max(0, y1)
                x2_c, y2_c = min(w, x2), min(h, y2)
                yaw_deg = self._estimate_yaw_deg(frame[y1_c:y2_c, x1_c:x2_c])
                detections.append(
                    {
                        'class_name': cls_name,
                        'is_backside': cls_name == 'product_back',
                        'conf': conf,
                        'bbox_px': [x1, y1, x2, y2],
                        'yaw_deg': yaw_deg,
                    }
                )

        payload = {'detections': detections}
        self.det_pub.publish(String(data=json.dumps(payload)))

    def on_webcam_image(self, msg: Image) -> None:
        # Vest detection is outside the user's vision scope; kept as no-op.
        _ = msg

    def publish_heartbeat(self) -> None:
        state = 'OK' if self.model is not None else 'ERROR'
        payload = {'node_name': 'yolo_docker_node', 'alive': self.model is not None, 'state': state}
        self.health_pub.publish(String(data=json.dumps(payload)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloDockerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
