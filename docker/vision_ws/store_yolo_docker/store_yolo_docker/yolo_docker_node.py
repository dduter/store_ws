import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class YoloDockerNode(Node):
    """한 프로세스에서 상품과 조끼를 검출한다."""

    def __init__(self) -> None:
        super().__init__('yolo_docker_node')
        self.declare_parameter('weights_path', '/config/weights/products_yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.40)
        self.declare_parameter('vest_weights_path', '/config/weights/vest_yolov8n.pt')
        self.declare_parameter('vest_conf_threshold', 0.70)

        weights_path = self.get_parameter('weights_path').get_parameter_value().string_value
        self.conf_threshold = (
            self.get_parameter('conf_threshold').get_parameter_value().double_value
        )

        self.bridge = CvBridge()
        self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        self._vest_conf_threshold = self.get_parameter('vest_conf_threshold').value
        self.model = self._load_model(weights_path, '상품')
        vest_weights_path = self.get_parameter('vest_weights_path').value
        self.vest_model = self._load_model(vest_weights_path, '조끼', optional=True)

        self.det_pub = self.create_publisher(String, '/store/yolo_detections', 10)
        self.vest_pub = self.create_publisher(String, '/store/vision/vest_detected', 10)
        self.health_pub = self.create_publisher(String, '/store/node_health_raw', 10)

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/color/image_raw', self.on_color_image, sensor_qos)
        self.create_subscription(Image, '/webcam/image_raw', self.on_webcam_image, sensor_qos)
        self.create_timer(1.0, self.publish_heartbeat)

    def _load_model(self, path: str, label: str, *, optional: bool = False) -> YOLO | None:
        logger = self.get_logger()
        if not Path(path).is_file():
            log = logger.info if optional else logger.error
            log(f'{label} YOLO 가중치 파일을 찾지 못했습니다 ({path}).')
            return None
        try:
            model = YOLO(path)
            model.predict(np.zeros((640, 640, 3), dtype=np.uint8), device=self.device, verbose=False)
        except Exception as exc:
            logger.error(f'{label} YOLO 모델 로딩 실패 ({path}). {exc}')
            return None
        logger.info(f'{label} YOLO 모델을 로드했습니다 ({path}, {self.device}, {model.names}).')
        return model

    @staticmethod
    def _estimate_yaw_deg(crop_bgr: np.ndarray) -> float:
        """물체의 짧은 축을 파지하는 그리퍼 회전각을 (-90, 90] 범위로 추정한다."""
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
        # 경계 사각형의 짧은 변을 가로지르도록 그리퍼 방향을 맞춘다.
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

        payload = {
            'stamp_sec': int(msg.header.stamp.sec),
            'stamp_nanosec': int(msg.header.stamp.nanosec),
            'detections': detections,
        }
        self.det_pub.publish(String(data=json.dumps(payload)))

    def on_webcam_image(self, msg: Image) -> None:
        if self.vest_model is None:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        result = self.vest_model.predict(
            frame, conf=self._vest_conf_threshold, device=self.device, verbose=False
        )[0]
        confidence = max(result.boxes.conf.tolist(), default=0.0) if result.boxes is not None else 0.0
        payload = {
            'detected': confidence > 0.0 and confidence >= self._vest_conf_threshold,
            'confidence': float(confidence),
            'source': 'vision',
        }
        self.vest_pub.publish(String(data=json.dumps(payload)))

    def publish_heartbeat(self) -> None:
        models = ', '.join(
            label for label, model in (('products', self.model), ('vest', self.vest_model))
            if model is not None
        )
        payload = {
            'node_name': 'yolo_docker_node',
            'alive': True,
            'state': 'OK' if self.model is not None else 'ERROR',
            'message': f'로드된 모델: {models or "없음"}, 장치: {self.device}',
        }
        self.health_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloDockerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
