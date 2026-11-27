import json
from pathlib import Path

import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from store_interfaces.msg import DetectedProduct
from store_interfaces.srv import ScanWorkspace

from store_vision.core.coordinate_math import (
    camera_to_base_3d,
    pixel_to_camera_3d,
)


def normalize_rz_deg(angle_deg: float) -> float:
    """Normalize rotation angle to [-90, 90] deg."""
    angle = float(angle_deg)
    while angle > 90.0:
        angle -= 180.0
    while angle <= -90.0:
        angle += 180.0
    return round(angle, 2)


class PerceptionNode(Node):
    """Integrates YOLO detections and aligned depth into 3D base coordinates (Z_surface)."""

    def __init__(self) -> None:
        super().__init__('perception_node')
        self.declare_parameter(
            'catalog_path', '/home/ms-04/store_ws/config/products_catalog.yaml'
        )
        self.declare_parameter(
            'calibration_path', '/home/ms-04/store_ws/config/calibration/T_gripper2camera.npy'
        )
        self.declare_parameter('fx', 615.0)
        self.declare_parameter('fy', 615.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)

        self.fx = self.get_parameter('fx').get_parameter_value().double_value
        self.fy = self.get_parameter('fy').get_parameter_value().double_value
        self.cx = self.get_parameter('cx').get_parameter_value().double_value
        self.cy = self.get_parameter('cy').get_parameter_value().double_value

        self.bridge = CvBridge()
        self._latest_detections: list[dict] = []
        self._latest_depth: np.ndarray | None = None
        self._has_camera_info = False

        self._catalog_by_yolo = self._load_catalog(
            self.get_parameter('catalog_path').get_parameter_value().string_value
        )
        self._t_gripper2camera = self._load_calibration(
            self.get_parameter('calibration_path').get_parameter_value().string_value
        )
        # Fallback scan_inspect pose (420, 0, 380 mm) looking straight down (-Z).
        # TODO(비전B): Replace with live robot pose (get_current_posx) on hardware integration.
        self._t_base2gripper = np.array(
            [
                [1.0, 0.0, 0.0, 420.0],
                [0.0, -1.0, 0.0, 0.0],
                [0.0, 0.0, -1.0, 380.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

        self.create_subscription(String, '/store/yolo_detections', self.on_yolo, 10)
        self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw', self.on_depth, 10
        )
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.on_camera_info, 10)
        self.scan_srv = self.create_service(
            ScanWorkspace, '/store/scan_workspace', self.handle_scan
        )
        self.get_logger().info('perception_node initialized (/store/scan_workspace ready)')

    def _load_catalog(self, path_str: str) -> dict[str, dict]:
        path = Path(path_str)
        if not path.exists():
            self.get_logger().warning(f'Catalog not found: {path_str}')
            return {}
        data = yaml.safe_load(path.read_text()) or {}
        by_yolo = {}
        for item in (data.get('products') or {}).values():
            yolo_cls = item.get('yolo_class')
            if yolo_cls:
                by_yolo[yolo_cls] = item
        return by_yolo

    def _lookup_catalog(self, yolo_cls: str) -> dict:
        if yolo_cls in self._catalog_by_yolo:
            return self._catalog_by_yolo[yolo_cls]
        # Allow temporary compatibility with pre-_front test weights.
        if not yolo_cls.endswith('_front'):
            return self._catalog_by_yolo.get(f'{yolo_cls}_front', {})
        return {}

    def _load_calibration(self, path_str: str) -> np.ndarray:
        path = Path(path_str)
        if path.exists():
            return np.load(str(path)).astype(np.float64)
        self.get_logger().warning(
            f'Calibration file not found ({path_str}); using identity fallback for testing'
        )
        return np.eye(4, dtype=np.float64)

    def on_yolo(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self._latest_detections = payload.get('detections', [])
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f'Invalid YOLO JSON payload: {exc}')

    def on_depth(self, msg: Image) -> None:
        self._latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def on_camera_info(self, msg: CameraInfo) -> None:
        if len(msg.k) >= 6 and msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.fx = float(msg.k[0])
            self.fy = float(msg.k[4])
            self.cx = float(msg.k[2])
            self.cy = float(msg.k[5])
            self._has_camera_info = True

    def _sample_depth_mm(self, bbox_px: list[int]) -> float | None:
        if self._latest_depth is None:
            return None
        h, w = self._latest_depth.shape[:2]
        x1, y1, x2, y2 = bbox_px
        u = int(np.clip((x1 + x2) // 2, 0, w - 1))
        v = int(np.clip((y1 + y2) // 2, 0, h - 1))
        patch = self._latest_depth[max(0, v - 2) : min(h, v + 3), max(0, u - 2) : min(w, u + 3)]
        valid = patch[patch > 0]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def handle_scan(
        self, request: ScanWorkspace.Request, response: ScanWorkspace.Response
    ) -> ScanWorkspace.Response:
        _ = request
        if self._latest_depth is None:
            response.success = False
            response.error_code = 'NO_DEPTH_FRAME'
            return response

        products: list[DetectedProduct] = []
        for idx, det in enumerate(self._latest_detections):
            bbox = det.get('bbox_px', [0, 0, 0, 0])
            depth_mm = self._sample_depth_mm(bbox)
            if depth_mm is None:
                continue

            u = (bbox[0] + bbox[2]) / 2.0
            v = (bbox[1] + bbox[3]) / 2.0
            h, w = self._latest_depth.shape[:2]
            cx = self.cx if self._has_camera_info else w / 2.0
            cy = self.cy if self._has_camera_info else h / 2.0

            p_cam_h = pixel_to_camera_3d(u, v, depth_mm, self.fx, self.fy, cx, cy)
            p_base = camera_to_base_3d(p_cam_h, self._t_base2gripper, self._t_gripper2camera)

            yolo_cls = str(det.get('class_name', ''))
            info = self._lookup_catalog(yolo_cls)
            rz_deg = normalize_rz_deg(float(det.get('yaw_deg', 0.0)))

            msg = DetectedProduct()
            msg.track_id = f'det_{idx}'
            msg.yolo_class = str(info.get('yolo_class', yolo_cls))
            msg.product_name = str(info.get('display_name', yolo_cls))
            msg.is_backside = bool(det.get('is_backside', False))
            msg.confidence = float(det.get('conf', 0.0))
            msg.bbox_px = [int(v) for v in bbox[:4]]
            # Return pure Z_surface (mm); robot executor applies category grasp_depth_offset_mm.
            msg.pose_3d_base = [
                round(float(p_base[0]), 2),
                round(float(p_base[1]), 2),
                round(float(p_base[2]), 2),
                rz_deg,
            ]
            msg.grasp_width_mm = float(info.get('grasp_width_mm', 60.0))
            msg.depth_layer = 1
            msg.status = 'NeedsReorientation' if msg.is_backside else 'Normal'
            products.append(msg)

        response.success = True
        response.error_code = 'OK'
        response.products = products
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
