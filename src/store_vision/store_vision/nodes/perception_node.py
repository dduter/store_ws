import json
import os
from functools import partial
from pathlib import Path
from threading import Event, Lock

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from store_interfaces.msg import ContainerPose, DetectedProduct, NodeHealth
from store_interfaces.srv import ScanWorkspace

from store_vision.core.catalog_utils import (
    find_config_dir,
    load_catalog_index,
    load_default_containers,
    load_scan_pose_6d,
    lookup_catalog,
)
from store_vision.core.coordinate_math import (
    camera_to_base_3d,
    camera_yaw_to_base_rz,
    is_valid_se3_matrix,
    normalize_rz_deg,
    pixel_to_camera_3d,
    pose_6d_to_matrix,
)
from store_vision.core.depth_layer import assign_depth_layers
from store_vision.core.vision_vlm import inspect_crop_with_openai, inspect_crops_parallel
from store_vision.nodes.image_utils import image_msg_to_numpy


class PerceptionNode(Node):
    """YOLO 검출과 정렬 깊이를 로봇 베이스 기준의 상품 표면 좌표로 변환한다."""

    def __init__(self) -> None:
        super().__init__('perception_node')
        self.declare_parameters('', [
            ('config_dir', ''), ('catalog_path', ''), ('calibration_path', ''), ('zones_path', ''),
            ('robot_id', 'dsr01'), ('use_live_robot_pose', True), ('robot_pose_timeout_sec', 0.8),
            ('color_topic', '/camera/color/image_raw'),
            ('depth_topic', '/camera/aligned_depth_to_color/image_raw'),
            ('camera_info_topic', '/camera/color/camera_info'),
            ('vlm_enabled', True), ('vlm_model', 'gpt-4o'), ('vlm_timeout_sec', 12.0),
            ('layer_gap_mm', 25.0), ('heartbeat_period_sec', 2.0),
            ('fx', 615.0), ('fy', 615.0), ('cx', 320.0), ('cy', 240.0),
        ])
        param = self.get_parameter
        config_dir = Path(param('config_dir').value or find_config_dir()).expanduser()
        catalog_path = Path(param('catalog_path').value or config_dir / 'products_catalog.yaml').expanduser()
        zones_path = Path(param('zones_path').value or config_dir / 'workspace_zones.yaml').expanduser()
        calibration_path = Path(
            param('calibration_path').value or config_dir / 'calibration' / 'T_gripper2camera.npy'
        ).expanduser()

        self._catalog = load_catalog_index(catalog_path)
        self._containers = load_default_containers(zones_path)
        self._scan_transform = pose_6d_to_matrix(load_scan_pose_6d(zones_path))
        self._t_gripper2camera = self._load_calibration(calibration_path)
        self._vlm_enabled = param('vlm_enabled').value
        self._vlm_model = param('vlm_model').value
        self._vlm_timeout = param('vlm_timeout_sec').value
        self._layer_gap_mm = param('layer_gap_mm').value
        self._pose_timeout = param('robot_pose_timeout_sec').value
        self._intrinsics = tuple(param(name).value for name in ('fx', 'fy', 'cx', 'cy'))

        self._lock = Lock()
        self._latest_detections: list[dict] = []
        self._latest_depth: np.ndarray | None = None
        self._latest_color: np.ndarray | None = None
        self._has_camera_info = False
        self._io_group = ReentrantCallbackGroup()
        # 스캔을 직렬화해 자세 서비스 응답에 필요한 실행기 스레드를 확보한다.
        self._scan_group = MutuallyExclusiveCallbackGroup()
        self._pose_client = None
        if param('use_live_robot_pose').value:
            try:
                from dsr_msgs2.srv import GetCurrentPosx
            except ImportError:
                self.get_logger().info('dsr_msgs2를 찾지 못해 설정된 스캔 자세를 사용합니다.')
            else:
                robot_id = param('robot_id').value.strip('/')
                self._pose_request_type = GetCurrentPosx.Request
                self._pose_client = self.create_client(
                    GetCurrentPosx, f'/{robot_id}/dsr_controller2/aux_control/get_current_posx',
                    callback_group=self._io_group,
                )

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for msg_type, name, callback in (
            (Image, 'color_topic', self.on_color),
            (Image, 'depth_topic', self.on_depth),
            (CameraInfo, 'camera_info_topic', self.on_camera_info),
        ):
            self.create_subscription(
                msg_type, param(name).value, callback, sensor_qos, callback_group=self._io_group
            )
        self.create_subscription(
            String, '/store/yolo_detections', self.on_yolo, 10, callback_group=self._io_group
        )
        self._health_pub = self.create_publisher(NodeHealth, '/store/node_health', 10)
        self.create_subscription(
            String, '/store/node_health_raw', self.on_health_raw, 10, callback_group=self._io_group
        )
        self.create_timer(
            param('heartbeat_period_sec').value, self.publish_heartbeat, callback_group=self._io_group
        )
        self.scan_srv = self.create_service(
            ScanWorkspace, '/store/scan_workspace', self.handle_scan, callback_group=self._scan_group
        )

    def _load_calibration(self, path: Path) -> np.ndarray | None:
        try:
            matrix = np.load(path).astype(np.float64)
            if not is_valid_se3_matrix(matrix):
                raise ValueError('유효한 4×4 SE(3) 강체 변환 행렬이 필요합니다.')
            return matrix
        except (OSError, ValueError) as exc:
            self.get_logger().error(f'카메라 보정 행렬을 읽지 못했습니다 ({path}). {exc}')
            return None

    def _base_to_gripper(self) -> np.ndarray:
        client = self._pose_client
        if client is not None and client.service_is_ready():
            try:
                event = Event()
                future = client.call_async(self._pose_request_type(ref=0))
                future.add_done_callback(lambda _: event.set())
                if event.wait(self._pose_timeout):
                    result = future.result()
                    if result.success and result.task_pos_info:
                        return pose_6d_to_matrix(result.task_pos_info[0].data[:6])
                else:
                    client.remove_pending_request(future)
                    future.cancel()
            except (RuntimeError, ValueError) as exc:
                self.get_logger().warning(f'로봇 자세 조회 실패. {exc}')
            self.get_logger().warning('로봇 자세를 받지 못해 설정된 스캔 자세를 사용합니다.')
        return self._scan_transform

    def on_yolo(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            detections = payload['detections']
            if not isinstance(detections, list):
                raise ValueError('검출 목록이 배열이어야 합니다.')
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warning(f'YOLO 검출 메시지 오류. {exc}')
            return
        with self._lock:
            self._latest_detections = detections

    def on_color(self, msg: Image) -> None:
        try:
            if msg.encoding not in ('bgr8', 'rgb8'):
                raise ValueError(msg.encoding)
            color = image_msg_to_numpy(msg)
            if msg.encoding == 'rgb8':
                color = color[:, :, ::-1]
        except (ValueError, KeyError) as exc:
            self.get_logger().warning(f'컬러 프레임 변환 실패. {exc}')
            return
        with self._lock:
            self._latest_color = color

    def on_depth(self, msg: Image) -> None:
        try:
            if msg.encoding not in ('16UC1', '32FC1'):
                raise ValueError(msg.encoding)
            depth = image_msg_to_numpy(msg)
        except (ValueError, KeyError) as exc:
            self.get_logger().warning(f'깊이 프레임 변환 실패. {exc}')
            return
        with self._lock:
            self._latest_depth = depth

    def on_camera_info(self, msg: CameraInfo) -> None:
        fx, fy, cx, cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
        if fx > 0 and fy > 0 and np.isfinite([fx, fy, cx, cy]).all():
            with self._lock:
                self._intrinsics = (fx, fy, cx, cy)
                self._has_camera_info = True

    def on_health_raw(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            name, alive, state = payload['node_name'], payload['alive'], payload['state']
            message = payload.get('message', '')
            if not (isinstance(name, str) and name and isinstance(alive, bool)
                    and isinstance(state, str) and isinstance(message, str)):
                raise ValueError('상태 필드 형식이 올바르지 않습니다.')
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warning(f'노드 상태 메시지 변환 실패. {exc}')
            return
        self._publish_health(name, alive, state, message)

    def _publish_health(self, name, alive, state, message) -> None:
        self._health_pub.publish(NodeHealth(
            node_name=name, alive=alive, state=state, message=message,
            stamp=self.get_clock().now().to_msg(),
        ))

    def publish_heartbeat(self) -> None:
        with self._lock:
            has_depth = self._latest_depth is not None
        if self._t_gripper2camera is None:
            state, message = 'ERROR', '카메라 보정 행렬을 읽지 못했습니다.'
        elif not has_depth:
            state, message = 'WAITING', '정렬 깊이 프레임을 기다리는 중입니다.'
        else:
            state, message = 'OK', '작업공간 스캔을 받을 수 있습니다.'
        self._publish_health(self.get_name(), True, state, message)

    def handle_scan(
        self, request: ScanWorkspace.Request, response: ScanWorkspace.Response
    ) -> ScanWorkspace.Response:
        response.containers = [ContainerPose(**entry) for entry in self._containers]
        with self._lock:
            detections = self._latest_detections.copy()
            depth = None if self._latest_depth is None else self._latest_depth.copy()
            color = None if self._latest_color is None else self._latest_color.copy()
            fx, fy, cx, cy = self._intrinsics
            has_camera_info = self._has_camera_info

        if depth is None or not depth.size:
            response.error_code = 'NO_DEPTH_FRAME'
            return response
        if self._t_gripper2camera is None:
            response.error_code = 'NO_CAMERA_CALIBRATION'
            return response

        height, width = depth.shape
        if not has_camera_info:
            cx, cy = width / 2.0, height / 2.0
        t_base2gripper = self._base_to_gripper()
        items = []
        for idx, det in enumerate(detections):
            if not isinstance(det, dict):
                continue
            yolo_cls = str(det.get('class_name', '')).strip()
            if not yolo_cls or yolo_cls.lower() == 'vest':
                continue
            try:
                x1, y1, x2, y2 = bbox = [int(value) for value in det['bbox_px']]
                yaw = normalize_rz_deg(det.get('yaw_deg', 0.0))
                confidence = float(det.get('conf', 0.0))
            except (TypeError, ValueError, KeyError, OverflowError):
                continue
            if x1 >= x2 or y1 >= y2 or not np.isfinite(confidence):
                continue
            u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            if not (0 <= u < width and 0 <= v < height):
                continue
            col, row = int(u), int(v)
            patch = depth[max(0, row - 2):row + 3, max(0, col - 2):col + 3]
            valid = patch[np.isfinite(patch) & (patch > 0)]
            if not valid.size:
                continue

            depth_mm = float(np.median(valid))
            p_cam = pixel_to_camera_3d(u, v, depth_mm, fx, fy, cx, cy)
            p_base = camera_to_base_3d(p_cam, t_base2gripper, self._t_gripper2camera)
            base_rz = camera_yaw_to_base_rz(yaw, t_base2gripper, self._t_gripper2camera)
            backside = bool(det.get('is_backside', False)) or yolo_cls.lower() == 'product_back'
            info = lookup_catalog(self._catalog, yolo_cls)
            unknown_backside = backside and (not info or yolo_cls.lower() == 'product_back')
            if unknown_backside:
                info = lookup_catalog(self._catalog, 'product_back')
            product = DetectedProduct(
                track_id=f'det_{idx}',
                yolo_class='product_back' if unknown_backside else info.get('yolo_class', yolo_cls),
                product_name='product_back' if unknown_backside else (
                    info.get('name') or info.get('display_name') or yolo_cls
                ),
                is_backside=backside, confidence=confidence, bbox_px=bbox,
                # 파지 깊이 보정은 로봇 실행기가 적용한다. Z는 표면 높이(mm)로 전달한다.
                pose_3d_base=[*p_base.tolist(), base_rz],
                grasp_width_mm=float(info.get('grasp_width_mm', 60.0)),
                status='NeedsReorientation' if backside else 'Normal',
            )
            items.append(dict(depth_mm=depth_mm, xy=(float(p_base[0]), float(p_base[1])), product=product))

        items = assign_depth_layers(items, layer_gap_mm=self._layer_gap_mm)
        products = []
        for item in items:
            product = item['product']
            product.depth_layer = item['depth_layer']
            products.append(product)

        if request.run_vlm_inspection and self._vlm_enabled:
            api_key = os.environ.get('OPENAI_API_KEY')
            if color is not None and api_key:
                self._inspect_products(products, color, api_key)
            else:
                self.get_logger().warning('VLM 검수가 요청되었으나 컬러 프레임 또는 OPENAI_API_KEY가 없어 Unjudgeable로 표시합니다.')
                for product in products:
                    if not product.is_backside:
                        product.status = 'Unjudgeable'

        response.products = products
        response.success = True
        response.error_code = 'OK'
        return response

    def _inspect_products(self, products, color, api_key) -> None:
        height, width = color.shape[:2]
        candidates, crops = [], []
        for product in products:
            if product.is_backside:
                continue
            x1, y1, x2, y2 = product.bbox_px
            product.status = 'Unreadable'
            if x2 > 0 and y2 > 0:
                crop = color[max(0, y1):min(height, y2), max(0, x1):min(width, x2)]
                if crop.size:
                    candidates.append(product)
                    crops.append(crop)

        inspect = partial(
            inspect_crop_with_openai, api_key=api_key, model=self._vlm_model,
            timeout_sec=self._vlm_timeout,
        )
        for product, result in zip(candidates, inspect_crops_parallel(crops, inspect)):
            product.expiration_raw = result['expiration_raw']
            product.status = result['status']
            product.disposal_reason = result['disposal_reason']


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
