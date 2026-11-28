from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from store_interfaces.msg import NodeHealth

from store_vision.nodes.image_utils import numpy_to_image_msg


FRAME_ID = 'camera_color_optical_frame'


class RealSenseNode(Node):
    """D435i 컬러·정렬 깊이·카메라 정보를 발행하며 테스트 이미지도 지원한다."""

    def __init__(self) -> None:
        super().__init__('realsense_node')
        self.declare_parameter('mock_mode', True)
        self.declare_parameter('test_image_path', '')
        self.declare_parameter('mock_depth_mm', 500)
        self.declare_parameter('publish_hz', 2.0)
        self.declare_parameter('heartbeat_period_sec', 2.0)

        self.mock_mode = self.get_parameter('mock_mode').get_parameter_value().bool_value
        self.test_image_path = (
            self.get_parameter('test_image_path').get_parameter_value().string_value
        )
        self.mock_depth_mm = (
            self.get_parameter('mock_depth_mm').get_parameter_value().integer_value
        )
        publish_hz = self.get_parameter('publish_hz').get_parameter_value().double_value

        self._health_pub = self.create_publisher(NodeHealth, '/store/node_health', 10)
        self._hardware_error = ''
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(
            Image, '/camera/aligned_depth_to_color/image_raw', 10
        )
        self.info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)

        self._mock_images: list[tuple[str, np.ndarray]] = []
        self._mock_idx = 0
        self._pipeline = None
        self._intrinsics: tuple[float, float, float, float] | None = None

        if self.mock_mode:
            self._load_mock_images()
            period = 1.0 / max(publish_hz, 0.2)
            self.create_timer(period, self._publish_mock_frame)
            self.get_logger().info(
                f'realsense_node started in mock_mode ({len(self._mock_images)} image(s), '
                f'depth={self.mock_depth_mm}mm)'
            )
        else:
            self._init_realsense()
        self.create_timer(self.get_parameter('heartbeat_period_sec').value, self.publish_heartbeat)

    def _load_mock_images(self) -> None:
        candidates: list[Path] = []
        if self.test_image_path:
            p = Path(self.test_image_path)
            if p.is_file():
                candidates.append(p)
            elif p.is_dir():
                candidates.extend(sorted(p.glob('*.jpg')) + sorted(p.glob('*.png')))
        else:
            test_dir = Path.home() / 'datasets' / 'store_5classes' / 'test' / 'images'
            if test_dir.exists():
                pep = sorted(test_dir.glob('pepero_*.jpg'))[:2]
                pos = sorted(test_dir.glob('postick_*.jpg'))[:2]
                drk = sorted(test_dir.glob('aloe_*.jpg'))[:2]
                candidates.extend(pep + pos + drk)

        for path in candidates:
            img = cv2.imread(str(path))
            if img is not None:
                self._mock_images.append((path.name, img))

        if not self._mock_images:
            synthetic = np.full((480, 640, 3), 180, dtype=np.uint8)
            cv2.putText(
                synthetic,
                'MOCK_CAMERA_NO_IMAGE',
                (120, 240),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (20, 20, 20),
                2,
            )
            self._mock_images.append(('synthetic.png', synthetic))

    @staticmethod
    def _build_camera_info(
        width: int, height: int, fx: float, fy: float, cx: float, cy: float, stamp
    ) -> CameraInfo:
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = FRAME_ID
        info.width = width
        info.height = height
        info.distortion_model = 'plumb_bob'
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _publish_mock_frame(self) -> None:
        _, bgr = self._mock_images[self._mock_idx % len(self._mock_images)]
        self._mock_idx += 1

        h, w = bgr.shape[:2]
        depth_mm = np.full((h, w), int(self.mock_depth_mm), dtype=np.uint16)

        stamp = self.get_clock().now().to_msg()
        self._publish_images(bgr, depth_mm, stamp, (615.0, 615.0, w / 2.0, h / 2.0))

    def _publish_images(self, bgr, depth_mm, stamp, intrinsics) -> None:
        height, width = bgr.shape[:2]
        self.color_pub.publish(numpy_to_image_msg(bgr, 'bgr8', stamp, FRAME_ID))
        self.depth_pub.publish(numpy_to_image_msg(depth_mm, '16UC1', stamp, FRAME_ID))
        self.info_pub.publish(self._build_camera_info(width, height, *intrinsics, stamp))

    def _init_realsense(self) -> None:
        try:
            import pyrealsense2 as rs
        except ImportError:
            self._hardware_error = (
                'pyrealsense2를 불러올 수 없습니다. realsense2_camera를 실행하거나 '
                'mock_mode:=true로 모의 영상을 사용하세요.'
            )
            self.get_logger().error(self._hardware_error)
            return

        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self._align = rs.align(rs.stream.color)
        try:
            profile = pipeline.start(config)
        except RuntimeError as exc:
            self._hardware_error = f'RealSense 시작 실패. {exc}'
            self.get_logger().error(self._hardware_error)
            return
        self._pipeline = pipeline
        self._depth_to_mm = profile.get_device().first_depth_sensor().get_depth_scale() * 1000.0
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self._intrinsics = (intr.fx, intr.fy, intr.ppx, intr.ppy)
        self.create_timer(1.0 / 30.0, self._publish_hw_frame)
        self.get_logger().info('realsense_node started with D435i hardware')

    def _publish_hw_frame(self) -> None:
        if self._pipeline is None:
            return
        frames = self._pipeline.poll_for_frames()
        if not frames:
            return
        aligned = self._align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            return

        bgr = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())
        h, w = bgr.shape[:2]
        stamp = self.get_clock().now().to_msg()

        if not np.isclose(self._depth_to_mm, 1.0):
            depth = np.clip(np.rint(depth * self._depth_to_mm), 0, 65535).astype(np.uint16)
        intrinsics = self._intrinsics or (615.0, 615.0, w / 2.0, h / 2.0)
        self._publish_images(bgr, depth, stamp, intrinsics)

    def publish_heartbeat(self) -> None:
        if self._hardware_error:
            state, message = 'ERROR', self._hardware_error
        elif self.mock_mode:
            state, message = 'OK', '모의 RealSense 프레임을 발행 중입니다.'
        else:
            state, message = 'OK', 'RealSense 하드웨어에서 프레임을 수신 중입니다.'
        self._health_pub.publish(NodeHealth(
            node_name=self.get_name(), alive=True, state=state, message=message,
            stamp=self.get_clock().now().to_msg(),
        ))

    def destroy_node(self):
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except RuntimeError as exc:
                self.get_logger().warning(f'RealSense 종료 중 오류가 발생했습니다. {exc}')
            finally:
                self._pipeline = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealSenseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
