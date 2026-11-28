from time import monotonic

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from store_interfaces.msg import NodeHealth

from store_vision.nodes.image_utils import numpy_to_image_msg


class WebcamNode(Node):
    """조끼 검출용 USB 영상 발행과 원격 모드의 카메라 인계를 담당한다."""

    def __init__(self) -> None:
        super().__init__('webcam_node')
        self.declare_parameters('', [
            ('camera_index', 0), ('width', 640), ('height', 480),
            ('fps', 15.0), ('mock_mode', False), ('heartbeat_period_sec', 2.0),
        ])
        param = self.get_parameter
        self._camera_index = param('camera_index').value
        self._width = param('width').value
        self._height = param('height').value
        self._fps = param('fps').value
        self._mock = param('mock_mode').value
        self._mock_frame = np.zeros((self._height, self._width, 3), dtype=np.uint8) if self._mock else None
        self._capture = None
        self._mode = ''
        self._override = False
        self._retry_at = 0.0

        self.image_pub = self.create_publisher(Image, '/webcam/image_raw', 10)
        self._health_pub = self.create_publisher(NodeHealth, '/store/node_health', 10)
        self.create_subscription(String, '/store/mode/state', self.on_mode_state, 10)
        override_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, '/store/mode/remote_override', self.on_override, override_qos)
        self.create_timer(1.0 / self._fps, self._publish_frame)
        self.create_timer(param('heartbeat_period_sec').value, self.publish_heartbeat)

    @property
    def _remote(self) -> bool:
        return self._override or self._mode == 'REMOTE'

    def on_mode_state(self, msg: String) -> None:
        was_remote = self._remote
        self._mode = msg.data.strip().upper()
        self._update_capture(was_remote)

    def on_override(self, msg: Bool) -> None:
        was_remote = self._remote
        self._override = msg.data
        self._update_capture(was_remote)

    def _update_capture(self, was_remote: bool) -> None:
        if self._remote:
            self._release_capture()
        elif was_remote:
            self._retry_at = 0.0

    def _release_capture(self) -> None:
        if self._capture is not None:
            self._capture.release()
            self._capture = None

    def _publish_frame(self) -> None:
        if self._remote:
            return
        frame = self._mock_frame
        if not self._mock:
            if self._capture is None:
                if monotonic() < self._retry_at:
                    return
                self._retry_at = monotonic() + 1.0
                capture = cv2.VideoCapture(self._camera_index)
                if not capture.isOpened():
                    capture.release()
                    return
                self._capture = capture
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
                capture.set(cv2.CAP_PROP_FPS, self._fps)
            ok, frame = self._capture.read()
            if not ok:
                self._release_capture()
                self._retry_at = monotonic() + 1.0
                return

        stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(numpy_to_image_msg(frame, 'bgr8', stamp, 'webcam_optical_frame'))

    def publish_heartbeat(self) -> None:
        if self._remote:
            state, message = 'REMOTE', '원격 조작에 웹캠을 인계했습니다.'
        elif self._mock:
            state, message = 'OK', '모의 웹캠 영상을 발행 중입니다.'
        elif self._capture is None:
            state, message = 'WAITING', f'웹캠 장치 {self._camera_index} 연결을 기다리는 중입니다.'
        else:
            state, message = 'OK', f'웹캠 장치 {self._camera_index} 영상을 발행 중입니다.'
        self._health_pub.publish(NodeHealth(
            node_name=self.get_name(), alive=True, state=state, message=message,
            stamp=self.get_clock().now().to_msg(),
        ))

    def destroy_node(self):
        self._release_capture()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebcamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
