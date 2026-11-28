import json
from threading import Event, Thread

import rclpy
import uvicorn
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from store_interfaces.msg import NodeHealth
from store_interfaces.srv import VerifyRemoteKey

from .core.state_store import FrameBuffer, StateStore, ros_image_to_jpeg
from .server import create_app


class WebUiNode(Node):
    def __init__(self):
        super().__init__("web_ui_node")
        self._host = self.declare_parameter("host", "0.0.0.0").value
        self._port = self.declare_parameter("port", 8080).value
        stale_timeout = self.declare_parameter("stale_timeout_sec", 6.0).value
        self._jpeg_quality = self.declare_parameter("jpeg_quality", 75).value
        self._max_width = self.declare_parameter("stream_max_width", 640).value
        heartbeat_period = self.declare_parameter("heartbeat_period_sec", 2.0).value
        self._state = StateStore(stale_timeout)
        self._frames = FrameBuffer()
        self._callbacks = ReentrantCallbackGroup()
        self._text_pub = self.create_publisher(String, "/store/text_cmd", 10)
        self._stop_pub = self.create_publisher(Bool, "/store/emergency_stop", 10)
        self._hand_pub = self.create_publisher(String, "/store/remote/hand_cmd", 1)
        self._health_pub = self.create_publisher(NodeHealth, "/store/node_health", 10)
        self._verify_client = self.create_client(
            VerifyRemoteKey, "/store/verify_remote_key", callback_group=self._callbacks,
        )

        override_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        for kind, topic, handler, qos in (
            (String, "/store/ui_snapshot", lambda msg: self._state.update_snapshot(msg.data), 10),
            (NodeHealth, "/store/node_health", self._on_health, 10),
            (String, "/store/mode/state", lambda msg: self._state.set_mode(msg.data), 10),
            (Bool, "/store/mode/remote_override",
             lambda msg: self._state.set_remote_override(msg.data), override_qos),
            (Bool, "/store/emergency_stop", lambda msg: self._state.set_emergency_stop(msg.data), 10),
        ):
            self.create_subscription(kind, topic, handler, qos)
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for name, topic in (
            ("realsense", "/camera/color/image_raw"), ("webcam", "/webcam/image_raw"),
        ):
            self.create_subscription(
                Image, topic, lambda msg, camera=name: self._on_image(camera, msg),
                image_qos, callback_group=MutuallyExclusiveCallbackGroup(),
            )
        self.create_timer(heartbeat_period, self._heartbeat, callback_group=self._callbacks)

        app = create_app(self._state, self, self._frames)
        self._server = uvicorn.Server(uvicorn.Config(
            app, host=self._host, port=self._port, log_level="warning",
            access_log=False, timeout_graceful_shutdown=1,
        ))
        self._http_thread = Thread(target=self._run_http, name="store-web-ui", daemon=True)

    def start_http(self):
        self._http_thread.start()

    def _run_http(self):
        try:
            self._server.run()
        except (SystemExit, OSError):
            self.get_logger().error(f"웹 서버를 시작할 수 없습니다 ({self._host}:{self._port}).")

    def stop_http(self):
        self._server.should_exit = True
        if self._http_thread.ident is not None:
            self._http_thread.join(timeout=3.0)

    def _heartbeat(self):
        self._health_pub.publish(NodeHealth(
            node_name=self.get_name(), alive=True, state="OK",
            message=f"http={self._host}:{self._port}", stamp=self.get_clock().now().to_msg(),
        ))

    def _on_health(self, msg):
        self._state.update_node_health(msg.node_name, msg.alive, msg.state, msg.message, {
            "sec": msg.stamp.sec, "nanosec": msg.stamp.nanosec,
        })

    def _on_image(self, name, msg):
        jpeg = ros_image_to_jpeg(
            msg.width, msg.height, msg.encoding, msg.data, self._max_width,
            self._jpeg_quality, step=msg.step,
        )
        if jpeg is not None:
            self._frames.update(name, jpeg)

    def publish_text_cmd(self, text):
        self._text_pub.publish(String(data=text))

    def publish_emergency_stop(self, stop):
        self._stop_pub.publish(Bool(data=stop))

    def publish_hand_cmd(self, dx_px, dy_px, dz_px, gripper_closed=None, stop=False):
        command = {"dx_px": dx_px, "dy_px": dy_px, "dz_px": dz_px, "stop": stop}
        # 기존 원격 노드는 그리퍼 유지 시 필드 생략을 요구합니다.
        if gripper_closed is not None:
            command["gripper_closed"] = gripper_closed
        self._hand_pub.publish(String(data=json.dumps(command, allow_nan=False)))

    def verify_remote_key(self, admin_key, enable_remote):
        if not self._verify_client.service_is_ready():
            return False, "원격 키 검증 서비스가 준비되지 않았습니다."
        try:
            future = self._verify_client.call_async(
                VerifyRemoteKey.Request(admin_key=admin_key, enable_remote=enable_remote),
            )
            completed = Event()
            future.add_done_callback(lambda _: completed.set())
            if not completed.wait(timeout=3.0):
                self._verify_client.remove_pending_request(future)
                future.cancel()
                return False, "원격 키 검증 응답 시간이 초과되었습니다. 현재 원격 모드 상태를 확인하세요."
            response = future.result()
        except Exception:
            return False, "원격 키 검증 서비스 호출에 실패했습니다."
        return response.granted, response.reason.replace(admin_key, "****") if admin_key else response.reason


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=4)
    node = None
    try:
        node = WebUiNode()
        executor.add_node(node)
        node.start_http()
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop_http()
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
