"""마이크 호출어/PTT 감지 후 OpenAI whisper-1 STT 결과를 발행하는 ROS 2 노드."""

from __future__ import annotations

import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from store_interfaces.msg import NodeHealth
from store_llm_agent.core.voice_stt import DEFAULT_MODEL_PATH, WakeWordDetector, WhisperSTT


class VoiceNode(Node):
    """호출어(openWakeWord) 및 PTT 트리거 기반 음성 인식 노드."""

    def __init__(self) -> None:
        super().__init__("voice_node")
        self.declare_parameter("model_path", str(DEFAULT_MODEL_PATH))
        self.declare_parameter("wakeword_threshold", 0.3)
        self.declare_parameter("record_duration_sec", 5.0)
        self.declare_parameter("enable_wakeword", True)

        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        threshold = self.get_parameter("wakeword_threshold").get_parameter_value().double_value
        duration = self.get_parameter("record_duration_sec").get_parameter_value().double_value
        self._enable_wakeword = self.get_parameter("enable_wakeword").get_parameter_value().bool_value

        self._stt = WhisperSTT(duration_sec=duration)
        self._detector = WakeWordDetector(model_path=model_path, threshold=threshold)
        self._lock = threading.Lock()
        self._running = True
        self._health_state = "OK"
        self._health_msg = "voice_node ready"

        self._pub_transcript = self.create_publisher(String, "/store/voice/transcript", 10)
        self._pub_health = self.create_publisher(NodeHealth, "/store/node_health", 10)
        self.create_subscription(Bool, "/store/voice/ptt_trigger", self._on_ptt, 10)
        self.create_timer(2.0, self._publish_health)

        if self._enable_wakeword:
            self._thread = threading.Thread(target=self._wakeword_loop, daemon=True)
            self._thread.start()

    def _publish_health(self) -> None:
        msg = NodeHealth()
        msg.node_name = "voice_node"
        msg.alive = True
        msg.state = self._health_state
        msg.message = self._health_msg
        msg.stamp = self.get_clock().now().to_msg()
        self._pub_health.publish(msg)

    def _publish_transcript(self, text: str, source: str) -> None:
        cleaned = (text or "").strip()
        if not cleaned:
            return
        payload = json.dumps({"text": cleaned, "source": source}, ensure_ascii=False)
        msg = String()
        msg.data = payload
        self._pub_transcript.publish(msg)
        self.get_logger().info(f"음성 발행({source}): {cleaned}")

    def _on_ptt(self, msg: Bool) -> None:
        if not msg.data:
            return
        threading.Thread(target=self._capture_and_publish, args=("ptt",), daemon=True).start()

    def _capture_and_publish(self, source: str) -> None:
        if not self._lock.acquire(blocking=False):
            return
        try:
            text = self._stt.speech2text()
            self._publish_transcript(text, source=source)
        except Exception as exc:
            self.get_logger().error(f"STT 처리 오류: {exc}")
        finally:
            self._lock.release()

    def _wakeword_loop(self) -> None:
        try:
            self._detector.open()
            self.get_logger().info("호출어 감지기 시작됨 (hello_rokey)")
        except Exception as exc:
            self._health_state = "WARN"
            self._health_msg = f"WakeWord 비활성(PTT 전용): {exc}"
            self.get_logger().warn(self._health_msg)
            return

        try:
            while self._running and rclpy.ok():
                if self._detector.is_wakeup():
                    self.get_logger().info("호출어 감지 → 음성 녹음 시작")
                    self._capture_and_publish("wakeword")
        except Exception as exc:
            self._health_state = "ERROR"
            self._health_msg = f"WakeWord 루프 중단: {exc}"
            self.get_logger().error(self._health_msg)
        finally:
            self._detector.close()

    def destroy_node(self) -> bool:
        self._running = False
        self._detector.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
