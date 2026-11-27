"""매장 전체 의사결정 두뇌 ROS 2 오케스트레이터 노드."""

from __future__ import annotations

import json
from pathlib import Path
import threading
from typing import Any

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from store_interfaces.action import ExecuteRobotTask
from store_interfaces.msg import NodeHealth
from store_interfaces.srv import ScanWorkspace
from store_llm_agent.core.action_domain import ActionDomain, RobotActionStep
from store_llm_agent.core.kg_manager import KGManager
from store_llm_agent.core.llm_parser import LLMParser
from store_llm_agent.core.mode_fsm import ModeFSM
from store_llm_agent.core.paths import find_workspace_root
from store_llm_agent.core.pipeline_engine import (
    StorePipelineEngine,
    load_buffer_slots,
    load_display_tiers,
    load_flip_zone,
    load_zone_bounds,
)
from store_llm_agent.core.scene_explorer import SceneExplorer
from store_llm_agent.core.symbolic_planner import SymbolicPlanner
from store_llm_agent.core.task_formalizer import TaskFormalizer

WS_ROOT = find_workspace_root(__file__)


def parse_text_payload(raw: str, default_source: str) -> tuple[str, str]:
    """단순 문자열 또는 JSON({"text": "...", "source": "..."})을 모두 파싱한다."""
    stripped = (raw or "").strip()
    if stripped.startswith("{"):
        try:
            data = json.loads(stripped)
            if isinstance(data, dict) and "text" in data:
                return str(data["text"]).strip(), str(data.get("source", default_source))
        except json.JSONDecodeError:
            pass
    return stripped, default_source


def parse_vest_payload(raw: str) -> tuple[bool, float]:
    """/store/vision/vest_detected 페이로드(JSON 또는 'true'/'false')를 파싱한다."""
    stripped = (raw or "").strip()
    if stripped.startswith("{"):
        try:
            data = json.loads(stripped)
            if isinstance(data, dict):
                raw_conf = data.get("confidence", data.get("conf", 0.0))
                return bool(data.get("detected", False)), float(0.0 if raw_conf is None else raw_conf)
        except (json.JSONDecodeError, ValueError, TypeError):
            return False, 0.0
    lowered = stripped.lower()
    if lowered in ("true", "1", "yes"):
        return True, 1.0
    return False, 0.0


class OrchestratorNode(Node):
    """ModeFSM + KGManager + LLMParser + RuleValidator + InboundPlanner 통합 ROS 2 노드."""

    def __init__(self) -> None:
        super().__init__("orchestrator")
        self._cb_group = ReentrantCallbackGroup()

        self.declare_parameter("ontology_path", str(WS_ROOT / "config" / "ontology" / "store_ontology.ttl"))
        self.declare_parameter("catalog_path", str(WS_ROOT / "config" / "products_catalog.yaml"))
        self.declare_parameter("zones_path", str(WS_ROOT / "config" / "workspace_zones.yaml"))
        self.declare_parameter("scan_timeout_sec", 15.0)
        self.declare_parameter("action_timeout_sec", 30.0)

        ont_path = self.get_parameter("ontology_path").get_parameter_value().string_value
        cat_path = self.get_parameter("catalog_path").get_parameter_value().string_value
        zones_path = self.get_parameter("zones_path").get_parameter_value().string_value
        self._scan_timeout = self.get_parameter("scan_timeout_sec").get_parameter_value().double_value
        self._action_timeout = self.get_parameter("action_timeout_sec").get_parameter_value().double_value

        self._kg = KGManager()
        self._kg.load_initial_graph(ont_path, cat_path, zones_yaml=zones_path)
        self._fsm = ModeFSM()
        self._parser = LLMParser()
        display_tiers = load_display_tiers(zones_path)
        self._formalizer = TaskFormalizer()
        self._domain = ActionDomain(
            display_tiers=display_tiers,
            buffer_slots=load_buffer_slots(zones_path),
            zone_bounds=load_zone_bounds(zones_path),
            flip_zone=load_flip_zone(zones_path),
        )
        self._planner = SymbolicPlanner()
        self._scene_explorer = SceneExplorer()

        self._engine = StorePipelineEngine(
            kg_manager=self._kg,
            mode_fsm=self._fsm,
            llm_parser=self._parser,
            task_formalizer=self._formalizer,
            action_domain=self._domain,
            symbolic_planner=self._planner,
            scan_fn=self._call_scan_workspace,
            execute_action_fn=self._call_execute_robot_task,
            display_tiers=display_tiers,
            scene_explorer=self._scene_explorer,
        )

        self._cmd_lock = threading.Lock()
        self._active_goal_handle: Any = None
        self._emergency_stopped = False

        # Publishers
        self._pub_mode = self.create_publisher(String, "/store/mode/state", 10)
        self._pub_ui = self.create_publisher(String, "/store/ui_snapshot", 10)
        self._pub_health = self.create_publisher(NodeHealth, "/store/node_health", 10)

        # Subscriptions
        self.create_subscription(
            String, "/store/voice/transcript", self._on_voice_cmd, 10, callback_group=self._cb_group
        )
        self.create_subscription(
            String, "/store/text_cmd", self._on_text_cmd, 10, callback_group=self._cb_group
        )
        self.create_subscription(
            String, "/store/vision/vest_detected", self._on_vest_detected, 10, callback_group=self._cb_group
        )
        self.create_subscription(
            Bool, "/store/emergency_stop", self._on_emergency_stop, 10, callback_group=self._cb_group
        )
        remote_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool, "/store/mode/remote_override", self._on_remote_override, remote_qos, callback_group=self._cb_group
        )

        # Service & Action Clients
        self._scan_client = self.create_client(
            ScanWorkspace, "/store/scan_workspace", callback_group=self._cb_group
        )
        self._robot_client = ActionClient(
            self, ExecuteRobotTask, "/store/execute_robot_task", callback_group=self._cb_group
        )

        self.create_timer(2.0, self._publish_health, callback_group=self._cb_group)
        self._publish_mode()
        self._publish_ui_snapshot()
        self.get_logger().info("orchestrator 노드 초기화 완료")

    def _publish_mode(self) -> None:
        msg = String()
        msg.data = self._fsm.mode
        self._pub_mode.publish(msg)

    def _publish_ui_snapshot(self) -> None:
        msg = String()
        msg.data = json.dumps(self._engine.build_ui_snapshot(), ensure_ascii=False)
        self._pub_ui.publish(msg)

    def _publish_health(self) -> None:
        msg = NodeHealth()
        msg.node_name = "orchestrator"
        msg.alive = True
        msg.state = "WARN" if self._emergency_stopped else "OK"
        msg.message = f"mode={self._fsm.mode}"
        msg.stamp = self.get_clock().now().to_msg()
        self._pub_health.publish(msg)

    def _cancel_active_goal(self, reason: str) -> None:
        handle = self._active_goal_handle
        if handle is not None:
            self.get_logger().warn(f"진행 중인 로봇 Action 취소 요청: {reason}")
            handle.cancel_goal_async()
            self._active_goal_handle = None

    def _on_emergency_stop(self, msg: Bool) -> None:
        self._emergency_stopped = bool(msg.data)
        if self._emergency_stopped:
            self._cancel_active_goal("emergency_stop")

    def _on_remote_override(self, msg: Bool) -> None:
        if msg.data:
            self._fsm.enter_remote()
            self._cancel_active_goal("REMOTE 모드 진입")
        else:
            self._fsm.exit_remote()
        self._publish_mode()
        self._publish_ui_snapshot()

    def _on_vest_detected(self, msg: String) -> None:
        prev_mode = self._fsm.mode
        detected, conf = parse_vest_payload(msg.data)
        new_mode = self._fsm.update_vest_detection(detected, conf)
        if new_mode != prev_mode:
            self.get_logger().info(f"모드 자동 전환: {prev_mode} -> {new_mode}")
            self._publish_mode()
            self._publish_ui_snapshot()

    def _on_voice_cmd(self, msg: String) -> None:
        text, src = parse_text_payload(msg.data, default_source="voice")
        self._dispatch_command(text, src)

    def _on_text_cmd(self, msg: String) -> None:
        text, src = parse_text_payload(msg.data, default_source="text")
        self._dispatch_command(text, src)

    def _dispatch_command(self, text: str, source: str) -> None:
        if not text:
            return
        if self._emergency_stopped:
            self.get_logger().warn("비상정지 상태이므로 명령을 무시합니다.")
            return
        if not self._cmd_lock.acquire(blocking=False):
            self.get_logger().warn("이전 작업 수행 중이므로 새 명령을 건너뜁니다.")
            return
        try:
            trace = self._engine.handle_command(text, source=source)
            self.get_logger().info(
                f"명령 처리 종료: success={trace.success}, actions={[a.action_type for a in trace.executed_actions]}, msg={trace.message}"
            )
            self._publish_mode()
            self._publish_ui_snapshot()
        finally:
            self._cmd_lock.release()

    def _call_scan_workspace(self, zone: str, run_vlm: bool) -> tuple[bool, str, list[Any], list[Any]]:
        if not self._scan_client.wait_for_service(timeout_sec=3.0):
            return False, "ScanWorkspace 서비스 서버를 찾을 수 없습니다.", [], []

        req = ScanWorkspace.Request()
        req.zone = zone
        req.run_vlm_inspection = run_vlm

        done_evt = threading.Event()
        future = self._scan_client.call_async(req)
        future.add_done_callback(lambda _: done_evt.set())
        if not done_evt.wait(timeout=self._scan_timeout):
            return False, "ScanWorkspace 응답 타임아웃", [], []

        resp = future.result()
        if resp is None:
            return False, "ScanWorkspace 빈 응답", [], []
        return bool(resp.success), str(resp.error_code), list(resp.products), list(resp.containers)

    def _call_execute_robot_task(self, step: RobotActionStep) -> tuple[bool, str]:
        if self._emergency_stopped or self._fsm.mode == "REMOTE":
            return False, "CANCELED"
        if not self._robot_client.wait_for_server(timeout_sec=3.0):
            return False, "ExecuteRobotTask 액션 서버를 찾을 수 없습니다."

        goal = ExecuteRobotTask.Goal()
        goal.action_type = step.action_type
        goal.target_pose = [float(v) for v in step.target_pose[:4]]
        goal.dest_pose = [float(v) for v in step.dest_pose[:4]]
        goal.product_category = step.product_category
        goal.grasp_width_mm = float(step.grasp_width_mm)
        goal.grasp_force_n = float(step.grasp_force_n)

        goal_evt = threading.Event()
        send_future = self._robot_client.send_goal_async(goal)
        send_future.add_done_callback(lambda _: goal_evt.set())
        if not goal_evt.wait(timeout=5.0):
            return False, "Action Goal 수락 타임아웃"

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, "Action Goal 거절됨"

        self._active_goal_handle = goal_handle
        res_evt = threading.Event()
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(lambda _: res_evt.set())
        finished = res_evt.wait(timeout=self._action_timeout)
        self._active_goal_handle = None

        if not finished:
            goal_handle.cancel_goal_async()
            return False, "Action 실행 타임아웃"

        wrapped = res_future.result()
        if wrapped is None:
            return False, "Action 결과 누락"
        res = wrapped.result
        if res.stopped_for_safety or not res.success:
            return False, str(res.error_code or "ROBOT_EXEC_FAILED")
        return True, str(res.error_code or "OK")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OrchestratorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
