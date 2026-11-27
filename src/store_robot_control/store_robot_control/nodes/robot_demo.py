import argparse
import json
from collections import deque
from dataclasses import dataclass, replace
from math import isfinite
from pathlib import Path
from time import monotonic

from ..core.safety_guard import SafetyGuard, load_settings


SUITES = ("all", "safety", "motion", "estop")
EMPTY_POSE = (0.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Scenario:
    name: str
    action_type: str
    target_pose: tuple[float, ...] = EMPTY_POSE
    dest_pose: tuple[float, ...] = EMPTY_POSE
    expected_error_code: str = "OK"
    expected_stopped_for_safety: bool = False
    emergency_stop: bool = False
    product_category: str = "paper_box"


def build_scenarios(workspace_cfg, suite="all", *, target_pose=None, dest_pose=None):
    if suite not in SUITES:
        raise ValueError(f"알 수 없는 시나리오 모음 ({suite})")
    guard = SafetyGuard(workspace_cfg)
    zones = workspace_cfg["zones"]
    display = zones["display_area"]
    x_min, x_max = display["x_range"]
    y_min, y_max = display["y_range"]
    y = (y_min + y_max) / 2
    z = max(40.0, guard.z_min_mm)

    def checked(pose, label):
        result = guard.validate_pose(pose)
        if not result.ok:
            raise ValueError(f"{label} 좌표 오류 ({result.error_code}) {result.message}")
        return tuple(result.clamped_pose)

    quarter = (x_max - x_min) / 4
    a = checked(target_pose if target_pose is not None else (x_min + quarter, y, z, 0.0), "테이블 A")
    b = checked(dest_pose if dest_pose is not None else (x_max - quarter, y, z, 0.0), "테이블 B")

    scenarios = []
    if suite in {"all", "safety"}:
        low = (*a[:2], guard.z_min_mm - 5.0, a[3])
        high = (*a[:2], guard.bounds["z_max"] + 20.0, a[3])
        scenarios.extend([
            Scenario("z_min_violation", "pick_place", low, b, "Z_MIN_VIOLATION", True),
            Scenario("out_of_reach", "pick_place", high, b, "OUT_OF_REACH", True),
            Scenario("invalid_action", "give_expired_box", a, b, "INVALID_ACTION"),
        ])
    if suite in {"all", "motion"}:
        buffer = checked(zones["buffer_slots"][0], "첫 번째 버퍼")
        scenarios.extend([
            Scenario("home", "home"),
            Scenario("pick_place", "pick_place", a, b),
            Scenario("move_to_buffer", "move_to_buffer", a, buffer),
            Scenario("restore_from_buffer", "restore_from_buffer", buffer, a),
            Scenario("reorient", "reorient", a),
            # 스캔 자세의 J5는 작업 공간 설정에서 피벗 시작 각도 상한을 넘을 수 있습니다.
            Scenario("scan_pose", "scan_pose"),
        ])
    if suite in {"all", "estop"}:
        scenarios.append(Scenario("estop", "pick_place", a, b, "CANCELED", True, emergency_stop=True))
    return scenarios


def summarize_blackbox(log_dir, limit=12, *, task_ids=None):
    path = Path(log_dir) / "robot_blackbox.jsonl"
    if not path.is_file():
        return []
    recent = deque(maxlen=limit)
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            try:
                entry = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not isinstance(entry, dict) or not {"action_type", "error_code"} <= entry.keys():
                continue
            if task_ids is None or entry.get("task_id") in task_ids:
                recent.append(entry)
    return list(recent)


class DemoRunner:
    def __init__(self, node, timeout):
        import rclpy
        from rclpy.action import ActionClient
        from std_msgs.msg import Bool
        from store_interfaces.action import ExecuteRobotTask
        from store_interfaces.msg import NodeHealth

        from .mock_robot import ACTION_NAME

        self.node = node
        self.timeout = timeout
        self.spin_once = rclpy.spin_once
        self.goal_type = ExecuteRobotTask.Goal
        self.bool_type = Bool
        self.client = ActionClient(node, ExecuteRobotTask, ACTION_NAME)
        self.stop_pub = node.create_publisher(Bool, "/store/emergency_stop", 10)
        self.health = {}
        self.task_ids = set()
        self.stop_active = False
        self.health_sub = node.create_subscription(NodeHealth, "/store/node_health", self._on_health, 10)

    def _on_health(self, message):
        if message.node_name in {"executor_node", "mock_robot"}:
            self.health[message.node_name] = (monotonic(), message)

    def _wait(self, ready, label, *, tick=None):
        deadline = monotonic() + self.timeout
        while not ready():
            remaining = deadline - monotonic()
            if remaining <= 0:
                raise TimeoutError(f"{label} 제한 시간 초과")
            if tick is not None:
                tick()
            self.spin_once(self.node, timeout_sec=min(remaining, 0.05))

    def _action_server_nodes(self):
        names = {name.lstrip("/") for name in self.node.get_node_names()}
        return sorted(names & {"executor_node", "mock_robot"})

    def connect(self):
        from .mock_robot import ACTION_NAME

        self._wait(self.client.server_is_ready, "액션 서버 연결")
        self._wait(lambda: bool(self._action_server_nodes()), "액션 서버 노드 탐지")
        servers = self._action_server_nodes()
        status_pubs = self.node.count_publishers(f"{ACTION_NAME}/_action/status")
        if len(servers) > 1 or status_pubs > 1:
            raise RuntimeError("실행기와 모의 로봇(또는 중복 액션 서버)이 동시에 감지되었습니다.")
        self.server_name = servers[0]
        self._wait(lambda: self.server_name in self.health, f"{self.server_name} 상태 수신")
        if len(self.health) > 1 or len(self._action_server_nodes()) > 1:
            raise RuntimeError("실행기와 모의 로봇이 동시에 감지되었습니다.")
        _, health = self.health[self.server_name]
        if not health.alive or health.state != "OK":
            raise RuntimeError(f"로봇이 준비되지 않았습니다 ({health.state}, {health.message})")
        if self.server_name == "executor_node" and "dry_run=" not in health.message:
            raise RuntimeError(f"실행기 모드를 확인할 수 없습니다 ({health.message})")
        return self.server_name == "executor_node" and "dry_run=false" in health.message

    def _await_health(self, after, state):
        def matches():
            received, health = self.health[self.server_name]
            return received >= after and health.alive and health.state == state

        self._wait(matches, f"로봇 상태 {state} 수신")

    def _publish_stop(self, active):
        published_at = monotonic()
        self.stop_pub.publish(self.bool_type(data=active))
        self.stop_active = active
        return published_at

    def _send(self, scenario, feedback_callback=None, tick=None):
        goal = self.goal_type(
            action_type=scenario.action_type, product_category=scenario.product_category,
            target_pose=scenario.target_pose, dest_pose=scenario.dest_pose,
        )
        accepted = self.client.send_goal_async(goal, feedback_callback=feedback_callback)
        self._wait(accepted.done, "목표 수락")
        handle = accepted.result()
        if not handle.accepted:
            raise RuntimeError("목표가 거부되었습니다.")
        self.task_ids.add(bytes(handle.goal_id.uuid).hex())
        future = handle.get_result_async()
        try:
            self._wait(future.done, "작업 결과 수신", tick=tick)
        except TimeoutError as exc:
            # 제한 시간 초과 시 실행 중인 목표를 취소하고 정지 여부를 확인합니다.
            cancel_future = handle.cancel_goal_async()
            deadline = monotonic() + 3.0
            while not future.done() and monotonic() < deadline:
                self.spin_once(self.node, timeout_sec=0.05)
            if future.done():
                raise TimeoutError("작업 결과 수신 제한 시간 초과 (실행 중 목표 취소 및 정지 확인)") from exc
            _ = cancel_future
            raise TimeoutError(
                "작업 결과 수신 제한 시간 초과 (취소를 요청했으나 정지 미확인 — 로봇 상태를 확인하세요)"
            ) from exc
        return future.result().result

    @staticmethod
    def _check_result(scenario, result):
        if (result.error_code != scenario.expected_error_code
                or result.stopped_for_safety != scenario.expected_stopped_for_safety
                or result.success != (scenario.expected_error_code == "OK")):
            raise RuntimeError(
                f"예상 {scenario.expected_error_code} (안전 정지 {scenario.expected_stopped_for_safety}), "
                f"실제 {result.error_code} (안전 정지 {result.stopped_for_safety}, 성공 {result.success})"
            )

    def run(self, scenario):
        if not scenario.emergency_stop:
            result = self._send(scenario)
            self._check_result(scenario, result)
            if scenario.name == "scan_pose":
                # 교시한 스캔 자세에서 이탈해 첫 파지 동작을 홈 자세에서 시작합니다.
                home = Scenario("home", "home")
                self._check_result(home, self._send(home))
                return "OK / home 복귀 OK"
            return result.error_code

        self._wait(lambda: self.stop_pub.get_subscription_count() > 0, "비상 정지 토픽 연결")
        stop_at = None
        fallback_at = monotonic() + 0.15

        def stop():
            nonlocal stop_at
            if stop_at is None:
                stop_at = self._publish_stop(True)

        def fallback():
            if monotonic() >= fallback_at:
                stop()

        result = self._send(scenario, feedback_callback=lambda _: stop(), tick=fallback)
        if stop_at is None:
            raise RuntimeError("비상 정지 발행 전에 작업이 종료되었습니다.")
        self._await_health(stop_at, "ERROR")
        self._check_result(scenario, result)
        cleared_at = self._publish_stop(False)
        self._await_health(cleared_at, "OK")
        home = Scenario("home", "home")
        self._check_result(home, self._send(home))
        return "CANCELED / ERROR / home OK"


def main(argv=None):
    parser = argparse.ArgumentParser(description="매장 로봇 동작 및 안전 시나리오 검증")
    parser.add_argument("--suite", choices=SUITES, default="all")
    parser.add_argument("--config-dir", type=Path)
    parser.add_argument("--log-dir", type=Path)
    parser.add_argument("--yes", action="store_true", help="실물 로봇 실행 확인 생략")
    parser.add_argument("--timeout-sec", type=float, default=60.0)
    parser.add_argument("--target-pose", type=float, nargs=4, metavar=("X", "Y", "Z", "RZ"))
    parser.add_argument("--dest-pose", type=float, nargs=4, metavar=("X", "Y", "Z", "RZ"))
    args = parser.parse_args(argv)
    if not isfinite(args.timeout_sec) or args.timeout_sec <= 0:
        print("실패 | 제한 시간은 유한한 양수여야 합니다.")
        return 1

    import rclpy

    from .mock_robot import find_config_dir

    rows = []
    runner = None
    try:
        config_dir = args.config_dir or find_config_dir()
        scenarios = build_scenarios(
            load_settings(config_dir / "workspace_zones.yaml"), args.suite,
            target_pose=args.target_pose, dest_pose=args.dest_pose,
        )
        rclpy.init(args=[])
        node = None
        try:
            node = rclpy.create_node("robot_demo")
            runner = DemoRunner(node, args.timeout_sec)
            real_robot = runner.connect()
            if real_robot and not args.yes:
                print("실물 로봇이 움직입니다. 작업 공간과 시험 물품을 확인하세요.")
                if args.suite in {"all", "motion"}:
                    print("테이블 A에 종이상자 하나를 놓고, 테이블 B와 첫 번째 버퍼를 비워 주세요.")
                if input("실행하려면 yes를 입력하세요. ").strip().lower() != "yes":
                    raise RuntimeError("사용자가 실물 실행을 취소했습니다.")
            failed = False
            for scenario in scenarios:
                if failed:
                    rows.append((scenario.name, "건너뜀", "앞선 시나리오 실패"))
                    continue
                try:
                    detail = runner.run(scenario)
                    if real_robot and scenario.name == "pick_place":
                        # 다음 버퍼 검증도 테이블 A에서 시작하므로 시험 물품을 돌려놓습니다.
                        reset = replace(scenario, target_pose=scenario.dest_pose, dest_pose=scenario.target_pose)
                        runner.run(reset)
                        detail += " / A 복귀 OK"
                except (RuntimeError, TimeoutError) as exc:
                    rows.append((scenario.name, "실패", str(exc)))
                    failed = True
                else:
                    rows.append((scenario.name, "통과", detail))
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown()
    except (OSError, ValueError, KeyError, IndexError, EOFError, RuntimeError, TimeoutError, KeyboardInterrupt) as exc:
        rows.append(("robot_demo", "실패", str(exc) or "실행 중단"))

    print("\n시나리오            | 판정 | 결과")
    print("--------------------|------|---------------------------")
    for name, status, detail in rows:
        print(f"{name:<19} | {status} | {detail}")
    if runner and runner.stop_active:
        print("비상 정지가 발행된 상태입니다. 로봇 상태를 확인하세요.")
    if args.log_dir and args.log_dir.is_dir():
        try:
            entries = summarize_blackbox(args.log_dir, task_ids=runner.task_ids if runner else set())
        except OSError as exc:
            print(f"블랙박스 읽기 실패 ({exc})")
        else:
            print(f"블랙박스 기록 {len(entries)}건")
            for entry in entries:
                print(f"  {entry['action_type']} | {entry['error_code']}")
    return 0 if rows and all(status == "통과" for _, status, _ in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
