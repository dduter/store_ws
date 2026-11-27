import os
from secrets import compare_digest
from math import isfinite
from pathlib import Path
from threading import Condition
from time import monotonic

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from store_interfaces.action import ExecuteRobotTask
from store_interfaces.msg import NodeHealth
from store_interfaces.srv import VerifyRemoteKey

from ..core.blackbox_log import BlackboxLogger
from ..core.grasp_planner import GraspPlanner
from ..core.reorient_motion import (
    MotionWaypoint, ReorientPlanningError, plan_gentle_flip, plan_reorient, plan_two_stage_flip,
)
from ..core.safety_guard import SafetyGuard, load_settings


ACTION_NAME = "/store/execute_robot_task"
TRANSFER_ACTIONS = {"pick_place", "move_to_buffer", "restore_from_buffer"}
REMOTE_OVERRIDE_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


def find_config_dir():
    for start in (Path.cwd(), Path(__file__).resolve().parent, Path.home() / "store_ws"):
        for parent in (start, *start.parents):
            candidate = parent / "config"
            if (candidate / "workspace_zones.yaml").is_file():
                return candidate
    return Path.cwd() / "config"


class MockRobot(Node):
    def __init__(self):
        super().__init__("mock_robot")
        config_dir = Path(self.declare_parameter("config_dir", str(find_config_dir())).value).resolve()
        settings = load_settings(config_dir / "control_params.yaml")
        self._reorient = settings.get("reorient", {})
        self._delay = self.declare_parameter(
            "step_delay_sec", float(settings["mock_robot"]["step_delay_sec"]),
        ).value
        if not isfinite(self._delay) or self._delay < 0:
            raise ValueError("step_delay_sec는 유한한 0 이상의 수여야 합니다.")
        blackbox_cfg = settings.get("blackbox", {})
        log_dir = self.declare_parameter(
            "log_dir", str(config_dir.parent / blackbox_cfg.get("log_dir", "data/logs")),
        ).value
        self._guard = SafetyGuard(config_dir / "workspace_zones.yaml")
        self._grasp = GraspPlanner(config_dir / "products_catalog.yaml")
        self._blackbox = BlackboxLogger(log_dir) if blackbox_cfg.get("enabled", True) else None
        self._state = Condition()
        self._busy = False
        self._active_goal = None
        self._remote = False
        self._stop_reason = None
        self._pose, self._j6 = self._fixed_pose("home")
        self._callback_group = ReentrantCallbackGroup()
        self._action = ActionServer(
            self, ExecuteRobotTask, ACTION_NAME, execute_callback=self.execute_callback,
            goal_callback=self.goal_callback, handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback, callback_group=self._callback_group,
        )
        self._stop_sub = self.create_subscription(
            Bool, "/store/emergency_stop", self._on_emergency_stop, 10,
            callback_group=self._callback_group,
        )
        self._remote_service = self.create_service(
            VerifyRemoteKey, "/store/verify_remote_key", self._verify_remote_key,
            callback_group=self._callback_group,
        )
        self._remote_pub = self.create_publisher(Bool, "/store/mode/remote_override", REMOTE_OVERRIDE_QOS)
        self._remote_pub.publish(Bool(data=False))
        self._health = self.create_publisher(NodeHealth, "/store/node_health", 10)
        self._timer = self.create_timer(
            settings["health"]["heartbeat_period_sec"], self._publish_health,
            callback_group=self._callback_group,
        )

    def _fixed_pose(self, action):
        poses = self._guard.poses
        x, y, z, _, _, rz = poses[f"{action}_cartesian"]
        return [x, y, z, rz], poses[f"{action}_joint_deg"][5]

    def _publish_health(self):
        stopped = self._guard.is_emergency_stopped()
        self._health.publish(NodeHealth(
            node_name="mock_robot", alive=True, state="ERROR" if stopped else "OK",
            message="비상 정지 활성" if stopped else "",
            stamp=self.get_clock().now().to_msg(),
        ))

    def _on_emergency_stop(self, message):
        with self._state:
            self._guard.set_emergency_stop(message.data)
            if message.data and self._busy:
                self._stop_reason = "emergency"
            self._state.notify_all()

    def _verify_remote_key(self, request, response):
        key = os.environ.get("REMOTE_ADMIN_KEY", "")
        response.granted = bool(key) and compare_digest(key.encode(), request.admin_key.encode())
        if not response.granted:
            response.reason = "원격 모드 인증에 실패했습니다."
            return response
        with self._state:
            if request.enable_remote and not self._remote and self._busy:
                if self._stop_reason is None:
                    self._stop_reason = "remote"
            self._remote = request.enable_remote
            self._state.notify_all()
        self._remote_pub.publish(Bool(data=self._remote))
        response.reason = "원격 모드 활성" if self._remote else "원격 모드 해제"
        return response

    def goal_callback(self, _request):
        with self._state:
            if self._busy:
                return GoalResponse.REJECT
            return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._state:
            if not self._busy:
                self._busy = True
                self._active_goal = goal_handle
                self._stop_reason = None
                if self._guard.is_emergency_stopped():
                    self._stop_reason = "emergency"
                elif self._remote:
                    self._stop_reason = "remote"
        try:
            goal_handle.execute()
        except Exception:
            with self._state:
                if self._active_goal is goal_handle:
                    self._active_goal = None
                    self._busy = False
            raise

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _checked_pose(self, pose):
        check = self._guard.validate_pose(pose)
        if not check.ok:
            raise ReorientPlanningError(check.error_code, check.message)
        return check.clamped_pose

    def _waypoints(self, action, target, destination, grasp_plan):
        if action in {"home", "scan_pose"}:
            pose, j6 = self._fixed_pose(action)
            if not self._guard.check_j6_limit(j6):
                raise ReorientPlanningError("J6_LIMIT_VIOLATION", "고정 자세의 J6 각도가 허용 범위를 벗어났습니다.")
            # 고정 관절 자세의 Cartesian 참조값은 작업 좌표 제한을 적용하지 않습니다.
            return [MotionWaypoint(action, pose, j6)]

        guard = self._guard
        target = self._checked_pose(target)
        if action == "reorient":
            if grasp_plan.reorient_mode == "gentle_flip":
                return plan_gentle_flip(
                    target, guard.poses["home_joint_deg"][4], self._j6, guard,
                    self._reorient.get("gentle_tilt_deg", 40.0),
                    self._reorient.get("flip_lift_delta_mm", 90.0),
                )
            if grasp_plan.reorient_mode == "two_stage_flip":
                return plan_two_stage_flip(target, guard)
            # 모의 로봇은 역기구학 대신 접근 시 사용할 J6 각도를 정합니다.
            j6 = self._j6
            if not any(guard.j6_min_deg < j6 + delta < guard.j6_max_deg for delta in (180.0, -180.0)):
                j6 = guard.clamp_j6_angle(-90.0)
            return plan_reorient(target, j6, guard)

        destination = self._checked_pose(destination)
        transit_z = guard.safe_transit_z(max(target[2], destination[2]))
        above_target = self._checked_pose([*target[:2], transit_z, target[3]])
        above_destination = self._checked_pose([*destination[:2], transit_z, destination[3]])
        return [
            MotionWaypoint(stage, pose, self._j6)
            for stage, pose in (
                ("approach", above_target), ("descend", target), ("grasp", target),
                ("lift", above_target), ("transit", above_destination),
                ("place", destination), ("release", destination),
                ("retreat", above_destination),
            )
        ]

    def _interruption(self, goal_handle):
        if self._stop_reason == "emergency" or self._guard.is_emergency_stopped():
            return "CANCELED", True
        if self._stop_reason == "remote" or goal_handle.is_cancel_requested:
            return "CANCELED", False
        return "", False

    def execute_callback(self, goal_handle):
        started = monotonic()
        request = goal_handle.request
        goal = {
            "action_type": request.action_type,
            "product_category": request.product_category,
            "target_pose": list(request.target_pose),
            "dest_pose": list(request.dest_pose),
            "grasp_width_mm": request.grasp_width_mm,
            "grasp_force_n": request.grasp_force_n,
        }
        grasp_plan = None
        with self._state:
            owns_robot = self._active_goal is goal_handle
            try:
                if not owns_robot:
                    error_code, stopped_for_safety = "BUSY", False
                else:
                    error_code, stopped_for_safety = self._interruption(goal_handle)
                action = request.action_type
                if not error_code and action not in TRANSFER_ACTIONS | {"reorient", "scan_pose", "home"}:
                    error_code = "INVALID_ACTION"
                if not error_code:
                    if action in TRANSFER_ACTIONS or action == "reorient":
                        grasp_plan = self._grasp.plan_grasp(
                            request.product_category, request.grasp_width_mm, request.grasp_force_n,
                        )
                    if action == "reorient" and grasp_plan.reorient_mode not in {
                        "gentle_flip", "two_stage_flip", "in_place_yaw_180", "flip_180_in_place",
                    }:
                        error_code = "INVALID_ACTION"
                        waypoints = []
                    else:
                        waypoints = self._waypoints(
                            action, goal["target_pose"], goal["dest_pose"], grasp_plan,
                        )
                    for index, waypoint in enumerate(waypoints, 1):
                        deadline = monotonic() + self._delay
                        while not self._interruption(goal_handle)[0]:
                            remaining = deadline - monotonic()
                            if remaining <= 0:
                                break
                            self._state.wait(timeout=min(remaining, 0.02))
                        error_code, stopped_for_safety = self._interruption(goal_handle)
                        if error_code:
                            break
                        self._pose, self._j6 = waypoint.pose, waypoint.j6_deg
                        goal_handle.publish_feedback(ExecuteRobotTask.Feedback(
                            stage=waypoint.stage, progress=index / len(waypoints),
                        ))
            except ReorientPlanningError as exc:
                error_code, stopped_for_safety = exc.error_code, True
            except ValueError:
                error_code, stopped_for_safety = "INVALID_GRASP", False
            except Exception as exc:
                self.get_logger().error(f"모의 작업 실행 실패: {exc}")
                error_code, stopped_for_safety = "INTERNAL_ERROR", False

            success = not error_code
            result_error_code = "OK" if success else error_code
            result = ExecuteRobotTask.Result(
                success=success, stopped_for_safety=stopped_for_safety,
                error_code=result_error_code, final_pose=list(self._pose),
            )
            try:
                if success:
                    goal_handle.succeed()
                elif not stopped_for_safety and goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                if self._blackbox is not None:
                    try:
                        self._blackbox.log(
                            task_id=bytes(goal_handle.goal_id.uuid).hex(),
                            action_type=request.action_type, product_category=request.product_category,
                            goal=goal, grasp_plan=grasp_plan, success=success,
                            stopped_for_safety=stopped_for_safety, error_code=result_error_code,
                            final_pose=list(self._pose), duration_ms=(monotonic() - started) * 1000,
                        )
                    except OSError as exc:
                        self.get_logger().error(f"블랙박스 기록 실패: {exc}")
            finally:
                if self._active_goal is goal_handle:
                    self._active_goal = None
                    self._busy = False
        return result


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=4)
    node = None
    try:
        node = MockRobot()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
