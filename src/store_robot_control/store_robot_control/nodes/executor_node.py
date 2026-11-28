from math import cos, hypot, isfinite, radians, sin
from pathlib import Path
from threading import RLock
from time import monotonic, sleep

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool
from store_interfaces.action import ExecuteRobotTask
from store_interfaces.msg import NodeHealth

from ..core.blackbox_log import BlackboxLogger
from ..core.grasp_planner import GraspPlanner
from ..core.reorient_motion import (
    ReorientPlanningError, plan_back_tilt_vertical_flip, plan_edge_pivot_flip,
    plan_gentle_flip, plan_reorient, plan_two_stage_flip,
)
from ..core.safety_guard import SafetyGuard, load_settings
from ..hardware.dsr_interface import DoosanRobotInterface, MotionUnreachableError
from ..hardware.rg2_gripper import RG2Gripper
from .mock_robot import ACTION_NAME, REMOTE_OVERRIDE_QOS, TRANSFER_ACTIONS, find_config_dir


class TaskFailure(Exception):
    def __init__(self, code, stopped_for_safety):
        super().__init__(code)
        self.code = code
        self.stopped_for_safety = stopped_for_safety


class ExecutorNode(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__("executor_node", parameter_overrides=parameter_overrides)
        self._dsr_node = None
        self._gripper = None
        self._action = None
        try:
            self._configure()
        except Exception:
            self.destroy_node()
            raise

    def _configure(self):
        config_dir = Path(self.declare_parameter("config_dir", str(find_config_dir())).value).resolve()
        settings = load_settings(config_dir / "control_params.yaml")
        workspace = load_settings(config_dir / "workspace_zones.yaml")
        motion = settings["motion"]
        self._dry_run = self.declare_parameter("dry_run", motion["dry_run"]).value
        self._step_delay = self.declare_parameter(
            "step_delay_sec", float(settings.get("mock_robot", {}).get("step_delay_sec", 0.15)),
        ).value
        if not isfinite(self._step_delay) or self._step_delay < 0:
            raise ValueError("단계 대기 시간은 유한한 0 이상의 값이어야 합니다.")
        self._velocity = self.declare_parameter("velocity", float(motion["default_velocity"])).value
        self._acceleration = self.declare_parameter(
            "acceleration", float(motion["default_acceleration"]),
        ).value
        self._slow_velocity = float(motion.get("slow_velocity", 30.0))
        self._slow_acceleration = float(motion.get("slow_acceleration", 30.0))
        reorient = settings.get("reorient", {})
        self._gentle_tilt_deg = float(reorient.get("gentle_tilt_deg", 40.0))
        self._flip_lift_delta_mm = float(reorient.get("flip_lift_delta_mm", 90.0))
        self._release_clearance_mm = float(reorient.get("release_clearance_mm", 55.0))
        self._edge_offset_mm = float(reorient.get("edge_offset_mm", 35.0))
        self._pivot_lift_mm = float(reorient.get("pivot_lift_mm", 70.0))
        self._pivot_tilt_deg = float(reorient.get("pivot_tilt_deg", 35.0))
        self._drag_forward_mm = float(reorient.get("drag_forward_mm", 55.0))
        self._release_z_offset_mm = float(reorient.get("release_z_offset_mm", 25.0))
        self._vertical_flip_params = {
            "back_tilt_deg": float(reorient.get("back_tilt_deg", 30.0)),
            "mid_tilt_deg": float(reorient.get("vertical_mid_tilt_deg", 0.0)),
            "fwd_tilt_deg": float(reorient.get("vertical_fwd_tilt_deg", 45.0)),
            "edge_offset_mm": float(reorient.get("vertical_edge_offset_mm", 65.0)),
            "mid_y_offset_mm": float(reorient.get("vertical_mid_y_offset_mm", 0.0)),
            "sweep_y_mm": float(reorient.get("vertical_sweep_y_mm", 145.0)),
            "lift_delta_mm": float(reorient.get("vertical_lift_delta_mm", 165.0)),
            "mid_z_delta_mm": float(reorient.get("vertical_mid_z_delta_mm", 145.0)),
            "release_z_delta_mm": float(reorient.get("vertical_release_z_delta_mm", 115.0)),
        }
        speeds = (self._velocity, self._acceleration, self._slow_velocity, self._slow_acceleration)
        if not all(isfinite(value) and value > 0 for value in speeds):
            raise ValueError("속도와 가속도는 유한한 양수여야 합니다.")
        self._check_grip = self.declare_parameter("check_grip_detection", True).value
        self._apply_depth = self.declare_parameter(
            "apply_grasp_depth_offset", bool(motion.get("apply_grasp_depth_offset", True)),
        ).value
        blackbox = settings.get("blackbox", {})
        log_dir = self.declare_parameter(
            "log_dir", str(config_dir.parent / blackbox.get("log_dir", "data/logs")),
        ).value
        self._blackbox = BlackboxLogger(log_dir) if blackbox.get("enabled", True) else None
        self._guard = SafetyGuard(workspace)
        self._grasp = GraspPlanner(config_dir / "products_catalog.yaml")
        self._state = RLock()
        self._busy = False
        self._remote_override = False
        self._saw_remote_pub = False
        self._active_goal = None
        self._stop_reason = None
        self._stop_fault = False
        self._stop_futures = []
        self._callback_group = ReentrantCallbackGroup()
        self._stop_group = ReentrantCallbackGroup()
        self._stop_client = None
        robot = workspace["robot"]
        if not self._dry_run:
            from dsr_msgs2.srv import MoveStop

            robot_id = robot["robot_id"].strip("/")
            self._stop_request = MoveStop.Request
            self._stop_client = self.create_client(
                MoveStop, f"/{robot_id}/dsr_controller2/motion/move_stop",
                callback_group=self._stop_group,
            )
            # Doosan API가 이 노드를 직접 스핀하므로 메인 실행기에 등록하지 않습니다.
            self._dsr_node = rclpy.create_node(
                "store_dsr_client", namespace=robot_id, use_global_arguments=False,
            )
        self._gripper = RG2Gripper(
            settings["gripper"]["ip"], settings["gripper"]["port"],
            gripper_type=settings["gripper"]["type"],
            wait_after_move_sec=settings["gripper"]["wait_after_move_sec"],
            dry_run=self._dry_run,
        )
        self._gripper.connect()
        self._robot = DoosanRobotInterface(
            **robot, home_cartesian=self._guard.poses["home_cartesian"],
            home_joint_deg=self._guard.poses["home_joint_deg"],
            dry_run=self._dry_run, dsr_node=self._dsr_node,
            arrival_tolerance_mm=motion.get("arrival_tolerance_mm", 15.0),
            arrival_angle_tolerance_deg=motion.get("arrival_angle_tolerance_deg", 10.0),
        )
        x, y, z, _, _, rz = self._guard.poses["home_cartesian"]
        self._final_pose = [x, y, z, rz]
        self._action = ActionServer(
            self, ExecuteRobotTask, ACTION_NAME, execute_callback=self.execute_callback,
            goal_callback=self.goal_callback, handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback, callback_group=self._callback_group,
        )
        self._stop_sub = self.create_subscription(
            Bool, "/store/emergency_stop", self._on_emergency_stop, 10,
            callback_group=self._stop_group,
        )
        self._remote_sub = self.create_subscription(
            Bool, "/store/mode/remote_override", self._on_remote_override, REMOTE_OVERRIDE_QOS,
            callback_group=self._stop_group,
        )
        self._health = self.create_publisher(NodeHealth, "/store/node_health", 10)
        self._timer = self.create_timer(
            settings["health"]["heartbeat_period_sec"], self._publish_health,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            f"executor_node 준비 완료 (dry_run={self._dry_run}, vel={self._velocity}, "
            f"acc={self._acceleration}, check_grip={self._check_grip}, apply_depth={self._apply_depth})"
        )

    def destroy_node(self):
        try:
            if self._action is not None:
                self._action.destroy()
            if self._gripper is not None:
                self._gripper.close()
        finally:
            if self._dsr_node is not None:
                self._dsr_node.destroy_node()
            super().destroy_node()

    def _publish_health(self):
        with self._state:
            pub_count = self.count_publishers("/store/mode/remote_override")
            if pub_count > 0:
                self._saw_remote_pub = True
            elif self._remote_override and self._saw_remote_pub:
                self._remote_override = False
                self._saw_remote_pub = False
                self.get_logger().warn("원격 모드 발행자가 종료되어 자동 실행 잠금을 해제했습니다.")
            stopped = self._guard.is_emergency_stopped()
            fault = self._stop_fault
            stop_pending = self._stop_pending()
            remote = self._remote_override
            busy = self._busy
        message = (
            f"dry_run={str(self._dry_run).lower()}, "
            f"busy={str(busy).lower()}, "
            f"remote_override={str(remote).lower()}"
        )
        if stopped:
            message += ", 비상 정지 활성"
        if fault:
            message += ", 급정지 오류 래치"
        if stop_pending:
            message += ", 급정지 응답 대기"
        if remote:
            message += ", 원격 모드 활성"
        self._health.publish(NodeHealth(
            node_name="executor_node", alive=True,
            state="ERROR" if (stopped or fault) else ("WARN" if (remote or stop_pending) else "OK"),
            message=message, stamp=self.get_clock().now().to_msg(),
        ))

    def quick_stop(self):
        if self._stop_client is None:
            return
        try:
            future = self._stop_client.call_async(self._stop_request(stop_mode=1))
            future.add_done_callback(self._on_quick_stop_done)
            self._stop_futures = [pending for pending in self._stop_futures if not pending.done()]
            self._stop_futures.append(future)
        except Exception as exc:
            self._stop_fault = True
            self.get_logger().error(f"Doosan 급정지 요청 실패 ({exc})")

    def _on_quick_stop_done(self, future):
        try:
            if not future.result().success:
                with self._state:
                    self._stop_fault = True
                self.get_logger().error("Doosan 제어기가 급정지 요청을 거부했습니다.")
        except Exception as exc:
            with self._state:
                self._stop_fault = True
            self.get_logger().error(f"Doosan 급정지 응답 실패 ({exc})")

    def _on_emergency_stop(self, message):
        with self._state:
            self._guard.set_emergency_stop(message.data)
            if message.data:
                if self._busy:
                    self._stop_reason = "emergency"
                self.quick_stop()

    def _stop_pending(self):
        return any(not future.done() for future in self._stop_futures)

    def _on_remote_override(self, message):
        with self._state:
            self._remote_override = message.data
            if self._remote_override and self._busy:
                if self._stop_reason != "emergency":
                    self._stop_reason = "remote"
                self.quick_stop()
        self._publish_health()

    def goal_callback(self, _request):
        with self._state:
            if self._remote_override or self._busy or self._stop_pending() or self._stop_fault:
                return GoalResponse.REJECT
            return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._state:
            if not self._busy and not self._stop_pending() and not self._stop_fault:
                self._busy = True
                self._active_goal = goal_handle
                self._stop_reason = "emergency" if self._guard.is_emergency_stopped() else None
                if self._stop_reason is None and self._remote_override:
                    self._stop_reason = "remote"
        # 동시에 수락된 목표도 실행 콜백에서 결과를 반환합니다.
        try:
            goal_handle.execute()
        except Exception:
            self._release_goal(goal_handle)
            raise

    def _release_goal(self, goal_handle):
        released = False
        with self._state:
            if self._active_goal is goal_handle:
                self._active_goal = None
                self._busy = False
                released = True
        if released:
            self._publish_health()

    def cancel_callback(self, goal_handle):
        with self._state:
            if self._active_goal is goal_handle and goal_handle.is_active:
                if self._stop_reason is None:
                    self._stop_reason = "cancel"
                self.quick_stop()
        return CancelResponse.ACCEPT

    def _check_interruption(self, goal_handle):
        with self._state:
            if self._stop_reason == "emergency" or self._guard.is_emergency_stopped():
                raise TaskFailure("CANCELED", True)
            if (self._stop_reason in {"cancel", "remote"} or self._remote_override
                    or goal_handle.is_cancel_requested):
                raise TaskFailure("CANCELED", False)

    def _step(self, handle, stage, index, total, command, *args):
        with self._state:
            self._check_interruption(handle)
        handle.publish_feedback(ExecuteRobotTask.Feedback(stage=stage, progress=index / total))
        if self._dry_run and self._step_delay:
            sleep(self._step_delay)
        with self._state:
            self._check_interruption(handle)
        command(*args)
        with self._state:
            self._check_interruption(handle)

    def _checked_pose(self, pose):
        check = self._guard.validate_pose(pose)
        if not check.ok:
            raise ReorientPlanningError(check.error_code, check.message)
        return check.clamped_pose

    def _predict_j6_after_linear(self, target_rz):
        current_j6 = self._robot.get_current_joints()[5]
        _, _, _, cur_rx, cur_ry, cur_rz = self._robot.get_current_pose_6d()
        # ZYZ 오일러에서 |ry|≈180°일 때 실제 Z축 회전은 (rz - rx)로 결정됩니다.
        current_eff_rz = (cur_rz - cur_rx) if abs(abs(cur_ry) - 180.0) < 5.0 else cur_rz
        delta_rz = (target_rz - current_eff_rz + 180.0) % 360.0 - 180.0
        return current_j6, current_j6 + delta_rz

    def _move_linear(self, pose, vel=None, acc=None):
        if not self._dry_run:
            current_j6, predicted_j6 = self._robot.predict_linear_j6([*pose[:3], 0.0, 180.0, pose[3]])
        else:
            current_j6, predicted_j6 = self._predict_j6_after_linear(pose[3])
        if not (self._guard.check_j6_limit(current_j6) and self._guard.check_j6_limit(predicted_j6)):
            raise TaskFailure("J6_LIMIT_VIOLATION", True)
        self._robot.move_linear_4d(
            pose, self._velocity if vel is None else vel, self._acceleration if acc is None else acc,
        )

    def _raise_for_approach(self, handle, pose, move_6d=None, *, raise_at_same_xy=False):
        current = self._robot.get_current_pose_6d()
        if current[2] < pose[2] and (raise_at_same_xy or current[:2] != pose[:2]):
            # 현재 XY에서 먼저 상승한 뒤 목표 XY로 이동합니다.
            vertical = self._checked_pose([*current[:2], pose[2], current[5]])
            self._check_interruption(handle)
            if move_6d is None:
                self._robot.move_linear_4d(
                    vertical, self._velocity, self._acceleration, rx=current[3], ry=current[4],
                )
            else:
                move_6d([*vertical[:3], *current[3:]])

    def _approach(self, handle, pose):
        self._raise_for_approach(handle, pose)
        self._check_interruption(handle)
        self._move_linear(pose)

    def _move_joint(self, joints):
        if not self._guard.check_j6_limit(joints[5]):
            raise TaskFailure("J6_LIMIT_VIOLATION", True)
        self._robot.move_joint(joints, self._velocity, self._acceleration)

    def _move_linear_6d(self, handle, pose, vel=None, acc=None):
        current, predicted = self._robot.predict_linear_j6(pose)
        if not all(self._guard.check_j6_limit(j6) for j6 in (current, predicted)):
            raise TaskFailure("J6_LIMIT_VIOLATION", True)
        self._check_interruption(handle)
        self._robot.move_linear_6d(
            pose, self._velocity if vel is None else vel, self._acceleration if acc is None else acc,
        )
        self._check_interruption(handle)
        if not self._guard.check_j6_limit(self._robot.get_current_joints()[5]):
            raise TaskFailure("J6_LIMIT_VIOLATION", True)

    def _verify_grasp(self, handle, grasp, above_target, retreat=None):
        if self._check_grip and not self._gripper.is_grip_detected():
            self._step(handle, "grasp_fail_retreat", 0, 1, retreat or self._move_linear, above_target)
            self._step(handle, "grasp_fail_release", 1, 1, self._gripper.open_mm,
                       grasp.open_width_mm, grasp.grasp_force_n)
            raise TaskFailure("GRASP_FAIL", True)

    def _execute_two_stage_flip(self, handle, waypoints, grasp, open_args, move_args):
        def move(pose):
            self._move_linear_6d(handle, pose)

        def approach(pose):
            self._raise_for_approach(handle, pose, move)
            move(pose)

        total = len(waypoints) + 1
        self._step(handle, "open_gripper", 1, total, self._gripper.open_mm, *open_args)
        retreat_pose = None
        for index, waypoint in enumerate(waypoints, 2):
            if waypoint.action == "grasp":
                self._step(handle, waypoint.stage, index, total, self._gripper.move_mm, *move_args)
                if waypoint.check_grip:
                    self._verify_grasp(handle, grasp, retreat_pose, move)
            elif waypoint.action == "release":
                self._step(handle, waypoint.stage, index, total, self._gripper.open_mm, *open_args)
            else:
                pose = [*waypoint.pose[:3], *waypoint.orientation_zyz]
                command = approach if waypoint.stage == "approach" else move
                self._step(handle, waypoint.stage, index, total, command, pose)
                if waypoint.stage in {"approach", "reapproach_stage2"}:
                    retreat_pose = pose

    def _execute_gentle_flip(self, handle, target, grasp, open_args, move_args):
        current_j6, expected_j6 = self._predict_j6_after_linear(target[3])
        if not self._guard.check_j6_limit(current_j6):
            raise TaskFailure("J6_LIMIT_VIOLATION", True)
        tilt = self._gentle_tilt_deg
        waypoints = plan_gentle_flip(
            target, self._robot.get_current_joints()[4], expected_j6,
            self._guard, tilt, self._flip_lift_delta_mm,
            release_clearance_mm=self._release_clearance_mm,
        )
        (
            approach, descend, grip, lift, tilt_point,
            place_flip, release, lift_after, restore, retreat,
        ) = waypoints
        total = len(waypoints) + 1
        self._step(handle, "open_gripper", 1, total, self._gripper.open_mm, *open_args)
        self._step(handle, approach.stage, 2, total, self._approach, handle, approach.pose)
        self._step(handle, descend.stage, 3, total, self._move_linear, descend.pose)
        self._step(handle, grip.stage, 4, total, self._gripper.move_mm, *move_args)
        self._verify_grasp(handle, grasp, approach.pose)
        self._step(handle, lift.stage, 5, total, self._move_linear, lift.pose)

        # 직선 이동 이후의 실제 관절 자세를 기준으로 J5만 변경하고 정기구학으로 TCP 범위를 검증합니다.
        joints = list(self._robot.get_current_joints())
        j5 = joints[4]
        delta_j5 = -tilt if j5 >= 0.0 else tilt
        tilted_j5 = j5 + delta_j5
        if not all(isfinite(angle) and abs(angle) <= 125.0 for angle in (j5, tilted_j5)):
            raise ReorientPlanningError("OUT_OF_REACH", "J5 안전 각도 범위를 초과합니다.")
        if not self._guard.check_j6_limit(joints[5]):
            raise TaskFailure("J6_LIMIT_VIOLATION", True)
        for fraction in (0.5, 1.0):
            sample = joints.copy()
            sample[4] = j5 + delta_j5 * fraction
            predicted_pose = self._robot.predict_forward_pose(sample)
            if predicted_pose is not None:
                check = self._guard.validate_pose([*predicted_pose[:3], predicted_pose[5]])
                if not check.ok:
                    raise ReorientPlanningError(
                        check.error_code, "J5 회전 궤적이 작업 범위를 벗어납니다.",
                    )
        tilted_joints = joints.copy()
        tilted_joints[4] = tilted_j5
        speeds = (
            min(self._velocity, self._slow_velocity),
            min(self._acceleration, self._slow_acceleration),
        )
        self._step(handle, tilt_point.stage, 6, total, self._robot.move_joint, tilted_joints, *speeds)
        tilted_pose = self._robot.get_current_pose_6d()
        place_z = min(tilted_pose[2], place_flip.pose[2])
        place_6d = [*tilted_pose[:2], place_z, *tilted_pose[3:]]
        self._checked_pose([*place_6d[:3], place_6d[5]])
        self._step(handle, place_flip.stage, 7, total, self._move_linear_6d, handle, place_6d, *speeds)
        self._step(handle, release.stage, 8, total, self._gripper.open_mm, *open_args)
        self._step(handle, lift_after.stage, 9, total, self._move_linear_6d, handle, tilted_pose, *speeds)
        if not self._guard.check_j6_limit(self._robot.get_current_joints()[5]):
            raise TaskFailure("J6_LIMIT_VIOLATION", True)
        self._step(handle, restore.stage, 10, total, self._robot.move_joint, joints, *speeds)
        self._step(handle, retreat.stage, 11, total, self._move_linear, retreat.pose)

    def _check_j5_arc_fkin(self, base_joints, delta_j5):
        j5 = base_joints[4]
        for fraction in (0.5, 1.0):
            sample = base_joints.copy()
            sample[4] = j5 + delta_j5 * fraction
            predicted_pose = self._robot.predict_forward_pose(sample)
            if predicted_pose is not None:
                self._checked_pose([*predicted_pose[:3], predicted_pose[5]])
            elif not self._dry_run:
                raise ReorientPlanningError("OUT_OF_REACH", "J5 회전 궤적의 정기구학을 확인할 수 없습니다.")

    def _execute_edge_pivot_flip(self, handle, target, grasp, open_args, move_args):
        joints = self._robot.get_current_joints()
        tilt = self._pivot_tilt_deg
        try:
            waypoints = plan_edge_pivot_flip(
                target, joints[4], joints[5], self._guard,
                edge_offset_mm=self._edge_offset_mm, pivot_lift_mm=self._pivot_lift_mm,
                pivot_tilt_deg=tilt, drag_forward_mm=self._drag_forward_mm,
                release_z_offset_mm=self._release_z_offset_mm,
            )
        except ReorientPlanningError as exc:
            if exc.error_code != "OUT_OF_REACH":
                raise
            self.get_logger().warn(
                f"가장자리 피벗 공간 부족({exc})으로 소각도 gentle_flip으로 대체합니다."
            )
            self._execute_gentle_flip(handle, target, grasp, open_args, move_args)
            return
        approach, descend, grip, lift, tilt_point, drag, release, lift_after, restore, retreat = waypoints
        total = len(waypoints) + 1
        self._step(handle, "open_gripper", 1, total, self._gripper.open_mm, *open_args)
        self._step(handle, approach.stage, 2, total, self._approach, handle, approach.pose)
        self._step(handle, descend.stage, 3, total, self._move_linear, descend.pose)
        self._step(handle, grip.stage, 4, total, self._gripper.move_mm, *move_args)
        self._verify_grasp(handle, grasp, approach.pose)
        self._step(handle, lift.stage, 5, total, self._move_linear, lift.pose)

        # 직선 상승 후 실제 관절 자세로 J5의 특이점 여유(|J5|>=20°)와 TCP 궤적을 다시 확인합니다.
        joints = list(self._robot.get_current_joints())
        j5 = joints[4]
        effective_tilt = min(tilt, abs(j5) - 20.0)
        if effective_tilt < 20.0 or abs(j5) > 125.0:
            raise ReorientPlanningError("OUT_OF_REACH", "J5 특이점 또는 안전 각도 범위를 벗어납니다.")
        delta_j5 = -effective_tilt if j5 >= 0.0 else effective_tilt
        tilted_j5 = j5 + delta_j5
        if not (20.0 <= abs(tilted_j5) <= 125.0 and j5 * tilted_j5 > 0):
            raise ReorientPlanningError("OUT_OF_REACH", "J5 특이점 또는 안전 각도 범위를 벗어납니다.")
        if not self._guard.check_j6_limit(joints[5]):
            raise TaskFailure("J6_LIMIT_VIOLATION", True)
        self._check_j5_arc_fkin(joints, delta_j5)

        speeds = (
            min(self._velocity, self._slow_velocity),
            min(self._acceleration, self._slow_acceleration),
        )
        tilted_joints = joints.copy()
        tilted_joints[4] = tilted_j5
        self._step(handle, tilt_point.stage, 6, total, self._robot.move_joint, tilted_joints, *speeds)

        tilted_pose = self._robot.get_current_pose_6d()
        x, y = target[:2]
        radial = hypot(x, y)
        ux, uy = (x / radial, y / radial) if radial > 1e-6 else (1.0, 0.0)
        drag_6d = [
            tilted_pose[0] + ux * self._drag_forward_mm,
            tilted_pose[1] + uy * self._drag_forward_mm,
            min(tilted_pose[2], drag.pose[2]),
            *tilted_pose[3:],
        ]
        lift_6d = [*drag_6d[:2], tilted_pose[2], *tilted_pose[3:]]
        self._checked_pose([*drag_6d[:3], drag_6d[5]])
        self._checked_pose([*lift_6d[:3], lift_6d[5]])
        self._step(handle, drag.stage, 7, total, self._move_linear_6d, handle, drag_6d, *speeds)
        self._step(handle, release.stage, 8, total, self._gripper.open_mm, *open_args)
        self._step(handle, lift_after.stage, 9, total, self._move_linear_6d, handle, lift_6d, *speeds)

        # 드래그 후 관절에서 J5만 복원하며, 복원 궤적도 정기구학으로 검증합니다.
        post_joints = list(self._robot.get_current_joints())
        post_j5 = post_joints[4]
        restored_joints = post_joints.copy()
        restored_joints[4] -= delta_j5
        if not (20.0 <= abs(post_j5) <= 125.0 and 20.0 <= abs(restored_joints[4]) <= 125.0
                and post_j5 * restored_joints[4] > 0):
            raise ReorientPlanningError("OUT_OF_REACH", "J5 복원 각도가 안전 범위를 벗어납니다.")
        if not self._guard.check_j6_limit(restored_joints[5]):
            raise TaskFailure("J6_LIMIT_VIOLATION", True)
        self._check_j5_arc_fkin(post_joints, -delta_j5)
        self._step(handle, restore.stage, 10, total, self._robot.move_joint, restored_joints, *speeds)
        self._step(handle, retreat.stage, 11, total, self._move_linear, retreat.pose)

    def _check_joint_path_fkin(self, start_joints, end_joints, fractions=(0.25, 0.5, 0.75, 1.0)):
        for fraction in fractions:
            joints = [
                start + fraction * (end - start) for start, end in zip(start_joints, end_joints)
            ]
            predicted = self._robot.predict_forward_pose(joints)
            if predicted is not None:
                self._checked_pose([*predicted[:3], predicted[5]])
            elif not self._dry_run:
                raise ReorientPlanningError("OUT_OF_REACH", "관절 이동 경로의 정기구학을 확인할 수 없습니다.")

    def _execute_back_tilt_vertical_flip(self, handle, target, grasp, open_args, move_args):
        robot = self._robot
        cur_joints = list(robot.get_current_joints())
        params = dict(self._vertical_flip_params)
        if grasp.vertical_edge_offset_mm is not None:
            params["edge_offset_mm"] = grasp.vertical_edge_offset_mm
        if grasp.vertical_back_tilt_deg is not None:
            params["back_tilt_deg"] = grasp.vertical_back_tilt_deg
        if grasp.vertical_fwd_tilt_deg is not None:
            params["fwd_tilt_deg"] = grasp.vertical_fwd_tilt_deg
        if grasp.vertical_sweep_mm is not None:
            params["sweep_y_mm"] = grasp.vertical_sweep_mm
        if grasp.vertical_lift_delta_mm is not None:
            params["lift_delta_mm"] = grasp.vertical_lift_delta_mm
        if grasp.vertical_mid_z_delta_mm is not None:
            params["mid_z_delta_mm"] = grasp.vertical_mid_z_delta_mm
        if grasp.vertical_release_z_delta_mm is not None:
            params["release_z_delta_mm"] = grasp.vertical_release_z_delta_mm
        params["corner_x_offset_mm"] = grasp.vertical_corner_x_offset_mm
        params["entry_axis"] = grasp.vertical_entry_axis
        approach, descend, grip, lift, mid, flip, release, reverse, retreat = (
            plan_back_tilt_vertical_flip(target, cur_joints[5], self._guard, **params)
        )

        j6_limits = (self._guard.j6_min_deg, self._guard.j6_max_deg)

        def solve(pose, reference, *, max_joint_deltas=None):
            kwargs = {"j6_limits": j6_limits}
            if max_joint_deltas is not None:
                kwargs["max_joint_deltas"] = max_joint_deltas
            joints = robot.solve_continuous_ik(pose, reference, **kwargs)
            if joints is None:
                raise ReorientPlanningError("OUT_OF_REACH", "길이 방향 반전에 필요한 연속 관절 해가 없습니다.")
            return joints

        rz = approach.pose[3]
        entry_axis = grasp.vertical_entry_axis
        sign = grasp.vertical_tilt_sign
        fwd_sign = sign if grasp.vertical_fwd_tilt_sign is None else grasp.vertical_fwd_tilt_sign
        j6_off = grasp.vertical_j6_offset_deg
        # ZYZ [90 - rz, sign * (180 - tilt), 90 + j6_off]은 tilt->0에서 [0, 180, rz + j6_off]와 연속입니다.
        tilt_rx = (90.0 - rz + 180.0) % 360.0 - 180.0
        tilt_gamma = (90.0 + j6_off + 180.0) % 360.0 - 180.0
        mid_rz = (rz + j6_off + 180.0) % 360.0 - 180.0
        if entry_axis == "x":
            x_rx = (grasp.vertical_entry_yaw_deg - rz + 180.0) % 360.0 - 180.0
            if abs(params["back_tilt_deg"]) < 1e-6:
                # 수직(0°) 인형뽑기 진입: J6만 entry_yaw + j6_off 회전시켜 두 손가락이 꼭짓점 양옆 빈 공간으로 하강합니다.
                back_rz = (rz + grasp.vertical_entry_yaw_deg + j6_off + 180.0) % 360.0 - 180.0
                back_orientation = [0.0, 180.0, back_rz]
            else:
                # 45° 대각선 두 손가락 수평 유지(toolX Z=0) + 팔 각도만 back_tilt_deg(예: 30°) 눕혀 진입합니다.
                back_gamma = (-90.0 + j6_off + 180.0) % 360.0 - 180.0
                back_orientation = [x_rx, sign * (180.0 - params["back_tilt_deg"]), back_gamma]
            fwd_orientation = [tilt_rx, fwd_sign * (180.0 - params["fwd_tilt_deg"]), tilt_gamma]
        else:
            back_orientation = [tilt_rx, sign * (180.0 - params["back_tilt_deg"]), tilt_gamma]
            fwd_orientation = [tilt_rx, fwd_sign * (180.0 - params["fwd_tilt_deg"]), tilt_gamma]
        mid_tilt = params["mid_tilt_deg"]
        mid_orientation = (
            [0.0, 180.0, mid_rz] if abs(mid_tilt) < 1e-6 else [tilt_rx, sign * (180.0 - mid_tilt), tilt_gamma]
        )
        app_6d = [*approach.pose[:3], *back_orientation]
        grp_6d = [*descend.pose[:3], *back_orientation]
        mid_6d = [*mid.pose[:3], *mid_orientation]
        end_6d = [*flip.pose[:3], *fwd_orientation]
        retreat_6d = [*retreat.pose[:3], 0.0, 180.0, rz]

        init_deltas = (180.0, 120.0, 120.0, 85.0, 120.0, 180.0)
        roll_deltas = (95.0, 90.0, 95.0, 85.0, 65.0, 110.0)
        slide_in = grasp.vertical_slide_in_y_mm
        if slide_in > 0.0:
            # 옆에서 봤을 때 ㄷ자 진입: entry_axis='x'면 진입 각도(x_rx) 반대 바깥에서 대각선/정면 슬라이드 진입
            rz_rad = radians(rz)
            if entry_axis == "x":
                slide_rad = radians(x_rx)
                out_x = descend.pose[0] + slide_in * cos(slide_rad)
                out_y = descend.pose[1] + slide_in * sin(slide_rad)
            else:
                out_x = descend.pose[0] - slide_in * sin(rz_rad)
                out_y = descend.pose[1] - slide_in * cos(rz_rad)
            out_app_pose = self._checked_pose([out_x, out_y, approach.pose[2], rz])
            out_pre_pose = self._checked_pose([out_x, out_y, descend.pose[2], rz])
            out_app_6d = [*out_app_pose[:3], *back_orientation]
            out_pre_6d = [*out_pre_pose[:3], *back_orientation]
            j_out_app = solve(out_app_6d, cur_joints, max_joint_deltas=init_deltas)
            j_out_pre = solve(out_pre_6d, j_out_app)
            j_grp = solve(grp_6d, j_out_pre)
            j_lift = solve(app_6d, j_grp)
            j_mid = solve(mid_6d, j_lift, max_joint_deltas=roll_deltas)
            j_end = solve(end_6d, j_mid, max_joint_deltas=roll_deltas)
            j_retreat = solve(retreat_6d, j_mid, max_joint_deltas=roll_deltas)
            joint_set = (j_out_app, j_out_pre, j_grp, j_lift, j_mid, j_end, j_retreat)
        else:
            j_app = solve(app_6d, cur_joints, max_joint_deltas=init_deltas)
            j_grp = solve(grp_6d, j_app)
            j_mid = solve(mid_6d, j_app, max_joint_deltas=roll_deltas)
            j_end = solve(end_6d, j_mid, max_joint_deltas=roll_deltas)
            j_retreat = solve(retreat_6d, j_mid, max_joint_deltas=roll_deltas)
            joint_set = (j_app, j_grp, j_mid, j_end, j_retreat)

        if not all(self._guard.check_j6_limit(joints[5]) for joints in joint_set):
            raise TaskFailure("J6_LIMIT_VIOLATION", True)

        if slide_in > 0.0:
            self._check_joint_path_fkin(j_out_app, j_out_pre)
            self._check_joint_path_fkin(j_out_pre, j_grp)
            self._check_joint_path_fkin(j_grp, j_lift)
            self._check_joint_path_fkin(j_lift, j_mid)
        else:
            self._check_joint_path_fkin(j_app, j_grp)
            self._check_joint_path_fkin(j_app, j_mid)
        self._check_joint_path_fkin(j_mid, j_end, fractions=(0.2, 0.4, 0.6, 0.8, 1.0))
        self._check_joint_path_fkin(j_mid, j_retreat)

        speeds = (
            min(self._velocity, self._slow_velocity),
            min(self._acceleration, self._slow_acceleration),
        )

        def move_joint(joints, sim_pose=None):
            robot.move_joint(joints, *speeds)
            if self._dry_run and sim_pose is not None:
                robot.set_simulated_reference_pose(sim_pose)

        if slide_in > 0.0:
            def approach_outside():
                self._raise_for_approach(
                    handle, out_app_pose, lambda pose: self._move_linear_6d(handle, pose, *speeds),
                    raise_at_same_xy=True,
                )
                self._check_interruption(handle)
                self._check_joint_path_fkin(list(robot.get_current_joints()), j_out_app)
                self._check_interruption(handle)
                move_joint(j_out_app, out_app_6d)

            total = 11
            self._step(handle, "open_gripper", 1, total, self._gripper.open_mm, *open_args)
            self._step(handle, approach.stage, 2, total, approach_outside)
            self._step(handle, "descend_outside", 3, total, move_joint, j_out_pre, out_pre_6d)
            self._step(handle, "slide_in_grasp", 4, total, move_joint, j_grp, grp_6d)
            self._step(handle, grip.stage, 5, total, self._gripper.move_mm, *move_args)
            self._verify_grasp(handle, grasp, j_lift, retreat=lambda joints: move_joint(joints, app_6d))
            self._step(handle, lift.stage, 6, total, move_joint, j_lift, app_6d)
            self._step(handle, mid.stage, 7, total, move_joint, j_mid, mid_6d)
            self._step(handle, flip.stage, 8, total, move_joint, j_end, end_6d)
            self._step(handle, release.stage, 9, total, self._gripper.open_mm, *open_args)
            self._step(handle, reverse.stage, 10, total, move_joint, j_mid, mid_6d)
            self._step(handle, retreat.stage, 11, total, move_joint, j_retreat, retreat_6d)
            return

        def approach_back():
            self._raise_for_approach(
                handle, approach.pose, lambda pose: self._move_linear_6d(handle, pose, *speeds),
                raise_at_same_xy=True,
            )
            self._check_interruption(handle)
            self._check_joint_path_fkin(list(robot.get_current_joints()), j_app)
            self._check_interruption(handle)
            move_joint(j_app, app_6d)

        total = 10
        self._step(handle, "open_gripper", 1, total, self._gripper.open_mm, *open_args)
        self._step(handle, approach.stage, 2, total, approach_back)
        self._step(handle, descend.stage, 3, total, move_joint, j_grp, grp_6d)
        self._step(handle, grip.stage, 4, total, self._gripper.move_mm, *move_args)
        self._verify_grasp(handle, grasp, j_app, retreat=lambda joints: move_joint(joints, app_6d))
        self._step(handle, lift.stage, 5, total, move_joint, j_app, app_6d)
        self._step(handle, mid.stage, 6, total, move_joint, j_mid, mid_6d)
        self._step(handle, flip.stage, 7, total, move_joint, j_end, end_6d)
        self._step(handle, release.stage, 8, total, self._gripper.open_mm, *open_args)
        self._step(handle, reverse.stage, 9, total, move_joint, j_mid, mid_6d)
        self._step(handle, retreat.stage, 10, total, move_joint, j_retreat, retreat_6d)

    def _execute_grasp_task(self, handle, action, goal, grasp):
        if action == "reorient" and grasp.reorient_mode not in {
            "edge_pivot_flip", "gentle_flip", "two_stage_flip", "in_place_yaw_180", "flip_180_in_place",
            "back_tilt_vertical_flip",
        }:
            raise TaskFailure("INVALID_ACTION", False)
        target = self._checked_pose(goal["target_pose"])
        source_z = target[2]
        if self._apply_depth:
            target[2] = max(
                self._guard.z_min_at(target[0], target[1]), source_z - grasp.grasp_depth_offset_mm,
            )
        target = self._checked_pose(target)
        open_args = (grasp.open_width_mm, grasp.grasp_force_n)
        move_args = (grasp.grasp_width_mm, grasp.grasp_force_n)
        if action == "reorient":
            if grasp.reorient_mode == "back_tilt_vertical_flip":
                self._execute_back_tilt_vertical_flip(handle, target, grasp, open_args, move_args)
                return
            if grasp.reorient_mode == "edge_pivot_flip":
                self._execute_edge_pivot_flip(handle, target, grasp, open_args, move_args)
                return
            if grasp.reorient_mode == "gentle_flip":
                self._execute_gentle_flip(handle, target, grasp, open_args, move_args)
                return
            if grasp.reorient_mode == "two_stage_flip":
                waypoints = plan_two_stage_flip(target, self._guard)
                self._execute_two_stage_flip(handle, waypoints, grasp, open_args, move_args)
                return
            _, expected_j6 = self._predict_j6_after_linear(target[3])
            waypoints = plan_reorient(target, expected_j6, self._guard)
            above_target = waypoints[0].pose
            self._step(handle, "open_gripper", 0, len(waypoints), self._gripper.open_mm, *open_args)
            total = len(waypoints)
            self._step(handle, "approach", 1, total, self._approach, handle, above_target)
            self._step(handle, "descend", 2, total, self._move_linear, waypoints[1].pose)
            self._step(handle, "grasp", 3, total, self._gripper.move_mm, *move_args)
            self._verify_grasp(handle, grasp, above_target)
            self._step(handle, "lift", 4, total, self._move_linear, waypoints[3].pose)
            # 직선 접근 중 J6가 변할 수 있어 회전 직전에 180도 경로를 다시 확인합니다.
            joints = self._robot.get_current_joints()
            if joints[5] != expected_j6:
                waypoints = plan_reorient(target, joints[5], self._guard)
            joints[5] = waypoints[4].j6_deg
            self._step(handle, "flip_180", 5, total, self._move_joint, joints)
            self._step(handle, "place", 6, total, self._move_linear, waypoints[5].pose)
            self._step(handle, "release", 7, total, self._gripper.open_mm, *open_args)
            self._step(handle, "retreat", 8, total, self._move_linear, waypoints[7].pose)
            return

        def apply_pick_offset(pose, *, apply_depth=False):
            ox, oy, yaw_off = grasp.pick_offset_x_mm, grasp.pick_offset_y_mm, grasp.pick_yaw_offset_deg
            if ox == 0.0 and oy == 0.0 and yaw_off == 0.0:
                px, py, prz = pose[0], pose[1], pose[3]
            else:
                rz_rad = radians(pose[3])
                px = pose[0] + ox * cos(rz_rad) + oy * sin(rz_rad)
                py = pose[1] - ox * sin(rz_rad) + oy * cos(rz_rad)
                prz = (pose[3] + yaw_off + 180.0) % 360.0 - 180.0
            pz = pose[2]
            if apply_depth and self._apply_depth:
                pz = max(self._guard.z_min_at(px, py), source_z - grasp.grasp_depth_offset_mm)
            return self._checked_pose([px, py, pz, prz])

        target = apply_pick_offset(target, apply_depth=True)
        destination = apply_pick_offset(self._checked_pose(goal["dest_pose"]))
        dest_base_z = destination[2]
        if grasp.place_z_offset_mm > 0.0:
            place_z = dest_base_z + grasp.place_z_offset_mm
            if place_z > self._guard.bounds["z_max"]:
                raise ReorientPlanningError("OUT_OF_REACH", "투하 높이가 작업 범위를 벗어났습니다.")
            destination = self._checked_pose([
                destination[0], destination[1], place_z, destination[3],
            ])
        transit_z = self._guard.safe_transit_z(max(source_z, dest_base_z))
        if grasp.pick_transit_z_mm is not None:
            transit_z = min(self._guard.bounds["z_max"], max(transit_z, grasp.pick_transit_z_mm))
        if destination[2] > transit_z:
            if grasp.pick_transit_z_mm is not None:
                raise ReorientPlanningError("OUT_OF_REACH", "투하 높이가 상품 이동 높이를 초과합니다.")
            transit_z = self._guard.safe_transit_z(max(source_z, destination[2]))

        def predict_transfer_j6(t_pose, d_pose):
            if not self._dry_run:
                abv_t = [*t_pose[:2], transit_z, t_pose[3]]
                abv_d = [*d_pose[:2], transit_z, d_pose[3]]
                cur_j6, abv_t_j6 = self._robot.predict_linear_j6([*abv_t[:3], 0.0, 180.0, abv_t[3]])
                _, t_j6 = self._robot.predict_linear_j6([*t_pose[:3], 0.0, 180.0, t_pose[3]])
                _, abv_d_j6 = self._robot.predict_linear_j6([*abv_d[:3], 0.0, 180.0, abv_d[3]])
                _, d_j6 = self._robot.predict_linear_j6([*d_pose[:3], 0.0, 180.0, d_pose[3]])
                return cur_j6, abv_t_j6, t_j6, abv_d_j6, d_j6
            cur_j6, t_j6 = self._predict_j6_after_linear(t_pose[3])
            d_j6 = t_j6 + ((d_pose[3] - t_pose[3] + 180.0) % 360.0 - 180.0)
            return cur_j6, t_j6, d_j6

        def wrap_rz(angle):
            return (angle + 180.0) % 360.0 - 180.0

        j6_values = predict_transfer_j6(target, destination)
        if not all(self._guard.check_j6_limit(j6) for j6 in j6_values):
            # 대칭 2핑거 그리퍼가 180° 반대 꼭짓점(|rz|>90°)으로 잡을 때 J6 한계 초과 시 ±180° 등가 각도를 시도합니다.
            if grasp.pick_yaw_offset_deg != 0.0 and abs(target[3]) > 90.0:
                for shift in (-180.0, 180.0):
                    alt_t = [*target[:3], wrap_rz(target[3] + shift)]
                    alt_d = [*destination[:3], wrap_rz(destination[3] + shift)]
                    alt_j6 = predict_transfer_j6(alt_t, alt_d)
                    if all(self._guard.check_j6_limit(j6) for j6 in alt_j6):
                        target, destination, j6_values = alt_t, alt_d, alt_j6
                        break
        if not all(self._guard.check_j6_limit(j6) for j6 in j6_values):
            raise TaskFailure("J6_LIMIT_VIOLATION", True)

        transfer_force = (
            grasp.pick_grasp_force_n if grasp.pick_grasp_force_n is not None else grasp.grasp_force_n
        )
        open_args = (grasp.open_width_mm, transfer_force)
        move_args = (grasp.grasp_width_mm, transfer_force)
        above_target = self._checked_pose([*target[:2], transit_z, target[3]])
        above_destination = self._checked_pose([*destination[:2], transit_z, destination[3]])
        steps = [
            ("open_gripper", self._gripper.open_mm, open_args),
            ("approach", self._approach, (handle, above_target)),
            ("descend", self._move_linear, (target,)),
            ("grasp", self._gripper.move_mm, move_args),
            ("lift", self._move_linear, (above_target,)),
            ("transit", self._move_linear, (above_destination,)),
            ("place", self._move_linear, (destination,)),
            ("release", self._gripper.open_mm, open_args),
            ("retreat", self._move_linear, (above_destination,)),
        ]
        for index, (stage, command, args) in enumerate(steps, 1):
            self._step(handle, stage, index, len(steps), command, *args)
            if stage == "grasp":
                self._verify_grasp(handle, grasp, above_target)

    def _finish_goal(self, handle, code, stopped):
        deadline = monotonic() + 0.5
        while True:
            with self._state:
                if self._active_goal is handle:
                    try:
                        self._check_interruption(handle)
                    except TaskFailure as exc:
                        code, stopped = exc.code, exc.stopped_for_safety
                # rclpy는 취소 콜백이 ACCEPT를 반환한 뒤 취소 상태를 설정합니다.
                if (self._active_goal is handle and self._stop_reason == "cancel"
                        and not handle.is_cancel_requested and monotonic() < deadline):
                    pass
                else:
                    if handle.is_cancel_requested:
                        handle.canceled()
                    elif code == "OK":
                        handle.succeed()
                    else:
                        handle.abort()
                    return code, stopped
            sleep(0.001)

    def execute_callback(self, goal_handle):
        try:
            return self._execute_task(goal_handle)
        finally:
            self._release_goal(goal_handle)

    def _execute_task(self, handle):
        started = monotonic()
        request = handle.request
        action = request.action_type
        goal = {
            "action_type": action, "product_category": request.product_category,
            "target_pose": list(request.target_pose), "dest_pose": list(request.dest_pose),
            "grasp_width_mm": request.grasp_width_mm, "grasp_force_n": request.grasp_force_n,
        }
        self.get_logger().info(
            f"작업 시작: action={action}, category={request.product_category}, "
            f"target={goal['target_pose']}, dest={goal['dest_pose']}"
        )
        grasp = None
        code, stopped = "OK", False
        with self._state:
            owns_robot = self._active_goal is handle
        try:
            if not owns_robot:
                raise TaskFailure("BUSY", False)
            self._check_interruption(handle)
            if action in {"home", "scan_pose"}:
                joints = self._guard.poses[f"{action}_joint_deg"]
                self._step(handle, action, 1, 1, self._move_joint, joints)
                if self._dry_run:
                    self._robot.set_simulated_reference_pose(self._guard.poses[f"{action}_cartesian"])
            elif action in TRANSFER_ACTIONS or action == "reorient":
                try:
                    grasp = self._grasp.plan_grasp(
                        request.product_category, request.grasp_width_mm, request.grasp_force_n,
                    )
                except ValueError as exc:
                    raise TaskFailure("INVALID_GRASP", False) from exc
                self._execute_grasp_task(handle, action, goal, grasp)
            else:
                raise TaskFailure("INVALID_ACTION", False)
        except TaskFailure as exc:
            code, stopped = exc.code, exc.stopped_for_safety
        except ReorientPlanningError as exc:
            code, stopped = exc.error_code, True
        except Exception as exc:
            try:
                self._check_interruption(handle)
            except TaskFailure as interruption:
                code, stopped = interruption.code, interruption.stopped_for_safety
            else:
                self.get_logger().error(f"로봇 작업 실행 실패 ({exc})")
                if isinstance(exc, MotionUnreachableError):
                    code, stopped = "OUT_OF_REACH", True
                else:
                    code, stopped = "INTERNAL_ERROR", False

        if owns_robot:
            try:
                self._final_pose = self._robot.get_current_pose_4d()
            except Exception as exc:
                self.get_logger().error(f"최종 자세 읽기 실패 ({exc})")
                if code == "OK":
                    code = "INTERNAL_ERROR"
        final_pose = list(self._final_pose)
        code, stopped = self._finish_goal(handle, code, stopped)
        result = ExecuteRobotTask.Result(
            success=code == "OK", stopped_for_safety=stopped, error_code=code, final_pose=final_pose,
        )
        self.get_logger().info(
            f"작업 종료: action={action}, success={result.success}, error_code={code}, final_pose={final_pose}"
        )
        if self._blackbox is not None:
            try:
                self._blackbox.log(
                    task_id=bytes(handle.goal_id.uuid).hex(), action_type=action,
                    product_category=request.product_category, goal=goal, grasp_plan=grasp,
                    success=result.success, stopped_for_safety=stopped, error_code=code,
                    final_pose=final_pose, duration_ms=(monotonic() - started) * 1000,
                )
            except OSError as exc:
                self.get_logger().error(f"블랙박스 기록 실패 ({exc})")
        return result


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=4)
    node = None
    try:
        node = ExecutorNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()
        try:
            executor.shutdown()
        finally:
            if node is not None:
                node.destroy_node()
