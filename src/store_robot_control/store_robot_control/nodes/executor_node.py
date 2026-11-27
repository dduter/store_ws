from math import hypot, isfinite
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
    ReorientPlanningError, plan_edge_pivot_flip, plan_gentle_flip, plan_reorient,
    plan_two_stage_flip,
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
        self._pivot_tilt_deg = float(reorient.get("pivot_tilt_deg", 50.0))
        self._drag_forward_mm = float(reorient.get("drag_forward_mm", 55.0))
        self._release_z_offset_mm = float(reorient.get("release_z_offset_mm", 25.0))
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

    def _move_linear(self, pose):
        current_j6, predicted_j6 = self._predict_j6_after_linear(pose[3])
        if not (self._guard.check_j6_limit(current_j6) and self._guard.check_j6_limit(predicted_j6)):
            raise TaskFailure("J6_LIMIT_VIOLATION", True)
        self._robot.move_linear_4d(pose, self._velocity, self._acceleration)

    def _raise_for_approach(self, handle, pose, move_6d=None):
        current = self._robot.get_current_pose_6d()
        if current[2] < pose[2] and current[:2] != pose[:2]:
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
        if effective_tilt < 35.0 or abs(j5) > 125.0:
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

    def _execute_grasp_task(self, handle, action, goal, grasp):
        if action == "reorient" and grasp.reorient_mode not in {
            "edge_pivot_flip", "gentle_flip", "two_stage_flip", "in_place_yaw_180", "flip_180_in_place",
        }:
            raise TaskFailure("INVALID_ACTION", False)
        target = self._checked_pose(goal["target_pose"])
        source_z = target[2]
        if self._apply_depth:
            target[2] = max(self._guard.z_min_mm, source_z - grasp.grasp_depth_offset_mm)
        target = self._checked_pose(target)
        open_args = (grasp.open_width_mm, grasp.grasp_force_n)
        move_args = (grasp.grasp_width_mm, grasp.grasp_force_n)
        if action == "reorient":
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

        destination = self._checked_pose(goal["dest_pose"])
        transit_z = self._guard.safe_transit_z(max(source_z, destination[2]))
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
