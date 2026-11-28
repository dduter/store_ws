import json
import os
import secrets
from math import hypot, isfinite
from pathlib import Path
from threading import Event, RLock
from time import monotonic

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from store_interfaces.msg import NodeHealth
from store_interfaces.srv import VerifyRemoteKey

from ..core.safety_guard import SafetyGuard, load_settings
from ..core.teleop_math import GripperGestureDebouncer, TeleopPipeline
from ..hardware.dsr_interface import (
    DoosanRobotInterface, MotionUnreachableError, top_down_effective_rz,
)
from ..hardware.rg2_gripper import RG2Gripper
from .mock_robot import REMOTE_OVERRIDE_QOS, find_config_dir


class RemoteNode(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__("remote_node", parameter_overrides=parameter_overrides)
        self._dsr_node = None
        self._gripper = None
        self._remote_pub = None
        self._remote_active = False
        try:
            self._configure()
        except Exception:
            self.destroy_node()
            raise

    def _configure(self):
        config_dir = Path(self.declare_parameter("config_dir", str(find_config_dir())).value)
        settings = load_settings(config_dir / "control_params.yaml")
        workspace = load_settings(config_dir / "workspace_zones.yaml")
        motion, teleop = settings["motion"], settings["teleop"]
        self._dry_run = self.declare_parameter("dry_run", True).value
        self._velocity = self.declare_parameter("velocity", float(motion["slow_velocity"])).value
        self._acceleration = self.declare_parameter(
            "acceleration", float(motion["slow_acceleration"]),
        ).value
        self._scale = teleop["px_to_mm_scale"]
        self._max_step = teleop["max_step_mm"]
        self._min_move = teleop["min_move_mm"]
        rate = teleop["control_rate_hz"]
        if not all(isfinite(value) and value > 0 for value in (
            self._velocity, self._acceleration, self._scale, self._max_step, self._min_move, rate,
        )):
            raise ValueError("원격 제어의 속도, 가속도, 이동 설정 및 주기는 유한한 양수여야 합니다.")
        self._open_width = teleop["gripper_open_width_mm"]
        self._close_width = teleop["gripper_close_width_mm"]
        self._force = teleop["gripper_force_n"]
        self._guard = SafetyGuard(workspace)
        self._teleop_settings = teleop
        self._state = RLock()
        self._remote_active = False
        self._stop_fault = False
        self._stop_future = None
        self._moving = False
        self._motion_idle = Event()
        self._motion_idle.set()
        self._stop_idle = Event()
        self._stop_idle.set()
        self._executor_seen = False
        self._executor_busy = False
        self._executor_remote_ack = False
        self._callback_group = ReentrantCallbackGroup()
        self._stop_group = MutuallyExclusiveCallbackGroup()
        self._motion_group = MutuallyExclusiveCallbackGroup()
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
            # Doosan API가 전용 노드를 직접 스핀합니다.
            self._dsr_node = rclpy.create_node(
                "store_remote_dsr_client", namespace=robot_id, use_global_arguments=False,
            )
        gripper = settings["gripper"]
        self._gripper = RG2Gripper(
            gripper["ip"], gripper["port"], gripper_type=gripper["type"],
            wait_after_move_sec=gripper["wait_after_move_sec"], dry_run=self._dry_run,
        )
        self._gripper.connect()
        self._commanded_closed = self._gripper.get_width_mm() < (self._open_width + self._close_width) / 2
        self._robot = DoosanRobotInterface(
            **robot, home_cartesian=self._guard.poses["home_cartesian"],
            home_joint_deg=self._guard.poses["home_joint_deg"],
            dry_run=self._dry_run, dsr_node=self._dsr_node,
            arrival_tolerance_mm=motion.get("arrival_tolerance_mm", 15.0),
            arrival_angle_tolerance_deg=motion.get("arrival_angle_tolerance_deg", 10.0),
        )
        self._reset_pipeline()
        self._remote_pub = self.create_publisher(Bool, "/store/mode/remote_override", REMOTE_OVERRIDE_QOS)
        self._remote_pub.publish(Bool(data=False))
        self._health = self.create_publisher(NodeHealth, "/store/node_health", 10)
        self._remote_service = self.create_service(
            VerifyRemoteKey, "/store/verify_remote_key", self._verify_remote_key,
            callback_group=self._callback_group,
        )
        self._stop_sub = self.create_subscription(
            Bool, "/store/emergency_stop", self._on_emergency_stop, 10,
            callback_group=self._stop_group,
        )
        self._health_sub = self.create_subscription(
            NodeHealth, "/store/node_health", self._on_node_health, 10,
            callback_group=self._stop_group,
        )
        self._hand_sub = self.create_subscription(
            String, "/store/remote/hand_cmd", self._on_hand_cmd, 1,
            callback_group=self._callback_group,
        )
        self._timer = self.create_timer(1.0 / rate, self._control_tick, callback_group=self._motion_group)
        # 이동 콜백과 독립적으로 워치독 만료 및 정지 후속 처리를 감시합니다.
        self._watchdog_timer = self.create_timer(0.1, self._watchdog_tick, callback_group=self._stop_group)
        self._heartbeat = self.create_timer(
            settings["health"]["heartbeat_period_sec"], self._publish_health,
            callback_group=self._callback_group,
        )
        self.get_logger().info(f"remote_node 준비 완료 (dry_run={self._dry_run}, {rate:g} Hz)")

    def destroy_node(self):
        try:
            if self._remote_pub is not None and self._remote_active:
                try:
                    self._remote_pub.publish(Bool(data=False))
                except Exception:
                    pass
            if self._gripper is not None:
                self._gripper.close()
        finally:
            if self._dsr_node is not None:
                self._dsr_node.destroy_node()
            super().destroy_node()

    def _reset_pipeline(self):
        self._pipeline = TeleopPipeline(self._teleop_settings)
        self._pipeline.gripper.stable_closed = self._commanded_closed
        self._deltas = (0.0, 0.0, 0.0)

    def _verify_remote_key(self, request, response):
        key = os.environ.get("REMOTE_ADMIN_KEY", "")
        if not key or not secrets.compare_digest(key.encode(), request.admin_key.encode()):
            response.reason = "원격 모드 인증에 실패했습니다."
            return response
        wait_for_stop = False
        with self._state:
            if request.enable_remote and self._guard.is_emergency_stopped():
                response.reason = "비상 정지 중에는 원격 모드를 활성화할 수 없습니다."
                return response
            if not request.enable_remote:
                if self._remote_active and self._moving:
                    self.quick_stop()
                self._remote_active = False
                self._reset_pipeline()
                wait_for_stop = True
            else:
                self._remote_active = True
                self._reset_pipeline()
                self._remote_pub.publish(Bool(data=True))
                response.granted = True
                response.reason = "원격 모드 활성"
                return response
        if wait_for_stop:
            # 원격 이동과 급정지가 완전히 끝난 뒤 자동 실행기에 제어권을 넘깁니다.
            self._motion_idle.wait(timeout=2.0)
            self._stop_idle.wait(timeout=2.0)
            with self._state:
                if not self._remote_active:
                    self._remote_pub.publish(Bool(data=False))
        response.granted = True
        response.reason = "원격 모드 해제"
        return response

    def _on_node_health(self, message):
        if message.node_name != "executor_node":
            return
        with self._state:
            self._executor_seen = True
            self._executor_busy = "busy=true" in message.message
            self._executor_remote_ack = "remote_override=true" in message.message

    def _on_emergency_stop(self, message):
        with self._state:
            self._guard.set_emergency_stop(message.data)
            if message.data:
                if not self._dry_run:
                    self.quick_stop()
                if self._remote_active:
                    self._remote_active = False
                    self._reset_pipeline()
                    self._remote_pub.publish(Bool(data=False))

    def quick_stop(self):
        if self._stop_client is None:
            return
        try:
            if not self._stop_client.service_is_ready():
                raise RuntimeError("Doosan 급정지 서비스를 사용할 수 없습니다.")
            self._stop_idle.clear()
            self._stop_future = self._stop_client.call_async(self._stop_request(stop_mode=1))
            self._stop_future.add_done_callback(self._on_quick_stop_done)
        except Exception as exc:
            with self._state:
                self._stop_fault = True
                self._stop_idle.set()
            self.get_logger().error(f"Doosan 급정지 요청 실패 ({exc})")

    def _on_quick_stop_done(self, future):
        try:
            if not future.result().success:
                raise RuntimeError("Doosan 제어기가 급정지 요청을 거부했습니다.")
        except Exception as exc:
            with self._state:
                self._stop_fault = True
            self.get_logger().error(f"Doosan 급정지 응답 실패 ({exc})")
        finally:
            with self._state:
                if self._stop_future is future:
                    self._stop_future = None
                    self._stop_idle.set()

    def _can_control(self):
        if (not self._remote_active or self._guard.is_emergency_stopped()
                or self._stop_fault or self._stop_future is not None):
            return False
        if self._executor_seen and (self._executor_busy or not self._executor_remote_ack):
            return False
        return True

    def _watchdog_tick(self):
        with self._state:
            if self._remote_active:
                expired = self._expire_input(monotonic())
                if expired and self._moving:
                    self.quick_stop()
            elif self._moving and self._stop_future is None:
                # 정지 처리 직후 진입한 이동 명령이 남아 있으면 즉시 후속 급정지를 보냅니다.
                self.quick_stop()

    def _expire_input(self, now):
        watchdog = self._pipeline.watchdog
        if not watchdog.is_expired(now):
            return False
        if watchdog.last_heartbeat_sec is not None:
            self.get_logger().warn("원격 입력이 끊겨 이동 및 그리퍼 대기 명령을 초기화했습니다.")
            self._reset_pipeline()
        return True

    def _on_hand_cmd(self, message):
        with self._state:
            if not self._can_control():
                return
            try:
                command = json.loads(message.data)
                if not isinstance(command, dict):
                    raise ValueError("JSON 객체가 필요합니다.")
                deltas = (command["dx_px"], command["dy_px"], command.get("dz_px", 0.0))
                if not all(type(value) in (float, int) and isfinite(value) for value in deltas):
                    raise ValueError("이동량은 유한한 숫자여야 합니다.")
                if not isfinite(hypot(*deltas[:2])):
                    raise ValueError("XY 이동량의 크기가 너무 큽니다.")
                if "gripper_closed" in command and type(command["gripper_closed"]) is not bool:
                    raise ValueError("그리퍼 상태는 불리언이어야 합니다.")
                if "stop" in command and type(command["stop"]) is not bool:
                    raise ValueError("정지 플래그는 불리언이어야 합니다.")
            except (ValueError, KeyError, OverflowError, RecursionError) as exc:
                self.get_logger().warn(f"잘못된 원격 입력을 무시했습니다 ({exc})", throttle_duration_sec=2.0)
                return
            now = monotonic()
            self._expire_input(now)
            if command.get("stop"):
                self._pipeline.ema.reset()
                self._deltas = (0.0, 0.0, 0.0)
                if self._moving:
                    self.quick_stop()
                if "gripper_closed" in command:
                    self._pipeline.gripper.update(command["gripper_closed"], now)
                self._pipeline.watchdog.feed(now)
                return
            dx, dy, dz = deltas
            # XY와 Z의 데드존을 따로 적용하고 세 축을 같은 EMA로 평활화합니다.
            xy = self._pipeline.deadzone.apply(dx, dy)
            filtered_z, _ = self._pipeline.deadzone.apply(dz, 0.0)
            filtered = self._pipeline.ema.update((*xy, filtered_z))
            if not all(map(isfinite, filtered)):
                self.get_logger().warn("원격 이동량의 범위를 초과해 입력을 초기화했습니다.")
                self._reset_pipeline()
                return
            self._deltas = filtered
            if "gripper_closed" in command:
                self._pipeline.gripper.update(command["gripper_closed"], now)
            self._pipeline.watchdog.feed(now)

    def _target_pose(self, current, step, rz):
        bounds = self._guard.bounds
        # 제어기 좌표의 반올림을 고려해 작업 공간 경계 안쪽에 여유를 둡니다.
        margin = 0.001
        limits = [(bounds["x_min"], bounds["x_max"]), (bounds["y_min"], bounds["y_max"]),
                  (self._guard.z_min_mm, bounds["z_max"])]
        # 작업 공간과 이번 틱의 축별 이동 범위의 교집합을 사용합니다.
        ranges = [
            (max(low + margin, value - self._max_step), min(high - margin, value + self._max_step))
            for value, (low, high) in zip(current, limits, strict=True)
        ]
        if any(low > high for low, high in ranges):
            return None
        x, y, z = [
            max(low, min(high, value + delta))
            for value, delta, (low, high) in zip(current, step, ranges, strict=True)
        ]
        radius = self._guard.max_radial_reach_mm - margin
        if hypot(x, y) > radius:
            # 현재 XY에서 목표 방향으로 반경 경계까지 이동합니다.
            ax, ay = (max(low, min(high, value)) for value, (low, high) in zip(current[:2], ranges[:2]))
            if hypot(ax, ay) > radius:
                ax, ay = (max(low, min(high, 0.0)) for low, high in ranges[:2])
                if hypot(ax, ay) > radius:
                    return None
            low, high = 0.0, 1.0
            for _ in range(32):
                mid = (low + high) / 2
                if hypot(ax + mid * (x - ax), ay + mid * (y - ay)) <= radius:
                    low = mid
                else:
                    high = mid
            x, y = ax + low * (x - ax), ay + low * (y - ay)
        return [x, y, z, rz]

    def _is_current(self, pipeline):
        return self._can_control() and self._pipeline is pipeline and not self._expire_input(monotonic())

    def _move(self, step, pipeline):
        current = self._robot.get_current_pose_6d()
        rz = top_down_effective_rz(current)
        if rz is None:
            self.get_logger().warn("하향 자세에서만 원격 이동할 수 있습니다.", throttle_duration_sec=2.0)
            return
        xyz = current[:3]
        target = self._target_pose(xyz, step, rz)
        if target is None or hypot(*(a - b for a, b in zip(target[:3], xyz))) < self._min_move:
            return
        check = self._guard.validate_pose(target)
        if not check.ok:
            self.get_logger().warn(check.message, throttle_duration_sec=2.0)
            return
        j6 = self._robot.predict_linear_j6([*target[:3], 0.0, 180.0, rz])
        if not all(map(self._guard.check_j6_limit, j6)):
            self.get_logger().warn("원격 이동 목표가 J6 한계를 벗어났습니다.", throttle_duration_sec=2.0)
            return
        with self._state:
            if not self._is_current(pipeline):
                return
            self._moving = True
            self._motion_idle.clear()
        try:
            self._robot.move_linear_4d(check.clamped_pose, self._velocity, self._acceleration)
            x, y, z, rz_out = check.clamped_pose
            self.get_logger().info(
                f"원격 이동 → [{x:.1f}, {y:.1f}, {z:.1f}, {rz_out:.1f}]",
                throttle_duration_sec=0.3,
            )
        finally:
            with self._state:
                self._moving = False
                self._motion_idle.set()

    def _control_tick(self):
        with self._state:
            if not self._can_control() or self._expire_input(monotonic()):
                return
            pipeline = self._pipeline
            step = [max(-self._max_step, min(self._max_step, delta * self._scale))
                    for delta in self._deltas]
        try:
            if hypot(*step) >= self._min_move:
                self._move(step, pipeline)
        except MotionUnreachableError as exc:
            self.get_logger().warn(f"원격 이동 목표에 도달할 수 없습니다 ({exc})", throttle_duration_sec=2.0)
        except Exception as exc:
            self.get_logger().error(f"원격 이동 실패 ({exc})", throttle_duration_sec=2.0)
            return
        with self._state:
            if not self._is_current(pipeline):
                return
            closed = pipeline.gripper.stable_closed
            if closed == self._commanded_closed:
                return
        try:
            if closed:
                self._gripper.move_mm(self._close_width, self._force)
                self.get_logger().info(f"원격 그리퍼 → 닫힘 ({self._close_width:.0f}mm)")
            else:
                self._gripper.open_mm(self._open_width, self._force)
                self.get_logger().info(f"원격 그리퍼 → 열림 ({self._open_width:.0f}mm)")
        except Exception as exc:
            self.get_logger().error(f"원격 그리퍼 명령 실패 ({exc})", throttle_duration_sec=2.0)
            return
        with self._state:
            self._commanded_closed = closed
            if pipeline is not self._pipeline:
                # 명령 대기 중 세션이 바뀌었으면 새 디바운서를 실제 명령 상태에 맞춥니다.
                self._pipeline.gripper = GripperGestureDebouncer(pipeline.gripper.debounce_sec)
                self._pipeline.gripper.stable_closed = closed

    def _publish_health(self):
        with self._state:
            stopped = self._guard.is_emergency_stopped()
            fault = self._stop_fault
            active = self._remote_active
        message = f"dry_run={str(self._dry_run).lower()}, remote_active={str(active).lower()}"
        if stopped:
            message += ", 비상 정지 활성"
        if fault:
            message += ", 급정지 오류 래치"
        self._health.publish(NodeHealth(
            node_name="remote_node", alive=True,
            state="ERROR" if (stopped or fault) else ("WARN" if active else "OK"),
            message=message, stamp=self.get_clock().now().to_msg(),
        ))


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=4)
    node = None
    try:
        node = RemoteNode()
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
