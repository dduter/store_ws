import json
import sys
from math import hypot
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from threading import Event
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.action import CancelResponse, GoalResponse
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from store_interfaces.action import ExecuteRobotTask

from store_robot_control.hardware.dsr_interface import DoosanRobotInterface, MotionUnreachableError
from store_robot_control.nodes.executor_node import ExecutorNode
from conftest import make_test_config_dir


TARGET = [350.0, 20.0, 40.0, 0.0]
DESTINATION = [420.0, -200.0, 40.0, 0.0]


class GoalHandle:
    def __init__(self, action, target=TARGET, destination=DESTINATION, category="paper_box"):
        self.request = ExecuteRobotTask.Goal(
            action_type=action, product_category=category,
            target_pose=target, dest_pose=destination,
        )
        self.goal_id = SimpleNamespace(uuid=bytes(range(16)))
        self.is_cancel_requested = False
        self.is_active = True
        self.status = None
        self.feedback = []

    def execute(self):
        pass

    def publish_feedback(self, feedback):
        self.feedback.append(feedback.stage)

    def succeed(self):
        self.status = "succeeded"
        self.is_active = False

    def abort(self):
        self.status = "aborted"
        self.is_active = False

    def canceled(self):
        assert self.is_cancel_requested
        self.status = "canceled"
        self.is_active = False


@pytest.fixture
def node(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    rclpy.init()
    executor = None
    try:
        cfg_dir = make_test_config_dir(tmp_path)
        executor = ExecutorNode(parameter_overrides=[
            Parameter("config_dir", value=str(cfg_dir)),
            Parameter("dry_run", value=True),
            Parameter("step_delay_sec", value=0.0),
            Parameter("log_dir", value=str(tmp_path)),
        ])
        assert executor._dsr_node is None
        assert executor._stop_client is None
        yield executor
    finally:
        if executor is not None:
            executor.destroy_node()
        rclpy.try_shutdown()


def run_goal(node, handle):
    assert node.goal_callback(handle.request) == GoalResponse.ACCEPT
    node.handle_accepted_callback(handle)
    result = node.execute_callback(handle)
    assert not node._busy
    return result


def test_doosan_simulated_pose_and_joints():
    robot = DoosanRobotInterface(
        "dsr01", "m0609", "tool", "tcp", [380, 0, 380, 0, 180, 0],
        [0, 0, 90, 0, 90, 0], dry_run=True,
    )
    robot.move_linear_4d([350, 20, 250, -90], 60, 60)
    assert robot.get_current_pose_6d() == [350, 20, 250, 0, 180, -90]
    assert robot.get_current_joints()[5] == -90
    robot.move_joint([1, 2, 3, 4, 5, 90], 60, 60)
    assert robot.get_current_joints() == [1, 2, 3, 4, 5, 90]
    assert robot.get_current_pose_4d() == [350, 20, 250, 90]


@pytest.mark.parametrize("action", [
    "home", "scan_pose", "pick_place", "move_to_buffer", "restore_from_buffer", "reorient",
])
def test_all_actions(node, action, tmp_path):
    poses = node._guard.poses
    handle = GoalHandle(action, destination=DESTINATION)
    result = run_goal(node, handle)
    assert result.success
    assert result.error_code == "OK"
    assert not result.stopped_for_safety
    assert handle.status == "succeeded"
    if action in {"home", "scan_pose"}:
        pose = poses[f"{action}_cartesian"]
        assert list(result.final_pose) == [*pose[:3], pose[5]]
        assert handle.feedback == [action]
    elif action == "reorient":
        assert handle.feedback == [
            "open_gripper", "approach", "descend", "grasp", "pivot_lift", "pivot_tilt_j5",
            "drag_over", "release_flip", "lift_after_flip", "restore_j5", "retreat",
        ]
        assert list(result.final_pose) == [350, 20, 260, 0]
        assert -150 < node._robot.get_current_joints()[5] < 150
    else:
        assert handle.feedback == [
            "open_gripper", "approach", "descend", "grasp", "lift",
            "transit", "place", "release", "retreat",
        ]
        assert list(result.final_pose) == [*DESTINATION[:2], node._guard.safe_transit_z(DESTINATION[2]), DESTINATION[3]]
    entry = json.loads((tmp_path / "robot_blackbox.jsonl").read_text())
    assert entry["action_type"] == action
    assert entry["error_code"] == "OK"


@pytest.mark.parametrize(("action", "target", "code", "safety"), [
    ("pick_place", [350.0, 20.0, 10.0, 0.0], "Z_MIN_VIOLATION", True),
    ("give_expired_box", TARGET, "INVALID_ACTION", False),
    ("unknown", TARGET, "INVALID_ACTION", False),
])
def test_invalid_tasks(node, action, target, code, safety):
    handle = GoalHandle(action, target)
    result = run_goal(node, handle)
    assert not result.success
    assert result.error_code == code
    assert result.stopped_for_safety == safety
    assert handle.status == "aborted"
    assert handle.feedback == []


@pytest.mark.parametrize("action", ["pick_place", "reorient"])
def test_grasp_failure_retreats_before_opening(node, action):
    node._gripper.simulate_grip_fail = True
    rz = -90.0 if action == "reorient" else 0.0
    handle = GoalHandle(action, target=[*TARGET[:3], rz])
    result = run_goal(node, handle)
    assert not result.success
    assert result.stopped_for_safety
    assert result.error_code == "GRASP_FAIL"
    x, y = TARGET[:2]
    edge_ratio = 1 - 35.0 / hypot(x, y) if action == "reorient" else 1.0
    assert list(result.final_pose) == pytest.approx([x * edge_ratio, y * edge_ratio, 260, rz])
    assert node._gripper.get_width_mm() == 95
    assert handle.feedback[-2:] == ["grasp_fail_retreat", "grasp_fail_release"]


@pytest.mark.parametrize("stop", ["emergency", "cancel", "remote", "emergency_then_remote"])
@pytest.mark.parametrize(("action", "method", "missed_arrival"), [
    ("pick_place", "move_linear_4d", False),
    ("pick_place", "move_linear_4d", True),
    ("reorient", "move_linear_6d", True),
])
def test_stop_during_blocking_motion(node, monkeypatch, stop, action, method, missed_arrival):
    emergency = stop.startswith("emergency")
    in_motion, release_motion, quick_stop_sent = Event(), Event(), Event()
    move = getattr(node._robot, method)

    def blocking_move(*args, **kwargs):
        in_motion.set()
        assert release_motion.wait(3)
        if missed_arrival:
            raise MotionUnreachableError("목표 좌표 미도달")
        return move(*args, **kwargs)

    stops = []

    def call_async(request):
        from concurrent.futures import Future

        stops.append(request.stop_mode)
        quick_stop_sent.set()
        future = Future()
        future.set_result(SimpleNamespace(success=True))
        return future

    monkeypatch.setattr(node._robot, method, blocking_move)
    node._stop_client = SimpleNamespace(call_async=call_async)
    node._stop_request = SimpleNamespace
    handle = GoalHandle(action)
    node.handle_accepted_callback(handle)
    with ThreadPoolExecutor() as pool:
        result_future = pool.submit(node.execute_callback, handle)
        try:
            assert in_motion.wait(3)
            assert node.goal_callback(handle.request) == GoalResponse.REJECT
            if emergency:
                node._on_emergency_stop(Bool(data=True))
                assert node._guard.is_emergency_stopped()
                node._on_emergency_stop(Bool(data=False))
            elif stop == "cancel":
                assert node.cancel_callback(handle) == CancelResponse.ACCEPT
                handle.is_cancel_requested = True
            if stop in {"remote", "emergency_then_remote"}:
                node._on_remote_override(Bool(data=True))
                node._on_remote_override(Bool(data=False))
            assert quick_stop_sent.is_set()
            assert stops == ([1, 1] if stop == "emergency_then_remote" else [1])
            assert not result_future.done()
        finally:
            release_motion.set()
        result = result_future.result(timeout=3)
    assert not result.success
    assert result.stopped_for_safety == emergency
    assert result.error_code == "CANCELED"
    assert handle.feedback == ["open_gripper", "approach"]
    assert handle.status == ("canceled" if stop == "cancel" else "aborted")
    assert not node._busy


def test_depth_offset_clamps_to_safety_floor(node, monkeypatch):
    assert node._apply_depth is True
    heights = []
    move = node._robot.move_linear_4d

    def record_height(pose, *args, **kwargs):
        heights.append(pose[2])
        return move(pose, *args, **kwargs)

    monkeypatch.setattr(node._robot, "move_linear_4d", record_height)
    result = run_goal(node, GoalHandle("pick_place", target=[350.0, 20.0, 20.0, 0.0]))
    assert result.success
    assert min(heights) == node._guard.z_min_mm


def test_depth_offset_applied_only_to_target_not_dest(node, monkeypatch):
    heights = []
    move = node._robot.move_linear_4d

    def record_height(pose, *args, **kwargs):
        heights.append(pose[2])
        return move(pose, *args, **kwargs)

    monkeypatch.setattr(node._robot, "move_linear_4d", record_height)
    # paper_box grasp_depth_offset_mm = 18.0 -> target 50.0 -> 32.0, dest 60.0 -> unchanged 60.0
    result = run_goal(
        node,
        GoalHandle("pick_place", target=[350.0, 20.0, 50.0, 0.0], destination=[420.0, -100.0, 60.0, 0.0]),
    )
    assert result.success
    assert 32.0 in heights
    assert 60.0 in heights


def test_health_reports_emergency_and_dry_run(node, monkeypatch):
    messages = []
    monkeypatch.setattr(node, "_health", SimpleNamespace(publish=messages.append))
    node._publish_health()
    node._on_emergency_stop(Bool(data=True))
    node._publish_health()
    assert [message.state for message in messages] == ["OK", "ERROR"]
    assert all(message.node_name == "executor_node" for message in messages)
    assert all("dry_run=true" in message.message for message in messages)


def test_quick_stop_failure_latches_fault_and_blocks_goals(node):
    from concurrent.futures import Future

    failed = Future()
    failed.set_result(SimpleNamespace(success=False))
    node._stop_client = SimpleNamespace(call_async=lambda _req: failed)
    node._stop_request = SimpleNamespace
    messages = []
    node._health = SimpleNamespace(publish=messages.append)

    node.quick_stop()
    assert node._stop_fault
    assert node.goal_callback(GoalHandle("home").request) == GoalResponse.REJECT
    node._publish_health()
    assert messages[-1].state == "ERROR"
    assert "급정지 오류" in messages[-1].message


def test_linear_move_rejects_j6_over_limit(node):
    node._robot.move_joint([0, 0, 90, 0, 90, 120], 60, 60)
    handle = GoalHandle("pick_place", target=[350.0, 20.0, 40.0, 170.0])
    result = run_goal(node, handle)
    assert not result.success
    assert result.stopped_for_safety
    assert result.error_code == "J6_LIMIT_VIOLATION"


@pytest.mark.parametrize("mode", ["in_place_yaw_180", "flip_180_in_place"])
def test_legacy_reorient_from_home(node, mode):
    node._grasp.categories["paper_box"]["reorient_mode"] = mode
    handle = GoalHandle("reorient", target=[350.0, 20.0, 40.0, -90.0])
    result = run_goal(node, handle)
    assert result.success
    assert result.error_code == "OK"
    assert node._robot.get_current_joints()[5] == 90.0


def test_ensure_tool_and_tcp_skips_when_matching_and_switches_mode_when_needed():
    robot = DoosanRobotInterface(
        "dsr01", "m0609", "Tool_Weight", "GripperDA_v1", [380, 0, 380, 0, 180, 0],
        [0, 0, 90, 0, 90, 0], dry_run=True,
    )
    calls = []
    robot._api = SimpleNamespace(
        get_tool=lambda: "Tool_Weight",
        get_tcp=lambda: "GripperDA_v1",
        set_tool=lambda name: calls.append(("set_tool", name)) or 0,
        set_tcp=lambda name: calls.append(("set_tcp", name)) or 0,
    )
    robot._ensure_tool_and_tcp("Tool_Weight", "GripperDA_v1")
    assert calls == []

    state = {"tool": "Old_Tool", "tcp": "GripperDA_v1"}

    def set_tool(name):
        calls.append(("set_tool", name))
        state["tool"] = name
        return 0

    def set_tcp(name):
        calls.append(("set_tcp", name))
        state["tcp"] = name
        return 0

    robot._api = SimpleNamespace(
        get_tool=lambda: state["tool"],
        get_tcp=lambda: state["tcp"],
        get_robot_mode=lambda: 1,
        set_robot_mode=lambda mode: calls.append(("set_mode", mode)) or 0,
        set_tool=set_tool,
        set_tcp=set_tcp,
    )
    robot._ensure_tool_and_tcp("Tool_Weight", "GripperDA_v1")
    assert calls == [("set_mode", 0), ("set_tool", "Tool_Weight"), ("set_mode", 1)]


def test_wait_for_dsr_services_waits_and_times_out():
    from store_robot_control.hardware.dsr_interface import wait_for_dsr_services

    attempts = iter([False, True])
    api = SimpleNamespace(
        _ros2_get_current_tool=SimpleNamespace(wait_for_service=lambda timeout_sec: next(attempts)),
    )
    wait_for_dsr_services(api, timeout_sec=5.0)

    never_ready = SimpleNamespace(
        _ros2_get_current_tool=SimpleNamespace(wait_for_service=lambda timeout_sec: False),
    )
    with pytest.raises(RuntimeError, match="대기 시간"):
        wait_for_dsr_services(never_ready, timeout_sec=0.5)


def test_zyz_gimbal_lock_at_ry_180_does_not_false_trigger_j6(node):
    # 실물 M0609 home 자세의 ZYZ 짐벌락 표현: rx≈140.9, ry≈180.0, rz≈140.9 (실제 yaw≈0°)
    node._robot.set_simulated_reference_pose([367.2, 3.7, 182.6, 140.9, 179.98, 140.9])
    handle = GoalHandle("pick_place", target=[350.0, 20.0, 40.0, 0.0])
    result = run_goal(node, handle)
    assert result.success
    assert result.error_code == "OK"
    assert abs(node._robot.get_current_joints()[5]) < 1.0


def test_reorient_rejects_non_reorientable_category(node):
    handle = GoalHandle("reorient", category="cigarette_mock")
    result = run_goal(node, handle)
    assert not result.success
    assert result.error_code == "INVALID_ACTION"
    assert not result.stopped_for_safety
    assert handle.feedback == []


def test_reorient_second_grasp_failure_retreats_at_offset(node, monkeypatch):
    node._grasp.categories["paper_box"]["reorient_mode"] = "two_stage_flip"
    detections = iter([True, False])
    monkeypatch.setattr(node._gripper, "is_grip_detected", lambda: next(detections))
    handle = GoalHandle("reorient")
    result = run_goal(node, handle)
    assert result.error_code == "GRASP_FAIL"
    assert list(result.final_pose) == [350, 55, 260, 0]
    assert handle.feedback[-2:] == ["grasp_fail_retreat", "grasp_fail_release"]


def test_no_arrival_reports_out_of_reach(node, monkeypatch):
    def fail(*args, **kwargs):
        raise MotionUnreachableError("목표 좌표 미도달")

    monkeypatch.setattr(node._robot, "move_linear_6d", fail)
    handle = GoalHandle("reorient")
    result = run_goal(node, handle)
    assert not result.success
    assert result.error_code == "OUT_OF_REACH"
    assert result.stopped_for_safety
    assert handle.feedback == ["open_gripper", "approach"]


@pytest.mark.parametrize("use_6d", [True, False])
def test_movel_checks_actual_xyz_and_orientation_on_hardware(monkeypatch, use_6d):
    from store_robot_control.hardware import dsr_interface

    commands = []
    # 4D 하향 테스트에서는 ZYZ 짐벌락 등가 표현(140.9, 180, 140.9 == 0, 180, 0)도 통과해야 합니다.
    actual = [350, 20, 375, 20, 90, 0] if use_6d else [350, 20, 375, 140.9, 180.0, 140.9]
    api = SimpleNamespace(
        DR_BASE=0, DR_MV_MOD_ABS=0,
        get_tool=lambda: "tool", get_tcp=lambda: "tcp",
        movel=lambda pose, **kwargs: commands.append(pose) or 0,
        get_current_posx=lambda **kwargs: (actual, 0),
        get_current_posj=lambda: [0, 0, 90, 0, 90, -20],
        get_solution_space=lambda joints: 2,
        ikin=lambda pose, solution, ref: [0, 0, 90, 0, 90, 10],
    )
    monkeypatch.setattr(dsr_interface, "load_dsr_api", lambda *args: api)
    monkeypatch.setitem(sys.modules, "DR_common2", SimpleNamespace(posx=list, posj=list))
    robot = DoosanRobotInterface(
        "dsr01", "m0609", "tool", "tcp", actual, [0, 0, 90, 0, 90, 0],
    )
    assert robot.predict_linear_j6([350, 20, 390, 20, 90, 0]) == (-20, 10)
    move = robot.move_linear_6d if use_6d else robot.move_linear_4d
    orientation = [20, 90, 0] if use_6d else [0]
    move([350, 20, 390, *orientation], 60, 60)
    with pytest.raises(MotionUnreachableError, match="16.0mm > 15.0mm"):
        move([350, 20, 391, *orientation], 60, 60)
    assert commands[-1] == [350, 20, 391, *(orientation if use_6d else [0, 180, 0])]


def test_movel_6d_rejects_orientation_mismatch_at_same_xyz(monkeypatch):
    from store_robot_control.hardware import dsr_interface

    # XYZ는 정확히 일치(0mm)하지만 tilt_90 자세에 도달하지 못하고 하향(0,180,0)에 머문 경우
    actual = [350.0, 20.0, 260.0, 0.0, 180.0, 0.0]
    api = SimpleNamespace(
        DR_BASE=0, DR_MV_MOD_ABS=0,
        get_tool=lambda: "tool", get_tcp=lambda: "tcp",
        movel=lambda pose, **kwargs: 0,
        get_current_posx=lambda **kwargs: (actual, 0),
    )
    monkeypatch.setattr(dsr_interface, "load_dsr_api", lambda *args: api)
    monkeypatch.setitem(sys.modules, "DR_common2", SimpleNamespace(posx=list, posj=list))
    robot = DoosanRobotInterface(
        "dsr01", "m0609", "tool", "tcp", actual, [0, 0, 90, 0, 90, 0],
    )
    with pytest.raises(MotionUnreachableError, match="회전 오차 90.0deg > 10.0deg"):
        robot.move_linear_6d([350.0, 20.0, 260.0, 0.0, 90.0, 0.0], 60, 60)


def test_teach_zones_fails_on_tcp_error_and_prints_effective_rz(capsys):
    from store_robot_control.nodes import teach_zones

    bad_api = SimpleNamespace(
        get_tool=lambda: "Tool_Weight",
        get_tcp=lambda: "Flange",
        get_robot_mode=lambda: 1,
        set_robot_mode=lambda mode: 0,
        set_tool=lambda name: 0,
        set_tcp=lambda name: -1,
    )
    with pytest.raises(RuntimeError, match="set_tcp"):
        teach_zones._ensure_tool_tcp(bad_api, {"tool_name": "Tool_Weight", "tcp_name": "GripperDA_v1"})

    # 하향 자세(rx=100, ry=180, rz=145 -> 유효 rz = +45.0°)
    top_down_api = SimpleNamespace(
        DR_BASE=0,
        get_current_posx=lambda **kwargs: ([350.0, 20.0, 40.0, 100.0, 180.0, 145.0], 0),
        get_current_posj=lambda: [0.0, 0.0, 90.0, 0.0, 90.0, 45.0],
    )
    teach_zones._read_coordinates(top_down_api)
    out = capsys.readouterr().out
    assert "[350.0, 20.0, 40.0, 45.0]" in out

    # 비하향 자세(ry=90°)는 4D 복사 불가 안내 출력 및 4D 저장 거부
    tilted_api = SimpleNamespace(
        DR_BASE=0,
        get_current_posx=lambda **kwargs: ([350.0, 20.0, 40.0, 0.0, 90.0, 0.0], 0),
        get_current_posj=lambda: [0.0, 0.0, 90.0, 0.0, 0.0, 0.0],
    )
    pose, joints = teach_zones._read_coordinates(tilted_api)
    assert "4D 복사 불가" in capsys.readouterr().out
    with pytest.raises(ValueError, match="하향 자세"):
        teach_zones._update_workspace("4", {"poses": {}, "zones": {}}, pose, joints)


@pytest.mark.parametrize("rz", [0.0, -90.0, 90.0, -45.0, 45.0])
def test_two_stage_flip_from_measured_home(node, monkeypatch, rz):
    node._grasp.categories["paper_box"]["reorient_mode"] = "two_stage_flip"
    from conftest import REPO_CONFIG
    from store_robot_control.core.safety_guard import load_settings

    poses = load_settings(REPO_CONFIG / "workspace_zones.yaml")["poses"]
    node._robot.move_joint(poses["home_joint_deg"], 60, 60)
    node._robot.set_simulated_reference_pose(poses["home_cartesian"])
    move = node._robot.move_linear_6d
    recorded_j6 = []

    def record_move(*args):
        move(*args)
        recorded_j6.append(node._robot.get_current_joints()[5])

    monkeypatch.setattr(node._robot, "move_linear_6d", record_move)
    result = run_goal(node, GoalHandle("reorient", target=[*TARGET[:3], rz]))
    assert result.success
    assert recorded_j6 and all(-150 < j6 < 150 for j6 in recorded_j6)
    assert node._robot.get_current_pose_6d() == [350, 20, 260, 90, 90, 90]



def test_remote_override_rejects_goals_and_reports_health(node, monkeypatch):
    messages = []
    monkeypatch.setattr(node, "_health", SimpleNamespace(publish=messages.append))
    handle = GoalHandle("home")
    node._on_remote_override(Bool(data=True))
    assert node.goal_callback(handle.request) == GoalResponse.REJECT
    node._publish_health()
    assert messages[-1].state == "WARN"
    assert "원격 모드 활성" in messages[-1].message
    assert "remote_override=true" in messages[-1].message
    assert "busy=false" in messages[-1].message
    # 원격 모드 발행자가 존재했다가 사라지면(크래시) 자동 실행 잠금이 자동 해제되어야 함
    pubs = iter([1, 0])
    monkeypatch.setattr(node, "count_publishers", lambda _topic: next(pubs, 0))
    node._publish_health()
    assert node._remote_override is True
    node._publish_health()
    assert node._remote_override is False
    assert run_goal(node, handle).success


@pytest.mark.parametrize("live_j5", [90.0, -90.0])
def test_gentle_flip_uses_live_j5_and_caps_joint_speed(node, monkeypatch, live_j5):
    node._grasp.categories["paper_box"]["reorient_mode"] = "gentle_flip"
    move_joint = node._robot.move_joint
    move_linear = node._robot.move_linear_4d
    commands = []
    node._velocity = 20.0
    node._apply_depth = False

    def linear(pose, *args, **kwargs):
        move_linear(pose, *args, **kwargs)
        if pose[2] == 310.0:
            joints = node._robot.get_current_joints()
            joints[4] = live_j5
            move_joint(joints, 20.0, 30.0)

    def joint(joints, velocity, acceleration):
        before = node._robot.get_current_joints()
        move_joint(joints, velocity, acceleration)
        commands.append((before, node._robot.get_current_joints(), velocity, acceleration))

    monkeypatch.setattr(node._robot, "move_joint", joint)
    monkeypatch.setattr(node._robot, "move_linear_4d", linear)
    result = run_goal(node, GoalHandle("reorient", target=[350.0, 20.0, 220.0, 0.0]))
    assert result.success
    before, tilted, velocity, acceleration = commands[0]
    assert before[4] == live_j5
    assert tilted[:4] == before[:4] and tilted[5] == before[5]
    assert tilted[4] == (50.0 if live_j5 > 0 else -50.0)
    assert (velocity, acceleration) == (20.0, 30.0)
    assert commands[1] == (tilted, before, 20.0, 30.0)


def test_edge_pivot_uses_measured_tcp_and_restores_only_live_j5(node, monkeypatch):
    node._velocity = 20.0
    node._apply_depth = False
    robot = node._robot
    move_joint = robot.move_joint
    move_linear = robot.move_linear_6d
    fk_samples, joint_commands, tilted_moves = [], [], []
    measured = [510.0, 8.0, 225.0, 2.0, 120.0, 0.0]

    def predict(joints):
        fk_samples.append(joints[4])
        return measured

    def joint(joints, vel, acc):
        joint_commands.append((joints.copy(), vel, acc))
        move_joint(joints, vel, acc)
        if len(joint_commands) == 1:
            robot._pose = measured.copy()

    def linear(pose, vel, acc):
        move_linear(pose, vel, acc)
        if pose[4] == measured[4]:
            tilted_moves.append((pose, vel, acc))
            robot._joints[0] = 12.0

    monkeypatch.setattr(robot, "predict_forward_pose", predict)
    monkeypatch.setattr(robot, "move_joint", joint)
    monkeypatch.setattr(robot, "move_linear_6d", linear)
    result = run_goal(node, GoalHandle("reorient", target=[332.5, 0.0, 40.0, 0.0]))

    assert result.success
    assert fk_samples == [65.0, 40.0, 65.0, 90.0]
    assert tilted_moves == [
        ([565.0, 8.0, 65.0, *measured[3:]], 20.0, 30.0),
        ([565.0, 8.0, 225.0, *measured[3:]], 20.0, 30.0),
    ]
    assert joint_commands == [
        ([0.0, 0.0, 90.0, 0.0, 40.0, 0.0], 20.0, 30.0),
        ([12.0, 0.0, 90.0, 0.0, 90.0, 0.0], 20.0, 30.0),
    ]


@pytest.mark.parametrize("unsafe_j5", [30.0, 54.0, None])
def test_edge_pivot_blocks_unsafe_live_tilt(node, monkeypatch, unsafe_j5):
    robot = node._robot
    move_linear = robot.move_linear_4d

    def linear(pose, *args, **kwargs):
        move_linear(pose, *args, **kwargs)
        if unsafe_j5 is not None and pose[2] == 110.0:
            robot._joints[4] = unsafe_j5

    def predict(joints):
        assert unsafe_j5 is None
        return [721.0, 0.0, 230.0, 0.0, 120.0, 0.0]

    monkeypatch.setattr(robot, "move_linear_4d", linear)
    monkeypatch.setattr(robot, "predict_forward_pose", predict)
    monkeypatch.setattr(robot, "move_joint", lambda *_: pytest.fail("안전 검사 실패 후 J5 회전"))
    node._apply_depth = False
    handle = GoalHandle("reorient")
    result = run_goal(node, handle)

    assert result.error_code == "OUT_OF_REACH"
    assert result.stopped_for_safety
    assert handle.feedback[-1] == "pivot_lift"


def test_edge_pivot_caps_live_j5_at_77_deg_safely(node, monkeypatch):
    # 실물 [312.57, 0, 102.1] 측정값 J5=+77.76°에서 특이점 여유(|J5|>=20°)를 지키며 50° 완주
    robot = node._robot
    move_linear = robot.move_linear_4d
    move_joint = robot.move_joint
    j5_targets = []

    def linear(pose, *args, **kwargs):
        move_linear(pose, *args, **kwargs)
        if abs(pose[2] - 102.0) < 1.0:
            robot._joints[4] = 77.76

    def joint(joints, vel, acc):
        j5_targets.append(joints[4])
        move_joint(joints, vel, acc)

    monkeypatch.setattr(robot, "move_linear_4d", linear)
    monkeypatch.setattr(robot, "move_joint", joint)
    result = run_goal(node, GoalHandle("reorient", target=[330.0, 0.0, 50.0, 0.0]))
    assert result.success
    assert j5_targets == [pytest.approx(27.76), pytest.approx(77.76)]


def test_edge_pivot_falls_back_to_gentle_flip_at_outer_display_edge(node):
    # [450, 0, 40, 0]은 edge_pivot_flip 드래그가 x_max(720)를 넘지만 gentle_flip은 안전하게 통과
    handle = GoalHandle("reorient", target=[450.0, 0.0, 40.0, 0.0])
    result = run_goal(node, handle)
    assert result.success
    assert "gentle_tilt_j5" in handle.feedback
    assert "place_flip" in handle.feedback


def test_edge_pivot_checks_fkin_before_restore_j5(node, monkeypatch):
    calls = []

    def predict(joints):
        calls.append(joints[4])
        # 기울기 검사 2회(60, 30)는 통과, 복원 검사 첫 단계에서 범위 이탈 시뮬레이션
        if len(calls) > 2:
            return [725.0, 0.0, 230.0, 0.0, 180.0, 0.0]
        return [400.0, 0.0, 200.0, 0.0, 120.0, 0.0]

    monkeypatch.setattr(node._robot, "predict_forward_pose", predict)
    handle = GoalHandle("reorient", target=[350.0, 20.0, 40.0, 0.0])
    result = run_goal(node, handle)
    assert not result.success
    assert result.error_code == "OUT_OF_REACH"
    assert result.stopped_for_safety
    assert handle.feedback[-1] == "lift_after_flip"
