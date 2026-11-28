import json
from contextlib import nullcontext
from math import hypot
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String
from store_interfaces.srv import VerifyRemoteKey

from conftest import make_test_config_dir
from store_robot_control.core.safety_guard import SafetyGuard
from store_robot_control.nodes import hand_client, remote_node
from store_robot_control.nodes.hand_client import (
    keyboard_key_to_command, landmarks_to_hand_command, serialize_hand_command,
)


@pytest.fixture
def rig(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    monkeypatch.setenv("REMOTE_ADMIN_KEY", "관리자 키")
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr(remote_node, "monotonic", lambda: clock.now)
    rclpy.init()
    node = None
    try:
        node = remote_node.RemoteNode(parameter_overrides=[
            Parameter("config_dir", value=str(make_test_config_dir(tmp_path))),
        ])
        assert node._dry_run and node._dsr_node is None
        messages = []
        monkeypatch.setattr(node, "_remote_pub", SimpleNamespace(publish=messages.append))
        yield node, clock, messages
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def verify(node, key="관리자 키", enable=True):
    return node._verify_remote_key(
        VerifyRemoteKey.Request(admin_key=key, enable_remote=enable), VerifyRemoteKey.Response(),
    )


def send(node, dx=0.0, dy=0.0, **kwargs):
    node._on_hand_cmd(String(data=json.dumps({"dx_px": dx, "dy_px": dy, **kwargs})))


def test_auth_override_and_emergency(rig, monkeypatch):
    node, _, messages = rig
    monkeypatch.delenv("REMOTE_ADMIN_KEY")
    assert not verify(node, key="").granted
    monkeypatch.setenv("REMOTE_ADMIN_KEY", "관리자 키")
    assert not verify(node, key="잘못된 키").granted
    assert messages == []
    assert verify(node).granted
    send(node, dx=70)
    node._on_emergency_stop(Bool(data=True))
    assert not node._remote_active
    assert node._deltas == (0.0, 0.0, 0.0)
    assert node._pipeline.watchdog.last_heartbeat_sec is None
    response = verify(node)
    assert not response.granted and "비상 정지" in response.reason
    assert [message.data for message in messages] == [True, False]
    node._on_emergency_stop(Bool(data=False))
    assert not node._remote_active
    assert verify(node).granted
    assert verify(node, enable=False).granted
    assert [message.data for message in messages] == [True, False, True, False]


def test_deadzone_ema_and_step_clamp(rig):
    node, _, _ = rig
    assert verify(node).granted
    start = node._robot.get_current_pose_6d()
    send(node, dx=10, dz_px=-15)
    node._control_tick()
    assert node._robot.get_current_pose_6d() == start
    send(node, dx=55)
    assert node._deltas == pytest.approx([12, 0, 0])
    node._control_tick()
    assert node._robot.get_current_pose_4d() == [383, 0, 380, 0]
    send(node, dx=1000, dz_px=-1000)
    node._control_tick()
    assert node._robot.get_current_pose_6d() == [398, 0, 365, 0, 180, 0]


@pytest.mark.parametrize(("start", "cmd", "axis", "limit"), [
    ([718, 0, 380], (1000, 0, 0), 0, 720),
    ([400, 418, 380], (0, 1000, 0), 1, 420),
    ([400, 0, 398], (0, 0, 1000), 2, 400),
    ([400, 0, 17], (0, 0, -1000), 2, 15),
    ([650, 380, 380], (1000, 1000, 0), None, 760),
])
def test_workspace_boundary(rig, start, cmd, axis, limit):
    node, _, _ = rig
    assert verify(node).granted
    node._robot.set_simulated_reference_pose([*start, 0, 180, 0])
    send(node, *cmd[:2], dz_px=cmd[2])
    node._control_tick()
    pose = node._robot.get_current_pose_4d()
    assert all(abs(a - b) <= 15 for a, b in zip(pose[:3], start))
    assert node._guard.validate_pose(pose).ok
    value = hypot(*pose[:2]) if axis is None else pose[axis]
    assert value == pytest.approx(limit, abs=0.002)
    assert value > limit if limit == 15 else value < limit


def test_lower_box_descent_and_desk_boundary(rig, zoned_workspace, monkeypatch):
    node, _, _ = rig
    monkeypatch.setattr(node, "_guard", SafetyGuard(zoned_workspace))
    assert verify(node).granted
    node._robot.set_simulated_reference_pose([350.0, -325.0, -110.0, 0.0, 180.0, 0.0])
    send(node, dz_px=-1000)
    node._control_tick()
    lowered = node._robot.get_current_pose_4d()
    assert lowered == pytest.approx([350.0, -325.0, -114.999, 0.0])

    send(node, dy=1000, dz_px=0)
    node._control_tick()
    assert node._robot.get_current_pose_4d() == lowered

    # [P1] 데스크 쪽에서 하단 구역으로 경계를 넘으며 동시에 하강할 때도 데스크 z_min(-18)을 넘지 않습니다.
    node._robot.set_simulated_reference_pose([350.0, -315.0, -15.0, 0.0, 180.0, 0.0])
    send(node, dy=-1000, dz_px=-1000)
    node._control_tick()
    crossed = node._robot.get_current_pose_4d()
    assert crossed[1] < -320.0
    assert crossed[2] >= -18.0


def test_gripper_debounce_closes_and_opens(rig):
    node, clock, _ = rig
    assert verify(node).granted
    for now, closed, width in [
        (0.0, True, 110), (0.49, True, 110), (0.5, True, 20),
        (0.6, False, 20), (1.09, False, 20), (1.1, False, 90),
    ]:
        clock.now = now
        send(node, gripper_closed=closed)
        node._control_tick()
        assert node._gripper.get_width_mm() == width


def test_watchdog_clears_motion_and_resets_ema(rig):
    node, clock, _ = rig
    assert verify(node).granted
    send(node, dx=100)
    node._control_tick()
    pose = node._robot.get_current_pose_4d()
    clock.now = 0.501
    node._control_tick()
    assert node._robot.get_current_pose_4d() == pose
    assert node._deltas == (0, 0, 0)
    assert node._pipeline.watchdog.last_heartbeat_sec is None
    send(node, dx=35)
    assert node._deltas == [20, 0, 0]


@pytest.mark.parametrize("payload", [
    "{", "[]", "{}", '{"dx_px":"20","dy_px":1}', '{"dx_px":true,"dy_px":1}',
    '{"dx_px":NaN,"dy_px":0}', '{"dx_px":0,"dy_px":1e400}',
    '{"dx_px":0,"dy_px":0,"gripper_closed":1}',
])
def test_malformed_input_is_ignored(rig, payload):
    node, _, _ = rig
    assert verify(node).granted
    node._on_hand_cmd(String(data=payload))
    assert node._pipeline.watchdog.last_heartbeat_sec is None
    assert node._deltas == (0.0, 0.0, 0.0)
    send(node, dx=100)
    assert node._deltas == [85, 0, 0]


def test_landmarks_and_keyboard_helpers():
    points = [(0.6, 0.4)] * 21
    for tip in (8, 12, 16, 20):
        points[tip - 2] = (0.6, 0.3)
        points[tip] = (0.6, 0.2)
    command = landmarks_to_hand_command(points, 640, 480)
    assert command["dx_px"] == pytest.approx(64)
    assert command["dy_px"] == pytest.approx(48)
    assert command["gripper_closed"] is False
    for tip in (8, 12, 16, 20):
        points[tip] = (0.6, 0.35)
    assert landmarks_to_hand_command(points, 640, 480)["gripper_closed"] is True
    for key, axis, delta in [
        ("w", "dy_px", 40), ("s", "dy_px", -40), ("a", "dx_px", -40),
        ("d", "dx_px", 40), ("r", "dz_px", 40), ("f", "dz_px", -40),
    ]:
        move, closed, quit_requested = keyboard_key_to_command(key)
        assert move[axis] == delta
        assert not (closed or quit_requested)
    cmd, closed, quit_requested = keyboard_key_to_command("g")
    assert closed and not quit_requested
    assert cmd == json.loads(serialize_hand_command(0, 0, gripper_closed=True))
    assert keyboard_key_to_command("g", gripper_closed=True)[1] is False
    assert keyboard_key_to_command("q", gripper_closed=True) == (None, True, True)
    assert keyboard_key_to_command("?") == (None, False, False)


def test_handover_waits_for_executor_idle_and_watchdog_stops_inflight_motion(rig):
    from store_interfaces.msg import NodeHealth

    node, clock, messages = rig
    stops = []
    node.quick_stop = lambda: stops.append("stop")

    # 1) executor_node가 바쁜 상태(busy=true)면 원격 이동을 보류하다가, ack + busy=false 후 허용
    node._on_node_health(NodeHealth(
        node_name="executor_node", alive=True, state="OK",
        message="dry_run=true, busy=true, remote_override=false",
    ))
    assert verify(node, enable=True).granted
    assert not node._can_control()
    start = node._robot.get_current_pose_6d()
    send(node, dx=80)
    node._control_tick()
    assert node._robot.get_current_pose_6d() == start

    node._on_node_health(NodeHealth(
        node_name="executor_node", alive=True, state="WARN",
        message="dry_run=true, busy=false, remote_override=true, 원격 모드 활성",
    ))
    assert node._can_control()
    send(node, dx=80)
    node._control_tick()
    assert node._robot.get_current_pose_6d() != start

    # 2) 이동 중(_moving=True) 워치독 만료 시 독립 감시 타이머가 즉시 급정지 호출
    node._moving = True
    clock.now = 0.6
    node._watchdog_tick()
    assert stops == ["stop"]
    assert node._deltas == (0.0, 0.0, 0.0)
    node._moving = False

    # 3) 원격 해제 후 이동이 남아 있으면 후속 급정지 호출
    assert verify(node, enable=False).granted
    node._moving = True
    node._watchdog_tick()
    assert stops == ["stop", "stop"]
    assert [m.data for m in messages] == [True, False]


@pytest.mark.parametrize("stop_key", [" ", "x"])
def test_keyboard_command_hold_and_stop(stop_key):
    holder = hand_client.KeyboardCommandHolder(hold_sec=0.25)
    neutral = {"dx_px": 0.0, "dy_px": 0.0, "dz_px": 0.0, "gripper_closed": False}
    holder.press("r", 1.0, 30.0)
    assert holder.current_command(1.24) == {**neutral, "dz_px": 30.0}
    assert holder.current_command(1.25) == neutral

    holder.press("w", 2.0)
    holder.press(stop_key, 2.1)
    assert holder.current_command(2.1) == {**neutral, "stop": True}
    assert holder.current_command(2.15) == neutral

    # hold_sec=0에서도 이동 키 입력은 단발 1회 발행되어야 합니다.
    single = hand_client.KeyboardCommandHolder(hold_sec=0.0)
    single.press("a", 3.0, 40.0)
    assert single.current_command(3.0) == {**neutral, "dx_px": -40.0}
    assert single.current_command(3.01) == neutral


def test_keyboard_batch_prioritizes_stop_and_remote_node_clears_ema(rig):
    node, _, _ = rig
    holder = hand_client.KeyboardCommandHolder(hold_sec=0.25)
    holder.press("w", 1.0, 40.0)
    assert not hand_client._apply_key_batch(holder, [" ", "w", "w"], 1.05, 40.0)
    stop_cmd = holder.current_command(1.05)
    assert stop_cmd["stop"] is True and stop_cmd["dy_px"] == 0.0

    stops = []
    node.quick_stop = lambda: stops.append("stop")
    assert verify(node).granted
    send(node, dx=100)
    assert node._deltas[0] > 0.0
    node._moving = True
    node._on_hand_cmd(String(data=hand_client.serialize_hand_command(**stop_cmd)))
    assert node._deltas == (0.0, 0.0, 0.0)
    assert stops == ["stop"]
    node._moving = False
    start = node._robot.get_current_pose_6d()
    node._control_tick()
    assert node._robot.get_current_pose_6d() == start


def test_keyboard_gripper_toggle_preserves_hold_deadline():
    holder = hand_client.KeyboardCommandHolder()
    holder.press("g", 0.0)
    assert holder.current_command(0.0) == {
        "dx_px": 0.0, "dy_px": 0.0, "dz_px": 0.0, "gripper_closed": True,
    }

    holder.press("d", 1.0)
    holder.press("g", 1.1)
    assert holder.current_command(1.1) == {
        "dx_px": 0.0, "dy_px": 0.0, "dz_px": 0.0, "gripper_closed": False,
    }
    assert holder.current_command(1.2) == {
        "dx_px": 40.0, "dy_px": 0.0, "dz_px": 0.0, "gripper_closed": False,
    }
    assert holder.current_command(1.25)["dx_px"] == 0.0


@pytest.mark.parametrize(("display", "no_preview", "quit_key"), [
    (":1", False, ord("q")),
    (":1", False, ord("Q")),
    (":1", False, 27),
    (None, False, None),
    (":1", True, None),
])
def test_webcam_preview_and_headless(monkeypatch, display, no_preview, quit_key):
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
    if display is None:
        monkeypatch.delenv("DISPLAY", raising=False)
    else:
        monkeypatch.setenv("DISPLAY", display)
    monkeypatch.setattr(hand_client, "sleep", lambda _: None)
    frame = SimpleNamespace(shape=(100, 200, 3))
    calls = []
    cv2 = SimpleNamespace(
        flip=lambda frame, _: frame,
        cvtColor=lambda frame, _: frame,
        COLOR_BGR2RGB=1,
    )
    landmarks = [SimpleNamespace(x=0.6, y=0.4)] * 21
    result = SimpleNamespace(multi_hand_landmarks=[SimpleNamespace(landmark=landmarks)])
    if quit_key is not None:
        cv2.line = lambda *args: None
        cv2.circle = lambda *args: calls.append(("circle", args[2]))
        cv2.arrowedLine = lambda *args: calls.append("arrow")
        cv2.putText = lambda *args: calls.append(("text", args[1]))
        cv2.FONT_HERSHEY_SIMPLEX = 0
        cv2.imshow = lambda name, image: calls.append((name, image))
        cv2.waitKey = lambda delay: calls.append(("wait", delay)) or quit_key
        cv2.destroyAllWindows = lambda: calls.append("destroy")
    detector = SimpleNamespace(process=lambda _: result)
    hands = SimpleNamespace(Hands=lambda **_: nullcontext(detector))
    camera = SimpleNamespace(read=lambda: (True, frame))
    published = []
    running = iter([True, False]) if quit_key is None else iter([True])

    hand_client._run_webcam(
        (cv2, hands, camera), published.append, lambda: next(running), 15.0, no_preview,
    )

    if quit_key is None:
        assert calls == []
        assert len(published) == 1
    else:
        assert ("circle", 15) in calls
        assert "arrow" in calls
        assert ("text", "Q/ESC: quit") in calls
        assert calls[-3:] == [("Store Remote Hand Control", frame), ("wait", 1), "destroy"]
        assert published == []
