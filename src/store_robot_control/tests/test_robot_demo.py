import json

import pytest

from conftest import TEST_WORKSPACE
from store_robot_control.core.safety_guard import SafetyGuard
from store_robot_control.nodes.robot_demo import build_scenarios, summarize_blackbox


SAFETY = ["z_min_violation", "out_of_reach", "invalid_action"]
MOTION = ["home", "pick_place", "move_to_buffer", "restore_from_buffer", "reorient", "scan_pose"]


@pytest.mark.parametrize("suite, names", [
    ("all", SAFETY + MOTION + ["estop"]),
    ("safety", SAFETY),
    ("motion", MOTION),
    ("estop", ["estop"]),
])
def test_suites(suite, names):
    scenarios = build_scenarios(TEST_WORKSPACE, suite)
    assert [item.name for item in scenarios] == names
    guard = SafetyGuard(TEST_WORKSPACE)
    for item in scenarios:
        if item.name in SAFETY[:2]:
            assert guard.validate_pose(item.target_pose).error_code == item.expected_error_code
            assert item.expected_stopped_for_safety
        elif item.name == "invalid_action":
            assert item.action_type == "give_expired_box"
            assert item.expected_error_code == "INVALID_ACTION"
            assert not item.expected_stopped_for_safety
        elif item.name == "estop":
            assert item.action_type == "pick_place"
            assert item.emergency_stop and item.expected_stopped_for_safety
            assert item.expected_error_code == "CANCELED"
        else:
            assert item.expected_error_code == "OK"
            assert not item.expected_stopped_for_safety


def test_pose_overrides():
    a = (350.0, 10.0, 35.0, 15.0)
    b = (410.0, -110.0, 50.0, -15.0)
    by_name = {item.name: item for item in build_scenarios(TEST_WORKSPACE, target_pose=a, dest_pose=b)}
    slot = tuple(TEST_WORKSPACE["zones"]["buffer_slots"][0])
    for name, source, dest in [
        ("pick_place", a, b), ("move_to_buffer", a, slot), ("restore_from_buffer", slot, a),
        ("estop", a, b),
    ]:
        assert (by_name[name].target_pose, by_name[name].dest_pose) == (source, dest)
    assert by_name["reorient"].target_pose == a
    assert by_name["z_min_violation"].target_pose == (350.0, 10.0, 10.0, 15.0)
    assert by_name["out_of_reach"].target_pose == (350.0, 10.0, 420.0, 15.0)
    with pytest.raises(ValueError, match="Z_MIN_VIOLATION"):
        build_scenarios(TEST_WORKSPACE, target_pose=(350.0, 10.0, 0.0, 0.0))


def test_summarize_blackbox(tmp_path):
    assert summarize_blackbox(tmp_path) == []
    rows = [
        {"action_type": "home", "error_code": "OK", "task_id": "a"},
        {"action_type": "pick_place", "error_code": "CANCELED", "task_id": "b"},
    ]
    (tmp_path / "robot_blackbox.jsonl").write_text(
        "\n".join([json.dumps(rows[0]), "{", "null", json.dumps(rows[1])]), encoding="utf-8",
    )
    assert summarize_blackbox(tmp_path, limit=1) == rows[1:]
    assert summarize_blackbox(tmp_path, task_ids={"a"}) == rows[:1]


def test_demo_runner_rejects_dual_servers_and_cancels_on_timeout():
    from types import SimpleNamespace
    from store_robot_control.nodes.robot_demo import DemoRunner, Scenario

    # 1) executor_node와 mock_robot이 동시에 그래프에 존재하면 connect()가 거부해야 함
    runner = DemoRunner.__new__(DemoRunner)
    runner.timeout = 0.2
    runner.spin_once = lambda *_a, **_kw: None
    runner.health = {}
    runner.client = SimpleNamespace(server_is_ready=lambda: True)
    runner.node = SimpleNamespace(
        get_node_names=lambda: ["/executor_node", "/mock_robot"],
        count_publishers=lambda _topic: 2,
    )
    with pytest.raises(RuntimeError, match="동시에 감지"):
        runner.connect()

    # 2) 결과 대기 시간 초과 시 실행 중인 목표에 cancel_goal_async()를 호출해야 함
    cancelled = []
    done_state = {"done": False}

    def cancel():
        cancelled.append(True)
        done_state["done"] = True
        return SimpleNamespace(done=lambda: True)

    handle = SimpleNamespace(
        accepted=True,
        goal_id=SimpleNamespace(uuid=bytes(16)),
        get_result_async=lambda: SimpleNamespace(done=lambda: done_state["done"]),
        cancel_goal_async=cancel,
    )
    runner.task_ids = set()
    runner.goal_type = lambda **kwargs: kwargs
    runner.client = SimpleNamespace(
        send_goal_async=lambda *_a, **_kw: SimpleNamespace(done=lambda: True, result=lambda: handle),
    )
    runner.timeout = 0.01
    with pytest.raises(TimeoutError, match="목표 취소 및 정지 확인"):
        runner._send(Scenario("home", "home"))
    assert cancelled == [True]
