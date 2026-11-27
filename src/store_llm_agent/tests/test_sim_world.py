"""STOREPLAN SimWorld 및 오프라인 벤치마크 단위 테스트."""

from store_llm_agent.core.action_domain import RobotActionStep
from store_llm_agent.core.sim_world import (
    SimWorld,
    run_offline_benchmark,
)


def test_sim_world_drag_toggle_and_safety_enforcement() -> None:
    world = SimWorld()
    world.load_preset("inbound_3tier")

    # 2층 in5 아래 숨겨진 in6은 초기 스캔에서 보이지 않음
    vis1, _ = world.scan_visible()
    vis_ids1 = {p["track_id"] for p in vis1}
    assert "in5" in vis_ids1 and "in6" not in vis_ids1

    # 마우스 드래그로 in5를 옮기면 숨겨졌던 in6이 즉시 노출됨
    assert world.move_product_mm("in5", 450.0, 100.0, 45.0) is True
    vis2, _ = world.scan_visible()
    assert "in6" in {p["track_id"] for p in vis2}

    # 앞/뒷면·상태·레이어 토글 동작 확인
    assert world.toggle_backside("in1") is True
    assert world.products["in1"].is_backside is True
    assert world.cycle_status("in1") == "Expired"
    assert world.toggle_depth_layer("in1") == 2

    # Z=240mm > 200mm 직접 reorient는 로봇 규약대로 OUT_OF_REACH 차단
    ok, err, _ = world.apply_robot_action(
        RobotActionStep(
            action_type="reorient",
            track_id="in1",
            target_pose=[520.0, 0.0, 240.0, 0.0],
            product_category="snack_bag",
        )
    )
    assert ok is False and err == "OUT_OF_REACH"


def test_offline_benchmark_all_scenarios_pass() -> None:
    rows = run_offline_benchmark()
    assert len(rows) == 6
    for row in rows:
        assert row.passed is True, f"Benchmark failed: {row}"
        assert row.safety_violations == 0
