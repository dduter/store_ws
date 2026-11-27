"""STOREPLAN SymbolicPlanner 단위 테스트 — Subgoal별 BFS, 4-A 즉시 분할, Z>200mm 버퍼 하강 자동 탐색."""

from pathlib import Path

from store_llm_agent.core.action_domain import ActionDomain
from store_llm_agent.core.kg_manager import KGManager
from store_llm_agent.core.pipeline_engine import (
    load_buffer_slots,
    load_display_tiers,
    load_zone_bounds,
)
from store_llm_agent.core.symbolic_planner import SymbolicPlanner
from store_llm_agent.core.task_formalizer import Subgoal, TaskSpec

WS_ROOT = Path(__file__).resolve().parents[3]
TTL_PATH = WS_ROOT / "config" / "ontology" / "store_ontology.ttl"
CATALOG_PATH = WS_ROOT / "config" / "products_catalog.yaml"
ZONES_PATH = WS_ROOT / "config" / "workspace_zones.yaml"


def _build_domain_and_state(products: list[dict], containers: list[dict]):
    kg = KGManager()
    kg.load_initial_graph(TTL_PATH, CATALOG_PATH)
    kg.sync_from_scan(products, containers)
    tiers = load_display_tiers(ZONES_PATH)
    domain = ActionDomain(
        display_tiers=tiers,
        buffer_slots=load_buffer_slots(ZONES_PATH),
        zone_bounds=load_zone_bounds(ZONES_PATH),
    )
    snap = kg.get_world_state_snapshot(display_tiers=tiers)
    return domain, domain.build_world_state(snap)


def test_bfs_inbound_3tier_places_front_items_then_splits_on_reorient() -> None:
    containers = [
        {"container_type": "checkout", "valid": True, "drop_pose_base": [300.0, -250.0, 80.0], "confidence": 0.99},
        {"container_type": "disposal", "valid": True, "drop_pose_base": [300.0, 250.0, 80.0], "confidence": 0.98},
    ]
    domain, state = _build_domain_and_state(
        [
            {
                "track_id": "t1", "product_name": "포스틱", "yolo_class": "postick_front",
                "is_backside": False, "pose_3d_base": [320.0, 250.0, 45.0, 0.0],
                "depth_layer": 1, "status": "Normal",
            },
            {
                "track_id": "t2", "product_name": "병음료", "yolo_class": "drink_front",
                "is_backside": False, "pose_3d_base": [360.0, 250.0, 45.0, 0.0],
                "depth_layer": 1, "status": "Normal",
            },
            {
                "track_id": "t3", "product_name": "", "yolo_class": "product_back",
                "is_backside": True, "pose_3d_base": [400.0, 250.0, 45.0, 0.0],
                "depth_layer": 1, "status": "NeedsReorientation",
            },
        ],
        containers,
    )

    spec = TaskSpec(
        intent="inbound",
        subgoals=[
            Subgoal(goal_type="on_tier_slot", track_id="t1", target="tier_1_0", reason="1단 배치"),
            Subgoal(goal_type="on_tier_slot", track_id="t2", target="tier_2_0", reason="2단 배치"),
            Subgoal(goal_type="front_facing", track_id="t3", reason="뒷면 뒤집기"),
        ],
    )
    planner = SymbolicPlanner()
    plan = planner.plan(state, spec, domain)

    assert plan.safety_stop is False
    assert plan.needs_rescan is True  # 4-A: reorient 직후 즉시 분할
    assert [a.action_type for a in plan.actions] == ["pick_place", "pick_place", "reorient"]
    assert plan.actions[0].dest_pose == [300.0, -150.0, 40.0, 0.0]   # tier_1_0
    assert plan.actions[1].dest_pose == [420.0, -150.0, 140.0, 0.0]  # tier_2_0
    assert plan.actions[2].track_id == "t3"


def test_bfs_auto_discovers_buffer_lowering_for_tier3_backside_reorient() -> None:
    containers = [
        {"container_type": "disposal", "valid": True, "drop_pose_base": [300.0, 250.0, 80.0], "confidence": 0.98},
    ]
    domain, state = _build_domain_and_state(
        [
            {
                "track_id": "t_high", "product_name": "", "yolo_class": "product_back",
                "is_backside": True, "pose_3d_base": [520.0, 0.0, 240.0, 0.0],  # tier_3 Z=240mm > 200mm
                "depth_layer": 1, "status": "NeedsReorientation",
            },
        ],
        containers,
    )

    spec = TaskSpec(
        intent="inspect",
        subgoals=[Subgoal(goal_type="front_facing", track_id="t_high", reason="3단 뒷면 확인")],
    )
    planner = SymbolicPlanner()
    plan = planner.plan(state, spec, domain)

    assert plan.safety_stop is False
    assert plan.needs_rescan is True
    # BFS가 Precondition(Z<=200mm)을 보고 자동으로 move_to_buffer → reorient 2스텝 경로를 탐색
    assert [a.action_type for a in plan.actions] == ["move_to_buffer", "reorient"]
    assert plan.actions[0].dest_pose == [300.0, -280.0, 40.0, 0.0]  # buffer_0
    assert plan.actions[1].target_pose == [300.0, -280.0, 40.0, 0.0]
    assert plan.final_state is not None
    assert len(plan.final_state.buffer_stack) == 1
    assert plan.final_state.buffer_stack[0].original_location == "tier_3_1"


def test_bfs_safety_stop_when_container_invalid() -> None:
    domain, state = _build_domain_and_state(
        [
            {
                "track_id": "t1", "product_name": "아몬드 빼빼로", "yolo_class": "pepero_almond_front",
                "is_backside": False, "pose_3d_base": [350.0, 0.0, 45.0, 0.0],
                "depth_layer": 1, "status": "Normal",
            },
        ],
        [{"container_type": "checkout", "valid": False, "drop_pose_base": [0.0, 0.0, 0.0], "confidence": 0.1}],
    )
    spec = TaskSpec(
        intent="order",
        subgoals=[Subgoal(goal_type="in_container", track_id="t1", target="checkout")],
    )
    plan = SymbolicPlanner().plan(state, spec, domain)
    assert plan.safety_stop is True
    assert "SAFETY_STOP" in plan.stop_reason


def test_bfs_uses_fixed_flip_zone_with_zero_rz_when_configured() -> None:
    kg = KGManager()
    kg.load_initial_graph(TTL_PATH, CATALOG_PATH)
    kg.sync_from_scan(
        [
            {
                "track_id": "t_flip", "product_name": "아몬드 빼빼로", "yolo_class": "product_back",
                "is_backside": True, "pose_3d_base": [520.0, 0.0, 240.0, 25.0],
                "depth_layer": 1, "status": "NeedsReorientation",
            },
        ],
        [{"container_type": "checkout", "valid": True, "drop_pose_base": [300.0, -250.0, 80.0], "confidence": 0.99}],
    )
    tiers = load_display_tiers(ZONES_PATH)
    domain = ActionDomain(
        display_tiers=tiers,
        buffer_slots=load_buffer_slots(ZONES_PATH),
        zone_bounds=load_zone_bounds(ZONES_PATH),
        flip_zone=[340.0, -200.0, 40.0, 0.0],
    )
    state = domain.build_world_state(kg.get_world_state_snapshot(display_tiers=tiers))
    spec = TaskSpec(
        intent="inspect",
        subgoals=[Subgoal(goal_type="front_facing", track_id="t_flip", reason="Flip Zone 경유 뒤집기")],
    )
    plan = SymbolicPlanner().plan(state, spec, domain)

    assert plan.safety_stop is False
    assert plan.needs_rescan is True
    assert [a.action_type for a in plan.actions] == ["pick_place", "reorient"]
    assert plan.actions[0].dest_pose == [340.0, -200.0, 40.0, 0.0]
    assert plan.actions[0].product_category == "pepero_almond"
    assert plan.actions[1].target_pose == [340.0, -200.0, 40.0, 0.0]
    assert plan.actions[1].product_category == "pepero_almond"

