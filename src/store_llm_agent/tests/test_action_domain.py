"""STOREPLAN ActionDomain 단위 테스트 — 구역 분류, Precondition/Effect, Z<=200mm reorient 제약, Grounding."""

from pathlib import Path

from store_llm_agent.core.action_domain import (
    ActionDomain,
    BufferEntry,
    MAX_REORIENT_Z_MM,
    SymbolicAction,
)
from store_llm_agent.core.kg_manager import KGManager
from store_llm_agent.core.pipeline_engine import (
    load_buffer_slots,
    load_display_tiers,
    load_zone_bounds,
)

WS_ROOT = Path(__file__).resolve().parents[3]
TTL_PATH = WS_ROOT / "config" / "ontology" / "store_ontology.ttl"
CATALOG_PATH = WS_ROOT / "config" / "products_catalog.yaml"
ZONES_PATH = WS_ROOT / "config" / "workspace_zones.yaml"


def _make_domain() -> ActionDomain:
    return ActionDomain(
        display_tiers=load_display_tiers(ZONES_PATH),
        buffer_slots=load_buffer_slots(ZONES_PATH),
        zone_bounds=load_zone_bounds(ZONES_PATH),
    )


def _make_kg_snapshot(products: list[dict], containers: list[dict]) -> dict:
    kg = KGManager()
    kg.load_initial_graph(TTL_PATH, CATALOG_PATH)
    kg.sync_from_scan(products, containers)
    return kg.get_world_state_snapshot(display_tiers=load_display_tiers(ZONES_PATH))


def test_zone_classification_and_slot_occupancy() -> None:
    domain = _make_domain()
    snap = _make_kg_snapshot(
        [
            {
                "track_id": "t_in", "product_name": "아몬드 빼빼로", "yolo_class": "pepero_almond_front",
                "is_backside": False, "pose_3d_base": [350.0, 260.0, 45.0, 0.0],
                "depth_layer": 1, "status": "Normal",
            },
            {
                "track_id": "t_tier3", "product_name": "", "yolo_class": "product_back",
                "is_backside": True, "pose_3d_base": [520.0, 0.0, 240.0, 0.0],
                "depth_layer": 1, "status": "NeedsReorientation",
            },
            {
                "track_id": "t_buf", "product_name": "포스틱", "yolo_class": "postick_front",
                "is_backside": False, "pose_3d_base": [300.0, -280.0, 40.0, 0.0],
                "depth_layer": 1, "status": "Normal",
            },
        ],
        [{"container_type": "checkout", "valid": True, "drop_pose_base": [300.0, -250.0, 80.0], "confidence": 0.99}],
    )
    state = domain.build_world_state(snap)

    assert state.get_product("t_in").location == "inbound_box"
    assert state.get_product("t_tier3").location == "tier_3_1"
    assert state.get_product("t_buf").location == "buffer_0"
    assert "tier_3_1" in state.occupied_slots
    assert "buffer_0" in state.occupied_slots


def test_reorient_blocked_above_200mm_and_allowed_after_buffer_lowering() -> None:
    domain = _make_domain()
    snap = _make_kg_snapshot(
        [
            {
                "track_id": "t_high", "product_name": "", "yolo_class": "product_back",
                "is_backside": True, "pose_3d_base": [520.0, 0.0, 240.0, 0.0],
                "depth_layer": 1, "status": "NeedsReorientation",
            }
        ],
        [{"container_type": "disposal", "valid": True, "drop_pose_base": [300.0, 250.0, 80.0], "confidence": 0.99}],
    )
    state = domain.build_world_state(snap)

    # 1) Z=240mm > 200mm 에서는 직접 reorient 불가
    ok_direct, reason = domain.check_precondition(state, SymbolicAction(action_type="reorient", track_id="t_high"))
    assert ok_direct is False
    assert f"{MAX_REORIENT_Z_MM:.0f}mm" in reason

    # 2) 낮은 버퍼(buffer_0, Z=40mm)로 하강 후에는 reorient 가능
    lower_act = SymbolicAction(
        action_type="move_to_buffer",
        track_id="t_high",
        dest_id="buffer_0",
        purpose="lower_for_reorient",
        needs_rescan=False,
    )
    step1, err1 = domain.ground_to_robot_step(state, lower_act)
    assert step1 is not None and err1 == "OK"
    assert step1.dest_pose[2] == 40.0

    state2 = domain.apply(state, lower_act)
    assert state2.get_product("t_high").location == "buffer_0"
    assert state2.get_product("t_high").pose[2] == 40.0
    assert len(state2.buffer_stack) == 1
    assert state2.buffer_stack[0] == BufferEntry(
        track_id="t_high",
        buffer_slot="buffer_0",
        original_location="tier_3_1",
        original_pose=(520.0, 0.0, 240.0, 0.0),
        purpose="lower_for_reorient",
    )

    ok_after, _ = domain.check_precondition(state2, SymbolicAction(action_type="reorient", track_id="t_high"))
    assert ok_after is True

    # 3) reorient 후 원위치(tier_3_1) LIFO 복원
    state3 = domain.apply(state2, SymbolicAction(action_type="reorient", track_id="t_high", needs_rescan=True))
    restore_act = SymbolicAction(action_type="restore_from_buffer", track_id="t_high")
    step3, err3 = domain.ground_to_robot_step(state3, restore_act)
    assert step3 is not None and err3 == "OK"
    assert step3.dest_pose == [520.0, 0.0, 240.0, 0.0]

    state4 = domain.apply(state3, restore_act)
    assert state4.get_product("t_high").location == "tier_3_1"
    assert len(state4.buffer_stack) == 0
