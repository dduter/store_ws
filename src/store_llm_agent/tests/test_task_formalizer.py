"""STOREPLAN TaskFormalizer 단위 테스트 — Pure 2차 LLM(1-B) 호출 및 Schema Sanitizer 환각 차단."""

from pathlib import Path
from types import SimpleNamespace

from store_llm_agent.core.action_domain import ActionDomain
from store_llm_agent.core.kg_manager import KGManager
from store_llm_agent.core.llm_parser import OrderConditions
from store_llm_agent.core.pipeline_engine import (
    load_buffer_slots,
    load_display_tiers,
    load_zone_bounds,
)
from store_llm_agent.core.task_formalizer import (
    Subgoal,
    TaskFormalizer,
    TaskSpec,
    compute_recommended_tier_mapping,
)

WS_ROOT = Path(__file__).resolve().parents[3]
TTL_PATH = WS_ROOT / "config" / "ontology" / "store_ontology.ttl"
CATALOG_PATH = WS_ROOT / "config" / "products_catalog.yaml"
ZONES_PATH = WS_ROOT / "config" / "workspace_zones.yaml"


def _make_mock_formalizer(spec: TaskSpec) -> TaskFormalizer:
    class _MockCompletions:
        def parse(self, **_kwargs):
            return SimpleNamespace(choices=[SimpleNamespace(message=SimpleNamespace(parsed=spec))])

    client = SimpleNamespace(beta=SimpleNamespace(chat=SimpleNamespace(completions=_MockCompletions())))
    return TaskFormalizer(client=client)


def test_task_formalizer_sanitizes_hallucinated_ids_and_duplicate_slots() -> None:
    kg = KGManager()
    kg.load_initial_graph(TTL_PATH, CATALOG_PATH)
    kg.sync_from_scan(
        [
            {
                "track_id": "t1", "product_name": "포스틱", "yolo_class": "postick_front",
                "is_backside": False, "pose_3d_base": [350.0, 260.0, 45.0, 0.0],
                "depth_layer": 1, "status": "Normal",
            },
            {
                "track_id": "t2", "product_name": "포스틱", "yolo_class": "postick_front",
                "is_backside": False, "pose_3d_base": [380.0, 280.0, 45.0, 0.0],
                "depth_layer": 1, "status": "Normal",
            },
        ],
        [{"container_type": "checkout", "valid": True, "drop_pose_base": [300.0, -250.0, 80.0], "confidence": 0.99}],
    )
    tiers = load_display_tiers(ZONES_PATH)
    domain = ActionDomain(
        display_tiers=tiers,
        buffer_slots=load_buffer_slots(ZONES_PATH),
        zone_bounds=load_zone_bounds(ZONES_PATH),
    )
    snap = kg.get_world_state_snapshot(display_tiers=tiers)
    state = domain.build_world_state(snap)

    # 2차 LLM이 없는 track_id와 중복 슬롯(tier_1_0 2회)을 반환하는 상황 시뮬레이션
    raw_spec = TaskSpec(
        intent="inbound",
        subgoals=[
            Subgoal(goal_type="on_tier_slot", track_id="t1", target="tier_1_0", reason="1단 배치"),
            Subgoal(goal_type="on_tier_slot", track_id="t2", target="tier_1_0", reason="중복 슬롯"),
            Subgoal(goal_type="in_container", track_id="ghost_id", target="checkout", reason="환각 ID"),
            Subgoal(goal_type="in_container", track_id="t1", target="bad_container", reason="잘못된 컨테이너"),
        ],
        reasoning="단맛 오름차순 1단 배치",
    )
    formalizer = _make_mock_formalizer(raw_spec)
    cleaned, warnings = formalizer.formalize(
        OrderConditions(intent="inbound", sort_by="sweetness", sort_order="asc"),
        state,
        snap["catalog"],
        domain,
    )

    assert len(cleaned.subgoals) == 2
    assert cleaned.subgoals[0].target == "tier_1_0"
    assert cleaned.subgoals[1].target == "tier_1_1"  # 같은 단 빈 슬롯으로 자동 보정
    assert len(warnings) == 3


def test_compute_recommended_tier_mapping_3_species_sweetness_and_category() -> None:
    catalog = [
        {"name": "포스틱", "robot_category": "snack_bag", "sweetness_rank": 1, "requires_remote_auth": False},
        {"name": "아몬드 빼빼로", "robot_category": "paper_box", "sweetness_rank": 3, "requires_remote_auth": False},
        {"name": "병음료", "robot_category": "bottle", "sweetness_rank": 2, "requires_remote_auth": False},
        {"name": "모형 담배", "robot_category": "ciga", "sweetness_rank": 0, "requires_remote_auth": True},
    ]
    # 1) 단맛 오름차순/내림차순: 3종 각 1단씩(단당 3개 진열) 매핑
    assert compute_recommended_tier_mapping(catalog, "sweetness", "asc") == {
        "tier_1": "포스틱",
        "tier_2": "병음료",
        "tier_3": "아몬드 빼빼로",
    }
    assert compute_recommended_tier_mapping(catalog, "sweetness", "desc") == {
        "tier_1": "아몬드 빼빼로",
        "tier_2": "병음료",
        "tier_3": "포스틱",
    }
    # 2) 카테고리 정렬: paper_box(0) -> snack_bag(1) -> bottle(2)
    assert compute_recommended_tier_mapping(catalog, "category", "asc") == {
        "tier_1": "아몬드 빼빼로",
        "tier_2": "포스틱",
        "tier_3": "병음료",
    }
