"""STOREPLAN E2E 시나리오 테스트 (확정 3종 × 각 3개 실물 = 총 9개 상품 + 모형 담배 기준)."""

from pathlib import Path
from types import SimpleNamespace

from store_llm_agent.core.action_domain import ActionDomain
from store_llm_agent.core.kg_manager import KGManager
from store_llm_agent.core.llm_parser import ItemCondition, LLMParser, OrderConditions
from store_llm_agent.core.mode_fsm import ModeFSM
from store_llm_agent.core.pipeline_engine import (
    StorePipelineEngine,
    load_buffer_slots,
    load_display_tiers,
    load_zone_bounds,
)
from store_llm_agent.core.symbolic_planner import SymbolicPlanner
from store_llm_agent.core.task_formalizer import Subgoal, TaskFormalizer, TaskSpec

WS_ROOT = Path(__file__).resolve().parents[3]
TTL_PATH = WS_ROOT / "config" / "ontology" / "store_ontology.ttl"
CATALOG_PATH = WS_ROOT / "config" / "products_catalog.yaml"
ZONES_PATH = WS_ROOT / "config" / "workspace_zones.yaml"


def _make_mock_parser(parsed_obj: OrderConditions) -> LLMParser:
    class _MockCompletions:
        def parse(self, **_kwargs):
            return SimpleNamespace(choices=[SimpleNamespace(message=SimpleNamespace(parsed=parsed_obj))])

    client = SimpleNamespace(beta=SimpleNamespace(chat=SimpleNamespace(completions=_MockCompletions())))
    return LLMParser(client=client)


def _make_mock_formalizer(spec_sequence: list[TaskSpec]) -> TaskFormalizer:
    idx = {"i": 0}

    class _MockCompletions:
        def parse(self, **_kwargs):
            cur = min(idx["i"], len(spec_sequence) - 1)
            idx["i"] += 1
            return SimpleNamespace(choices=[SimpleNamespace(message=SimpleNamespace(parsed=spec_sequence[cur]))])

    client = SimpleNamespace(beta=SimpleNamespace(chat=SimpleNamespace(completions=_MockCompletions())))
    return TaskFormalizer(client=client)


def _containers_ok() -> list[dict]:
    return [
        {"container_type": "checkout", "valid": True, "drop_pose_base": [300.0, -250.0, 80.0], "confidence": 0.99},
        {"container_type": "disposal", "valid": True, "drop_pose_base": [300.0, 250.0, 80.0], "confidence": 0.98},
    ]


def _build_engine(
    parsed_obj: OrderConditions,
    spec_sequence: list[TaskSpec],
    scan_sequence: list[tuple[list[dict], list[dict]]],
    mode_employee: bool = False,
    execute_fn=None,
) -> StorePipelineEngine:
    kg = KGManager()
    kg.load_initial_graph(TTL_PATH, CATALOG_PATH)
    fsm = ModeFSM(required_vest_frames=1)
    if mode_employee:
        fsm.update_vest_detection(True, 0.95, now=1.0)

    idx = {"i": 0}

    def scan_fn(_zone: str, _run_vlm: bool):
        cur = min(idx["i"], len(scan_sequence) - 1)
        idx["i"] += 1
        prods, conts = scan_sequence[cur]
        return True, "OK", prods, conts

    display_tiers = load_display_tiers(ZONES_PATH)
    domain = ActionDomain(
        display_tiers=display_tiers,
        buffer_slots=load_buffer_slots(ZONES_PATH),
        zone_bounds=load_zone_bounds(ZONES_PATH),
    )

    return StorePipelineEngine(
        kg_manager=kg,
        mode_fsm=fsm,
        llm_parser=_make_mock_parser(parsed_obj),
        task_formalizer=_make_mock_formalizer(spec_sequence),
        action_domain=domain,
        symbolic_planner=SymbolicPlanner(),
        scan_fn=scan_fn,
        execute_action_fn=execute_fn or (lambda _step: (True, "OK")),
        display_tiers=display_tiers,
    )


def test_e2e_1_normal_order() -> None:
    parsed = OrderConditions(intent="order", items=[ItemCondition(name="포스틱", quantity=1)])
    specs = [TaskSpec(intent="order", subgoals=[Subgoal(goal_type="in_container", track_id="t1", target="checkout")])]
    scan = [(
        [{
            "track_id": "t1", "product_name": "포스틱", "yolo_class": "postick_front",
            "is_backside": False, "pose_3d_base": [400.0, 100.0, 50.0, 0.0],
            "depth_layer": 1, "status": "Normal",
        }],
        _containers_ok(),
    )]
    engine = _build_engine(parsed, specs, scan)
    trace = engine.handle_command("포스틱 하나 줘", source="voice")

    assert trace.success is True
    assert [a.action_type for a in trace.executed_actions] == ["scan_pose", "pick_place", "home"]
    assert trace.executed_actions[1].product_category == "postick"
    snap = engine.build_ui_snapshot()
    assert set(snap.keys()) == {"mode", "inventory", "disposal_history", "kg_graph", "agent_log"}


def test_e2e_2_expired_order_with_alternative() -> None:
    # 동일 품목 3개 진열 환경에서 만료된 포스틱(t1)은 폐기통으로, 정상 포스틱(t2)은 결제통으로 이동
    parsed = OrderConditions(
        intent="order",
        items=[ItemCondition(name="포스틱", category="봉지과자", quantity=1)],
    )
    specs = [
        TaskSpec(
            intent="order",
            subgoals=[
                Subgoal(goal_type="in_container", track_id="t1", target="disposal", reason="만료 폐기"),
                Subgoal(goal_type="in_container", track_id="t2", target="checkout", reason="정상 재고 전달"),
            ],
        )
    ]
    scan = [(
        [
            {
                "track_id": "t1", "product_name": "포스틱", "yolo_class": "postick_front",
                "is_backside": False, "pose_3d_base": [400.0, 100.0, 50.0, 0.0],
                "depth_layer": 1, "status": "Expired", "disposal_reason": "유통기한 만료",
            },
            {
                "track_id": "t2", "product_name": "포스틱", "yolo_class": "postick_front",
                "is_backside": False, "pose_3d_base": [450.0, 100.0, 50.0, 0.0],
                "depth_layer": 1, "status": "Normal",
            },
        ],
        _containers_ok(),
    )]
    engine = _build_engine(parsed, specs, scan)
    trace = engine.handle_command("포스틱 줘")

    assert trace.success is True
    assert [a.action_type for a in trace.executed_actions] == [
        "scan_pose", "pick_place", "pick_place", "home"
    ]
    assert trace.executed_actions[1].dest_pose[:3] == [300.0, 250.0, 80.0]   # 폐기통
    assert trace.executed_actions[2].dest_pose[:3] == [300.0, -250.0, 80.0]  # 결제통
    assert len(engine.build_ui_snapshot()["disposal_history"]) == 1


def test_e2e_3_backside_reorient_then_rescan() -> None:
    parsed = OrderConditions(intent="order", items=[ItemCondition(name="아몬드 빼빼로", quantity=1)])
    specs = [
        TaskSpec(intent="order", subgoals=[Subgoal(goal_type="front_facing", track_id="t3")]),
        TaskSpec(intent="order", subgoals=[Subgoal(goal_type="in_container", track_id="t3", target="checkout")]),
    ]
    scan1 = (
        [{
            "track_id": "t3", "product_name": "", "yolo_class": "product_back",
            "is_backside": True, "pose_3d_base": [400.0, 0.0, 50.0, 0.0],
            "depth_layer": 1, "status": "NeedsReorientation",
        }],
        _containers_ok(),
    )
    scan2 = (
        [{
            "track_id": "t3", "product_name": "아몬드 빼빼로", "yolo_class": "pepero_almond_front",
            "is_backside": False, "pose_3d_base": [400.0, 0.0, 50.0, 0.0],
            "depth_layer": 1, "status": "Normal",
        }],
        _containers_ok(),
    )
    engine = _build_engine(parsed, specs, [scan1, scan2])
    trace = engine.handle_command("빼빼로 줘")

    assert trace.success is True
    assert [a.action_type for a in trace.executed_actions] == [
        "scan_pose", "reorient", "scan_pose", "pick_place", "home"
    ]


def test_e2e_4_customer_blocked_and_employee_dispose() -> None:
    parsed_inspect = OrderConditions(intent="inspect")
    engine_cust = _build_engine(
        parsed_inspect, [TaskSpec(intent="inspect")], [([], _containers_ok())], mode_employee=False
    )
    blocked = engine_cust.handle_command("매대 검수해줘")
    assert blocked.allowed is False
    assert blocked.executed_actions == []

    parsed_dispose = OrderConditions(intent="dispose")
    specs_dispose = [
        TaskSpec(intent="dispose", subgoals=[Subgoal(goal_type="in_container", track_id="t_exp", target="disposal")])
    ]
    scan_expired = [(
        [{
            "track_id": "t_exp", "product_name": "병음료", "yolo_class": "drink_front",
            "is_backside": False, "pose_3d_base": [400.0, 100.0, 50.0, 0.0],
            "depth_layer": 1, "status": "Expired", "disposal_reason": "유통기한 만료",
        }],
        _containers_ok(),
    )]
    engine_emp = _build_engine(parsed_dispose, specs_dispose, scan_expired, mode_employee=True)
    res = engine_emp.handle_command("만료된 거 폐기통에 버려줘")
    assert res.success is True
    assert [a.action_type for a in res.executed_actions] == ["scan_pose", "pick_place", "home"]
    assert res.executed_actions[1].dest_pose[:3] == [300.0, 250.0, 80.0]


def test_e2e_5_inbound_2layer_move_and_lifo_restore() -> None:
    parsed = OrderConditions(intent="inbound", items=[ItemCondition(name="아몬드 빼빼로", quantity=1)])
    specs = [
        TaskSpec(intent="inbound", subgoals=[Subgoal(goal_type="unblock_layer", track_id="b1")]),
        TaskSpec(
            intent="inbound",
            subgoals=[
                Subgoal(goal_type="on_tier_slot", track_id="target_a", target="tier_1_0"),
                Subgoal(goal_type="buffer_empty"),
            ],
        ),
    ]
    scan1 = (
        [{
            "track_id": "b1", "product_name": "병음료", "yolo_class": "drink_front",
            "is_backside": False, "pose_3d_base": [350.0, 260.0, 90.0, 0.0],
            "depth_layer": 2, "status": "Normal",
        }],
        _containers_ok(),
    )
    scan2 = (
        [
            {
                "track_id": "b1", "product_name": "병음료", "yolo_class": "drink_front",
                "is_backside": False, "pose_3d_base": [300.0, -280.0, 40.0, 0.0],
                "depth_layer": 1, "status": "Normal",
            },
            {
                "track_id": "target_a", "product_name": "아몬드 빼빼로", "yolo_class": "pepero_almond_front",
                "is_backside": False, "pose_3d_base": [350.0, 260.0, 45.0, 0.0],
                "depth_layer": 1, "status": "Normal",
            },
        ],
        _containers_ok(),
    )
    engine = _build_engine(parsed, specs, [scan1, scan2], mode_employee=True)
    trace = engine.handle_command("입고 상자에서 빼빼로 꺼내서 1단에 놓아줘")

    assert trace.success is True
    assert [a.action_type for a in trace.executed_actions] == [
        "scan_pose", "move_to_buffer", "scan_pose", "pick_place", "restore_from_buffer", "home"
    ]


def test_e2e_6_safety_stop_on_invalid_container() -> None:
    parsed = OrderConditions(intent="order", items=[ItemCondition(name="포스틱", quantity=1)])
    specs = [TaskSpec(intent="order", subgoals=[Subgoal(goal_type="in_container", track_id="t1", target="checkout")])]
    bad_cont = [{"container_type": "checkout", "valid": False, "drop_pose_base": [0.0, 0.0, 0.0], "confidence": 0.1}]
    scan = [(
        [{
            "track_id": "t1", "product_name": "포스틱", "yolo_class": "postick_front",
            "is_backside": False, "pose_3d_base": [400.0, 100.0, 50.0, 0.0],
            "depth_layer": 1, "status": "Normal",
        }],
        bad_cont,
    )]
    engine = _build_engine(parsed, specs, scan)
    trace = engine.handle_command("포스틱 줘")

    assert trace.safety_stop is True
    assert trace.success is False
    assert [a.action_type for a in trace.executed_actions] == ["scan_pose"]


def test_e2e_7_multi_qty_same_species_order_with_reorient_and_rescan() -> None:
    # 동일 품목(포스틱 2개) 복수 주문 + 1개 뒷면 뒤집기 검증
    parsed = OrderConditions(intent="order", items=[ItemCondition(name="포스틱", quantity=2)])
    specs = [
        TaskSpec(
            intent="order",
            subgoals=[
                Subgoal(goal_type="in_container", track_id="t1", target="checkout"),
                Subgoal(goal_type="front_facing", track_id="t2"),
            ],
        ),
        TaskSpec(
            intent="order",
            subgoals=[Subgoal(goal_type="in_container", track_id="t2", target="checkout")],
        ),
    ]
    scan1 = (
        [
            {
                "track_id": "t1", "product_name": "포스틱", "yolo_class": "postick_front",
                "is_backside": False, "pose_3d_base": [400.0, 100.0, 50.0, 0.0],
                "depth_layer": 1, "status": "Normal",
            },
            {
                "track_id": "t2", "product_name": "", "yolo_class": "product_back",
                "is_backside": True, "pose_3d_base": [400.0, 0.0, 50.0, 0.0],
                "depth_layer": 1, "status": "NeedsReorientation",
            },
        ],
        _containers_ok(),
    )
    scan2 = (
        [
            {
                "track_id": "t2", "product_name": "포스틱", "yolo_class": "postick_front",
                "is_backside": False, "pose_3d_base": [400.0, 0.0, 50.0, 0.0],
                "depth_layer": 1, "status": "Normal",
            },
        ],
        _containers_ok(),
    )
    engine = _build_engine(parsed, specs, [scan1, scan2])
    trace = engine.handle_command("포스틱 두 개 줘")

    assert trace.success is True
    assert [a.action_type for a in trace.executed_actions] == [
        "scan_pose", "pick_place", "reorient", "scan_pose", "pick_place", "home"
    ]
    assert trace.executed_actions[1].track_id == "t1"
    assert trace.executed_actions[2].track_id == "t2"
    assert trace.executed_actions[4].track_id == "t2"


def test_e2e_8_out_of_stock_returns_success_false() -> None:
    parsed = OrderConditions(intent="order", items=[ItemCondition(name="병음료", quantity=1)])
    specs = [TaskSpec(intent="order", subgoals=[], unfulfillable_reason="병음료 품절")]
    scan = [([], _containers_ok())]
    engine = _build_engine(parsed, specs, scan)
    trace = engine.handle_command("병음료 하나 줘")

    assert trace.success is False
    assert trace.safety_stop is False
    assert [a.action_type for a in trace.executed_actions] == ["scan_pose", "home"]


def test_e2e_9_remote_auth_product_and_remote_intent_blocked() -> None:
    parsed_tobacco = OrderConditions(intent="order", items=[ItemCondition(name="모형 담배", quantity=1)])
    engine = _build_engine(parsed_tobacco, [TaskSpec(intent="order")], [([], _containers_ok())])
    res_tobacco = engine.handle_command("모형 담배 하나 줘")
    assert res_tobacco.allowed is False
    assert res_tobacco.success is False
    assert res_tobacco.executed_actions == []
    assert "원격 관리자 인증" in res_tobacco.message

    parsed_remote = OrderConditions(intent="remote")
    engine_remote = _build_engine(parsed_remote, [TaskSpec(intent="remote")], [([], _containers_ok())])
    res_remote = engine_remote.handle_command("원격 제어 켜줘")
    assert res_remote.allowed is False
    assert res_remote.success is False
    assert res_remote.executed_actions == []
    assert "verify_remote_key" in res_remote.message


def test_e2e_10_safety_stop_on_j6_limit_violation() -> None:
    parsed = OrderConditions(intent="order", items=[ItemCondition(name="아몬드 빼빼로", quantity=1)])
    specs = [TaskSpec(intent="order", subgoals=[Subgoal(goal_type="in_container", track_id="t1", target="checkout")])]
    scan = [(
        [{
            "track_id": "t1", "product_name": "아몬드 빼빼로", "yolo_class": "pepero_almond_front",
            "is_backside": False, "pose_3d_base": [400.0, 100.0, 50.0, 0.0],
            "depth_layer": 1, "status": "Normal",
        }],
        _containers_ok(),
    )]

    def exec_fail_j6(step):
        if step.action_type == "pick_place":
            return False, "J6_LIMIT_VIOLATION"
        return True, "OK"

    engine = _build_engine(parsed, specs, scan, execute_fn=exec_fail_j6)
    trace = engine.handle_command("빼빼로 줘")
    assert trace.success is False
    assert trace.safety_stop is True


def test_e2e_11_grasp_fail_retries_once_after_rescan_and_out_of_reach_stops() -> None:
    parsed = OrderConditions(intent="order", items=[ItemCondition(name="병음료", quantity=1)])
    specs = [
        TaskSpec(intent="order", subgoals=[Subgoal(goal_type="in_container", track_id="t1", target="checkout")]),
        TaskSpec(intent="order", subgoals=[Subgoal(goal_type="in_container", track_id="t1", target="checkout")]),
    ]
    scan = [(
        [{
            "track_id": "t1", "product_name": "병음료", "yolo_class": "drink_front",
            "is_backside": False, "pose_3d_base": [400.0, 100.0, 50.0, 0.0],
            "depth_layer": 1, "status": "Normal",
        }],
        _containers_ok(),
    )]

    attempts = {"pick": 0}

    def exec_grasp_once(step):
        if step.action_type == "pick_place":
            attempts["pick"] += 1
            if attempts["pick"] == 1:
                return False, "GRASP_FAIL"
        return True, "OK"

    engine_retry = _build_engine(parsed, specs, scan, execute_fn=exec_grasp_once)
    res_retry = engine_retry.handle_command("병음료 줘")
    assert res_retry.success is True
    assert [a.action_type for a in res_retry.executed_actions] == [
        "scan_pose", "pick_place", "scan_pose", "pick_place", "home"
    ]

    engine_fail2 = _build_engine(
        parsed, specs, scan,
        execute_fn=lambda step: (False, "GRASP_FAIL") if step.action_type == "pick_place" else (True, "OK"),
    )
    res_fail2 = engine_fail2.handle_command("병음료 줘")
    assert res_fail2.success is False
    assert [a.action_type for a in res_fail2.executed_actions] == [
        "scan_pose", "pick_place", "scan_pose", "pick_place"
    ]

    engine_oor = _build_engine(
        parsed, specs, scan,
        execute_fn=lambda step: (False, "OUT_OF_REACH") if step.action_type == "pick_place" else (True, "OK"),
    )
    res_oor = engine_oor.handle_command("병음료 줘")
    assert res_oor.success is False
    assert [a.action_type for a in res_oor.executed_actions] == ["scan_pose", "pick_place"]


def test_e2e_12_inbound_3tier_sorting_with_backside_rescan() -> None:
    parsed = OrderConditions(intent="inbound", sort_by="category", sort_order="asc")
    specs = [
        TaskSpec(
            intent="inbound",
            subgoals=[
                Subgoal(goal_type="on_tier_slot", track_id="in1", target="tier_1_0"),
                Subgoal(goal_type="on_tier_slot", track_id="in4", target="tier_2_0"),
                Subgoal(goal_type="front_facing", track_id="in7"),
            ],
            reasoning="카테고리별 정렬: 1단 아몬드 빼빼로(박스), 2단 포스틱(봉지), 3단 병음료",
        ),
        TaskSpec(
            intent="inbound",
            subgoals=[Subgoal(goal_type="on_tier_slot", track_id="in7", target="tier_3_0")],
            reasoning="뒷면 판별 완료(병음료) → 3단 배치",
        ),
    ]
    scan1 = (
        [
            {
                "track_id": "in1", "product_name": "아몬드 빼빼로", "yolo_class": "pepero_almond_front",
                "is_backside": False, "pose_3d_base": [310.0, 240.0, 45.0, 0.0], "depth_layer": 1, "status": "Normal",
            },
            {
                "track_id": "in4", "product_name": "포스틱", "yolo_class": "postick_front",
                "is_backside": False, "pose_3d_base": [360.0, 240.0, 45.0, 0.0], "depth_layer": 1, "status": "Normal",
            },
            {
                "track_id": "in7", "product_name": "", "yolo_class": "product_back",
                "is_backside": True, "pose_3d_base": [410.0, 240.0, 45.0, 0.0], "depth_layer": 1,
                "status": "NeedsReorientation",
            },
        ],
        _containers_ok(),
    )
    scan2 = (
        [
            {
                "track_id": "in1", "product_name": "아몬드 빼빼로", "yolo_class": "pepero_almond_front",
                "is_backside": False, "pose_3d_base": [300.0, -150.0, 40.0, 0.0], "depth_layer": 1, "status": "Normal",
            },
            {
                "track_id": "in4", "product_name": "포스틱", "yolo_class": "postick_front",
                "is_backside": False, "pose_3d_base": [420.0, -150.0, 140.0, 0.0], "depth_layer": 1, "status": "Normal",
            },
            {
                "track_id": "in7", "product_name": "병음료", "yolo_class": "drink_front",
                "is_backside": False, "pose_3d_base": [410.0, 240.0, 45.0, 0.0], "depth_layer": 1, "status": "Normal",
            },
        ],
        _containers_ok(),
    )

    engine = _build_engine(parsed, specs, [scan1, scan2], mode_employee=True)
    trace = engine.handle_command("카테고리별로 3단 진열대 정리해줘")

    assert trace.success is True
    assert [a.action_type for a in trace.executed_actions] == [
        "scan_pose", "pick_place", "pick_place", "reorient", "scan_pose", "pick_place", "home"
    ]
    assert trace.executed_actions[5].dest_pose == [520.0, -150.0, 240.0, 0.0]  # tier_3_0


def test_e2e_13_tier3_high_backside_reorient_via_buffer_and_restore() -> None:
    parsed = OrderConditions(intent="inspect")
    specs = [
        TaskSpec(intent="inspect", subgoals=[Subgoal(goal_type="front_facing", track_id="t_high")]),
        TaskSpec(intent="inspect", subgoals=[Subgoal(goal_type="buffer_empty")]),
    ]
    scan1 = (
        [{
            "track_id": "t_high", "product_name": "", "yolo_class": "product_back",
            "is_backside": True, "pose_3d_base": [520.0, 0.0, 240.0, 0.0],  # tier_3_1 (Z=240 > 200)
            "depth_layer": 1, "status": "NeedsReorientation",
        }],
        _containers_ok(),
    )
    scan2 = (
        [{
            "track_id": "t_high", "product_name": "병음료", "yolo_class": "drink_front",
            "is_backside": False, "pose_3d_base": [300.0, -280.0, 40.0, 0.0],  # buffer_0
            "depth_layer": 1, "status": "Normal",
        }],
        _containers_ok(),
    )

    engine = _build_engine(parsed, specs, [scan1, scan2], mode_employee=True)
    trace = engine.handle_command("3단 진열대 검수해줘")

    assert trace.success is True
    assert [a.action_type for a in trace.executed_actions] == [
        "scan_pose", "move_to_buffer", "reorient", "scan_pose", "restore_from_buffer", "home"
    ]
    assert trace.executed_actions[4].dest_pose == [520.0, 0.0, 240.0, 0.0]  # 원래 tier_3_1 복원


def test_paths_and_vest_payload_helpers() -> None:
    import pytest
    from store_llm_agent.core.paths import find_workspace_root

    root = find_workspace_root()
    assert (root / "config" / "workspace_zones.yaml").is_file()

    pytest.importorskip("rclpy")
    from store_llm_agent.nodes.orchestrator import parse_vest_payload

    assert parse_vest_payload('{"detected": true, "confidence": 0.91}') == (True, 0.91)
    assert parse_vest_payload('{"detected": true, "conf": 0.88}') == (True, 0.88)


def test_e2e_14_clarification_for_ambiguous_noise_and_followup() -> None:
    # 1) 이해 불가 발화 -> 0스텝 역질문
    parsed_noise = OrderConditions(
        intent="clarify",
        clarification_reason="매장 카탈로그와 무관한 발화",
        clarification_question="말씀하신 내용을 이해하지 못했습니다. 다시 말씀해 주세요.",
    )
    eng_noise = _build_engine(parsed_noise, [TaskSpec(intent="order")], [([], _containers_ok())])
    tr_noise = eng_noise.handle_command("기가 막힐 toys", source="voice")
    assert tr_noise.needs_clarification is True
    assert tr_noise.executed_actions == []
    assert tr_noise.inferred_candidates == []

    # 2) 단일 상품(quantity=1) 상위 카테고리('과자') 다중 매칭 -> 온톨로지 확인질문 후 다음 턴 응답 연동
    calls: list[dict] = []
    responses = [
        OrderConditions(intent="order", items=[ItemCondition(category="과자", quantity=1)]),
        OrderConditions(intent="order", items=[ItemCondition(name="아몬드 빼빼로", quantity=1)]),
    ]

    class _SeqCompletions:
        def parse(self, **kwargs):
            calls.append(kwargs)
            parsed = responses[min(len(calls) - 1, len(responses) - 1)]
            return SimpleNamespace(choices=[SimpleNamespace(message=SimpleNamespace(parsed=parsed))])

    kg = KGManager()
    kg.load_initial_graph(TTL_PATH, CATALOG_PATH)
    tiers = load_display_tiers(ZONES_PATH)
    domain = ActionDomain(
        display_tiers=tiers,
        buffer_slots=load_buffer_slots(ZONES_PATH),
        zone_bounds=load_zone_bounds(ZONES_PATH),
    )
    eng = StorePipelineEngine(
        kg_manager=kg,
        mode_fsm=ModeFSM(),
        llm_parser=LLMParser(client=SimpleNamespace(beta=SimpleNamespace(chat=SimpleNamespace(completions=_SeqCompletions())))),
        task_formalizer=_make_mock_formalizer(
            [TaskSpec(intent="order", subgoals=[Subgoal(goal_type="in_container", track_id="t1", target="checkout")])]
        ),
        action_domain=domain,
        symbolic_planner=SymbolicPlanner(),
        scan_fn=lambda _z, _r: (
            True,
            "OK",
            [{
                "track_id": "t1", "product_name": "아몬드 빼빼로", "yolo_class": "pepero_almond_front",
                "is_backside": False, "pose_3d_base": [400.0, 100.0, 50.0, 0.0],
                "depth_layer": 1, "status": "Normal",
            }],
            _containers_ok(),
        ),
        execute_action_fn=lambda _s: (True, "OK"),
        display_tiers=tiers,
    )

    tr1 = eng.handle_command("과자 하나 줘", source="voice")
    assert tr1.needs_clarification is True
    assert tr1.executed_actions == []
    assert set(tr1.inferred_candidates) == {"포스틱", "아몬드 빼빼로"}

    tr2 = eng.handle_command("아몬드 빼빼로로 줘", source="voice")
    assert tr2.success is True
    assert tr2.needs_clarification is False
    assert [a.action_type for a in tr2.executed_actions] == ["scan_pose", "pick_place", "home"]
    assert "previous_clarification" in calls[1]["messages"][0]["content"]
