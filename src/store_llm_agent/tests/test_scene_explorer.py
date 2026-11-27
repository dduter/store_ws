"""OntoPlan SceneExplorer 단위 테스트 — Ask(SPARQL/OWL) Tool Calling 및 TaskFormalizer 전달."""

import json
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
from store_llm_agent.core.scene_explorer import SceneExplorer
from store_llm_agent.core.symbolic_planner import SymbolicPlanner
from store_llm_agent.core.task_formalizer import Subgoal, TaskFormalizer, TaskSpec

WS_ROOT = Path(__file__).resolve().parents[3]
TTL_PATH = WS_ROOT / "config" / "ontology" / "store_ontology.ttl"
CATALOG_PATH = WS_ROOT / "config" / "products_catalog.yaml"
ZONES_PATH = WS_ROOT / "config" / "workspace_zones.yaml"


def test_scene_explorer_executes_ask_tools_and_feeds_formalizer() -> None:
    kg = KGManager()
    kg.load_initial_graph(TTL_PATH, CATALOG_PATH, zones_yaml=ZONES_PATH)
    kg.sync_from_scan(
        [
            {
                "track_id": "t_soon", "product_name": "포스틱", "yolo_class": "postick_front",
                "is_backside": False, "pose_3d_base": [300.0, 0.0, 40.0, 0.0],
                "depth_layer": 1, "expiration_raw": "2026.12.01", "status": "Normal",
            },
            {
                "track_id": "t_exp", "product_name": "아몬드 빼빼로", "yolo_class": "pepero_almond_front",
                "is_backside": False, "pose_3d_base": [420.0, 0.0, 140.0, 0.0],
                "depth_layer": 1, "expiration_raw": "2025.01.01", "status": "Expired",
            },
        ],
        [
            {"container_type": "checkout", "valid": True, "drop_pose_base": [260.0, -240.0, 70.0], "confidence": 0.99},
            {"container_type": "disposal", "valid": True, "drop_pose_base": [260.0, 290.0, 70.0], "confidence": 0.98},
        ],
    )

    turn = {"n": 0}

    def _create(**_kwargs):
        turn["n"] += 1
        if turn["n"] == 1:
            tcs = [
                SimpleNamespace(
                    id="call_1",
                    function=SimpleNamespace(
                        name="ask_sellable_items",
                        arguments=json.dumps({"name": "포스틱"}),
                    ),
                ),
                SimpleNamespace(
                    id="call_2",
                    function=SimpleNamespace(
                        name="ask_ontology_relations",
                        arguments=json.dumps({"relation_type": "disposal_candidates"}),
                    ),
                ),
                SimpleNamespace(
                    id="call_3",
                    function=SimpleNamespace(name="ask_containers", arguments="{}"),
                ),
            ]
            return SimpleNamespace(choices=[SimpleNamespace(message=SimpleNamespace(content=None, tool_calls=tcs))])
        return SimpleNamespace(
            choices=[SimpleNamespace(message=SimpleNamespace(content="포스틱 t_soon 판매 가능, t_exp 폐기 대상 확인", tool_calls=[]))]
        )

    mock_exp_client = SimpleNamespace(chat=SimpleNamespace(completions=SimpleNamespace(create=_create)))
    explorer = SceneExplorer(client=mock_exp_client)

    captured_payload: dict = {}

    class _MockFormalizerCompletions:
        def parse(self, **kwargs):
            captured_payload["user"] = json.loads(kwargs["messages"][1]["content"])
            spec = TaskSpec(
                intent="order",
                subgoals=[Subgoal(goal_type="in_container", track_id="t_soon", target="checkout")],
            )
            return SimpleNamespace(choices=[SimpleNamespace(message=SimpleNamespace(parsed=spec))])

    formalizer = TaskFormalizer(
        client=SimpleNamespace(beta=SimpleNamespace(chat=SimpleNamespace(completions=_MockFormalizerCompletions())))
    )
    parser = LLMParser(
        client=SimpleNamespace(
            beta=SimpleNamespace(
                chat=SimpleNamespace(
                    completions=SimpleNamespace(
                        parse=lambda **_: SimpleNamespace(
                            choices=[
                                SimpleNamespace(
                                    message=SimpleNamespace(
                                        parsed=OrderConditions(
                                            intent="order",
                                            items=[ItemCondition(name="포스틱", quantity=1)],
                                        )
                                    )
                                )
                            ]
                        )
                    )
                )
            )
        )
    )
    tiers = load_display_tiers(ZONES_PATH)
    domain = ActionDomain(
        display_tiers=tiers,
        buffer_slots=load_buffer_slots(ZONES_PATH),
        zone_bounds=load_zone_bounds(ZONES_PATH),
    )
    engine = StorePipelineEngine(
        kg_manager=kg,
        mode_fsm=ModeFSM(),
        llm_parser=parser,
        task_formalizer=formalizer,
        action_domain=domain,
        symbolic_planner=SymbolicPlanner(),
        scan_fn=lambda _z, _v: (
            True,
            "OK",
            [
                {
                    "track_id": "t_soon", "product_name": "포스틱", "yolo_class": "postick_front",
                    "is_backside": False, "pose_3d_base": [300.0, 0.0, 40.0, 0.0],
                    "depth_layer": 1, "expiration_raw": "2026.12.01", "status": "Normal",
                },
                {
                    "track_id": "t_exp", "product_name": "아몬드 빼빼로", "yolo_class": "pepero_almond_front",
                    "is_backside": False, "pose_3d_base": [420.0, 0.0, 140.0, 0.0],
                    "depth_layer": 1, "expiration_raw": "2025.01.01", "status": "Expired",
                },
            ],
            [
                {"container_type": "checkout", "valid": True, "drop_pose_base": [260.0, -240.0, 70.0], "confidence": 0.99},
                {"container_type": "disposal", "valid": True, "drop_pose_base": [260.0, 290.0, 70.0], "confidence": 0.98},
            ],
        ),
        execute_action_fn=lambda _s: (True, "OK"),
        display_tiers=tiers,
        scene_explorer=explorer,
    )

    trace = engine.handle_command("포스틱 하나 줘")
    assert trace.success is True
    assert len(trace.cycles[0]["explorer_queries"]) == 3
    assert "scene_explorer_evidence" in captured_payload["user"]
    ev = captured_payload["user"]["scene_explorer_evidence"]["evidence"]
    assert ev["ask_sellable_items"][0]["track_id"] == "t_soon"
    assert ev["rel:disposal_candidates"][0]["track_id"] == "t_exp"
