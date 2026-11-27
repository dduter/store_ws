#!/usr/bin/env python3
"""실제 음성(whisper-1)/텍스트 + 실제 GPT-4o + 가상 매대 프리셋(mm+deg)으로 행동 리스트를 눈으로 검증하는 CLI 트레이서."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

WS_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS_ROOT / "src" / "store_llm_agent"))

from store_llm_agent.core.action_domain import ActionDomain
from store_llm_agent.core.kg_manager import KGManager
from store_llm_agent.core.llm_parser import LLMParser
from store_llm_agent.core.mode_fsm import ModeFSM
from store_llm_agent.core.paths import find_workspace_root
from store_llm_agent.core.pipeline_engine import (
    StorePipelineEngine,
    load_buffer_slots,
    load_display_tiers,
    load_zone_bounds,
)
from store_llm_agent.core.symbolic_planner import SymbolicPlanner
from store_llm_agent.core.task_formalizer import TaskFormalizer
from store_llm_agent.core.voice_stt import WhisperSTT

WS_ROOT = find_workspace_root(__file__)

CONTAINERS_OK = [
    {"container_type": "checkout", "valid": True, "drop_pose_base": [300.0, -250.0, 80.0], "confidence": 0.99},
    {"container_type": "disposal", "valid": True, "drop_pose_base": [300.0, 250.0, 80.0], "confidence": 0.98},
]

PRESETS = {
    "normal": [(
        [
            {"track_id": "t1", "product_name": "포스틱", "yolo_class": "postick_front", "is_backside": False,
             "pose_3d_base": [400.0, 100.0, 50.0, 0.0], "depth_layer": 1, "status": "Normal"},
            {"track_id": "t2", "product_name": "아몬드 빼빼로", "yolo_class": "pepero_almond_front", "is_backside": False,
             "pose_3d_base": [420.0, -50.0, 50.0, 0.0], "depth_layer": 1, "status": "Normal"},
            {"track_id": "t3", "product_name": "병음료", "yolo_class": "drink_front", "is_backside": False,
             "pose_3d_base": [380.0, 150.0, 40.0, 0.0], "depth_layer": 1, "status": "Normal"},
        ],
        CONTAINERS_OK,
    )],
    "expired": [(
        [
            {"track_id": "t1", "product_name": "포스틱", "yolo_class": "postick_front", "is_backside": False,
             "pose_3d_base": [400.0, 100.0, 50.0, 0.0], "depth_layer": 1,
             "status": "Expired", "disposal_reason": "유통기한 만료(2025.01.01)"},
            {"track_id": "t2", "product_name": "포스틱", "yolo_class": "postick_front", "is_backside": False,
             "pose_3d_base": [450.0, 120.0, 50.0, 0.0], "depth_layer": 1, "status": "Normal"},
        ],
        CONTAINERS_OK,
    )],
    "backside": [
        (
            [{"track_id": "t3", "product_name": "", "yolo_class": "product_back", "is_backside": True,
              "pose_3d_base": [400.0, 0.0, 50.0, 0.0], "depth_layer": 1,
              "status": "NeedsReorientation"}],
            CONTAINERS_OK,
        ),
        (
            [{"track_id": "t3", "product_name": "아몬드 빼빼로", "yolo_class": "pepero_almond_front", "is_backside": False,
              "pose_3d_base": [400.0, 0.0, 50.0, 0.0], "depth_layer": 1,
              "status": "Normal"}],
            CONTAINERS_OK,
        ),
    ],
    "inbound_2layer": [
        (
            [{"track_id": "b1", "product_name": "병음료", "yolo_class": "drink_front", "is_backside": False,
              "pose_3d_base": [350.0, 260.0, 90.0, 0.0], "depth_layer": 2,
              "status": "Normal"}],
            CONTAINERS_OK,
        ),
        (
            [{"track_id": "target_a", "product_name": "포스틱", "yolo_class": "postick_front", "is_backside": False,
              "pose_3d_base": [350.0, 260.0, 45.0, 0.0], "depth_layer": 1,
              "status": "Normal"}],
            CONTAINERS_OK,
        ),
    ],
    "inbound_3tier": [
        (
            [
                {"track_id": "in1", "product_name": "포스틱", "yolo_class": "postick_front",
                 "is_backside": False, "pose_3d_base": [310.0, 240.0, 45.0, 0.0], "depth_layer": 1, "status": "Normal"},
                {"track_id": "in2", "product_name": "병음료", "yolo_class": "drink_front",
                 "is_backside": False, "pose_3d_base": [360.0, 240.0, 45.0, 0.0], "depth_layer": 1, "status": "Normal"},
                {"track_id": "in3", "product_name": "", "yolo_class": "product_back",
                 "is_backside": True, "pose_3d_base": [410.0, 240.0, 45.0, 0.0], "depth_layer": 1,
                 "status": "NeedsReorientation"},
            ],
            CONTAINERS_OK,
        ),
        (
            [
                {"track_id": "in1", "product_name": "포스틱", "yolo_class": "postick_front",
                 "is_backside": False, "pose_3d_base": [300.0, -150.0, 40.0, 0.0], "depth_layer": 1, "status": "Normal"},
                {"track_id": "in2", "product_name": "병음료", "yolo_class": "drink_front",
                 "is_backside": False, "pose_3d_base": [420.0, -150.0, 140.0, 0.0], "depth_layer": 1, "status": "Normal"},
                {"track_id": "in3", "product_name": "아몬드 빼빼로", "yolo_class": "pepero_almond_front",
                 "is_backside": False, "pose_3d_base": [410.0, 240.0, 45.0, 0.0], "depth_layer": 1, "status": "Normal"},
            ],
            CONTAINERS_OK,
        ),
    ],
    "tier3_backside": [
        (
            [{"track_id": "t_high", "product_name": "", "yolo_class": "product_back", "is_backside": True,
              "pose_3d_base": [520.0, 0.0, 240.0, 0.0], "depth_layer": 1, "status": "NeedsReorientation"}],
            CONTAINERS_OK,
        ),
        (
            [{"track_id": "t_high", "product_name": "아몬드 빼빼로", "yolo_class": "pepero_almond_front", "is_backside": False,
              "pose_3d_base": [300.0, -280.0, 40.0, 0.0], "depth_layer": 1, "status": "Normal"}],
            CONTAINERS_OK,
        ),
    ],
}


def main() -> None:
    parser = argparse.ArgumentParser(description="Store LLM 파이프라인 행동 리스트 대화형 트레이서")
    parser.add_argument("--preset", choices=list(PRESETS.keys()), default="normal", help="가상 매대 프리셋")
    parser.add_argument("--mode", choices=["CUSTOMER", "EMPLOYEE"], default="CUSTOMER", help="초기 운영 모드")
    parser.add_argument("--voice", action="store_true", help="실제 마이크 녹음(whisper-1)으로 입력")
    parser.add_argument("--text", type=str, default="", help="단발성 텍스트 명령 실행")
    args = parser.parse_args()

    kg = KGManager()
    kg.load_initial_graph(
        WS_ROOT / "config" / "ontology" / "store_ontology.ttl",
        WS_ROOT / "config" / "products_catalog.yaml",
    )
    fsm = ModeFSM(required_vest_frames=1)
    if args.mode == "EMPLOYEE":
        fsm.update_vest_detection(True, 0.99, now=1.0)

    scan_seq = PRESETS[args.preset]
    idx = {"i": 0}

    def scan_fn(_zone: str, _run_vlm: bool):
        cur = min(idx["i"], len(scan_seq) - 1)
        idx["i"] += 1
        prods, conts = scan_seq[cur]
        return True, "OK", prods, conts

    zones_path = WS_ROOT / "config" / "workspace_zones.yaml"
    display_tiers = load_display_tiers(zones_path)
    engine = StorePipelineEngine(
        kg_manager=kg,
        mode_fsm=fsm,
        llm_parser=LLMParser(),
        task_formalizer=TaskFormalizer(),
        action_domain=ActionDomain(
            display_tiers=display_tiers,
            buffer_slots=load_buffer_slots(zones_path),
            zone_bounds=load_zone_bounds(zones_path),
        ),
        symbolic_planner=SymbolicPlanner(),
        scan_fn=scan_fn,
        execute_action_fn=lambda _step: (True, "OK"),
        display_tiers=display_tiers,
    )

    if args.voice:
        stt = WhisperSTT()
        text = stt.speech2text()
        source = "voice"
    elif args.text:
        text = args.text
        source = "text"
    else:
        text = input(f"[{args.mode}/{args.preset}] 명령 입력 > ").strip()
        source = "text"

    trace = engine.handle_command(text, source=source)

    print("\n================ PIPELINE TRACE ================")
    print(f"입력({trace.source}): {trace.input_text}")
    print(f"모드: {trace.mode} | 허용: {trace.allowed} | 성공: {trace.success} | 안전정지: {trace.safety_stop}")
    if trace.parsed:
        print(f"LLM 파싱: {trace.parsed.model_dump()}")
    print(f"최종 메시지: {trace.message}")
    print("\n[생성된 로봇 행동 리스트 (mm + deg)]")
    for i, step in enumerate(trace.executed_actions, 1):
        print(f"  {i}. {step.action_type:20s} | cat={step.product_category or '-':15s} | "
              f"target={step.target_pose} -> dest={step.dest_pose} | {step.reason}")
    print("\n[단계별 로그]")
    for line in trace.logs:
        print(f"  {line}")
    print("================================================")


if __name__ == "__main__":
    main()
