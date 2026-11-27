#!/usr/bin/env python3
"""STOREPLAN 순수 CLI 벤치마크 & 음성/텍스트 대화형 논리 트레이서."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time
from typing import Any

WS_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS_ROOT / "src" / "store_llm_agent"))
sys.path.insert(0, str(WS_ROOT / "src" / "store_robot_control"))

from store_llm_agent.core.sim_world import (
    SimWorld,
    build_live_engine,
    narrate_trace_korean,
    run_benchmark,
)
from store_llm_agent.core.voice_stt import WhisperSTT


def _run_single_command(
    world: SimWorld,
    text: str,
    source: str,
    employee_mode: bool,
    engine: Any | None = None,
) -> int:
    active_engine = engine if engine is not None else build_live_engine(world, employee_mode=employee_mode)
    trace = active_engine.handle_command(text, source=source)
    print("\n" + "=" * 72)
    print(narrate_trace_korean(trace))
    print("=" * 72 + "\n")
    return 0 if trace.success else 1


def _run_interactive(initial_preset: str, employee_mode: bool) -> int:
    world = SimWorld()
    world.load_preset(initial_preset)
    stt: WhisperSTT | None = None
    emp = employee_mode
    engine = build_live_engine(world, employee_mode=emp)

    print("\n=== STOREPLAN 대화형 음성/텍스트 벤치마크 ===")
    print("명령 안내:")
    print("  [엔터 또는 v]       마이크 음성 녹음(5초) 후 실행")
    print("  [문장 입력]         해당 문장으로 즉시 실행 (확인질문 뒤 짧은 답변 연속 지원)")
    print("  [mode / :emp / :cust] 직원↔손님 모드 전환 (emp/cust 단축 입력 가능)")
    print("  [preset 이름]       프리셋 초기화 (:preset 포함, " + ", ".join(world.list_presets()) + ")")
    print("  [q / :q / quit]     종료\n")

    while True:
        mode_tag = "직원" if emp else "손님"
        try:
            raw = input(f"[{mode_tag} | {world.active_preset}] 입력 (v=음성, q=종료) > ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n종료합니다.")
            return 0

        low = raw.lower()
        if low in ("q", ":q", "quit", ":quit", "exit", ":exit"):
            print("종료합니다.")
            return 0
        if low in ("help", ":help", "?"):
            print("  v / 엔터: 5초 음성 입력 | mode / :emp / :cust: 모드 전환 | preset <이름>: 프리셋 | q: 종료")
            continue
        if low in ("mode", ":mode"):
            emp = not emp
            engine.fsm.set_employee_mode(emp)
            print(f"→ {'직원 모드(EMPLOYEE)' if emp else '손님 모드(CUSTOMER)'}로 전환했습니다.")
            continue
        if low in ("emp", ":emp"):
            emp = True
            engine.fsm.set_employee_mode(True)
            print("→ 직원 모드(EMPLOYEE)로 전환했습니다.")
            continue
        if low in ("cust", ":cust"):
            emp = False
            engine.fsm.set_employee_mode(False)
            print("→ 손님 모드(CUSTOMER)로 전환했습니다.")
            continue
        if low.startswith((":preset", "preset")):
            parts = raw.split(maxsplit=1)
            if len(parts) == 2 and parts[1].strip() in world.list_presets():
                preset_name = parts[1].strip()
                world.load_preset(preset_name)
                engine.clear_clarification()
                print(f"→ 프리셋 '{preset_name}'로 초기화했습니다.")
            else:
                print(f"사용 가능한 프리셋: {world.list_presets()}")
            continue

        if raw in ("", "v", "V"):
            if stt is None:
                stt = WhisperSTT()
            print(f"🎤 {stt.duration_sec:.0f}초 동안 말씀해 주세요...")
            t_stt_start = time.perf_counter()
            try:
                rec_info = {"done_at": t_stt_start}

                def _on_rec_done(rec_sec: float) -> None:
                    rec_info["done_at"] = time.perf_counter()
                    print(f"⏳ 녹음 완료 ({rec_sec:.2f}초) → Whisper 음성 인식(API) 변환 중...")

                text = stt.speech2text(on_record_done=_on_rec_done)
                api_sec = time.perf_counter() - rec_info["done_at"]
                print(f"✅ 음성 인식 완료 (변환 {api_sec:.2f}초): \"{text}\"")
            except Exception as exc:
                print(f"음성 인식 오류: {exc}")
                continue
            if not text:
                print("인식된 음성이 없습니다.")
                continue
            source = "voice"
        else:
            text = raw
            source = "text"

        _run_single_command(world, text, source=source, employee_mode=emp, engine=engine)


def main() -> int:
    parser = argparse.ArgumentParser(description="STOREPLAN 순수 CLI 벤치마크 & 음성 논리 트레이서")
    parser.add_argument(
        "--mode",
        choices=("offline", "live"),
        default=None,
        help="일괄 벤치마크 모드 (offline: 모의 LLM / live: 실제 GPT-4o)",
    )
    parser.add_argument("--voice", action="store_true", help="실제 마이크 녹음(whisper-1)으로 1회 실행")
    parser.add_argument("--text", type=str, default="", help="텍스트 명령 1회 실행 (실제 GPT-4o)")
    parser.add_argument("--interactive", action="store_true", help="음성/텍스트 반복 대화형 모드")
    parser.add_argument(
        "--preset",
        choices=("default_store", "inbound_3tier", "order_fefo", "tier3_backside", "all_exceptions"),
        default="default_store",
        help="단발/대화형 실행 시 초기 가상 매장 프리셋",
    )
    parser.add_argument("--emp", action="store_true", help="직원 모드(EMPLOYEE)로 시작")
    parser.add_argument("--verbose", action="store_true", help="일괄 벤치마크 시 자연어 논리 흐름까지 모두 출력")
    args = parser.parse_args()

    if args.interactive:
        return _run_interactive(args.preset, args.emp)

    if args.voice:
        world = SimWorld()
        world.load_preset(args.preset)
        stt = WhisperSTT()
        print(f"🎤 {stt.duration_sec:.0f}초 동안 말씀해 주세요...")
        rec_info = {"done_at": time.perf_counter()}

        def _on_rec_done(rec_sec: float) -> None:
            rec_info["done_at"] = time.perf_counter()
            print(f"⏳ 녹음 완료 ({rec_sec:.2f}초) → Whisper 음성 인식(API) 변환 중...")

        text = stt.speech2text(on_record_done=_on_rec_done)
        api_sec = time.perf_counter() - rec_info["done_at"]
        print(f"✅ 음성 인식 완료 (변환 {api_sec:.2f}초): \"{text}\"")
        if not text:
            print("인식된 음성이 없습니다.")
            return 1
        return _run_single_command(world, text, source="voice", employee_mode=args.emp)

    if args.text:
        world = SimWorld()
        world.load_preset(args.preset)
        return _run_single_command(world, args.text, source="text", employee_mode=args.emp)

    mode = args.mode or "offline"
    rows = run_benchmark(mode=mode)
    passed_cnt = sum(1 for r in rows if r.passed)

    print(f"\n=== STOREPLAN CLI Benchmark (mode={mode}) ===")
    print(f"{'시나리오':34s} | {'결과':4s} | {'스텝':4s} | {'재스캔':6s} | {'안전위반':8s} | {'시간(ms)':8s} | 상세")
    print("-" * 110)
    for row in rows:
        status = "PASS" if row.passed else "FAIL"
        print(
            f"{row.name:34s} | {status:4s} | {row.steps:4d} | {row.rescans:6d} | "
            f"{row.safety_violations:8d} | {row.elapsed_ms:8.2f} | {row.detail}"
        )
        if args.verbose and row.trace is not None:
            print("\n" + narrate_trace_korean(row.trace))
            print("-" * 110)

    print(f"총계: {passed_cnt} / {len(rows)} PASS\n")
    return 0 if passed_cnt == len(rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
