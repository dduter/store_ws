"""STOREPLAN 기반 음성/텍스트 입력 → 2차 LLM 형식화 → BFS 계획 → 순차 실행 파이프라인 엔진."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import yaml

from store_llm_agent.core.action_domain import (
    ActionDomain,
    BufferEntry,
    RobotActionStep,
)
from store_llm_agent.core.kg_manager import KGManager, _get_field
from store_llm_agent.core.llm_parser import LLMParser, OrderConditions
from store_llm_agent.core.mode_fsm import ModeFSM
from store_llm_agent.core.symbolic_planner import SymbolicPlanner
from store_llm_agent.core.task_formalizer import TaskFormalizer

ScanCallback = Callable[[str, bool], tuple[bool, str, list[Any], list[Any]]]
ExecuteCallback = Callable[[RobotActionStep], tuple[bool, str]]

SAFETY_ERROR_CODES: frozenset[str] = frozenset({
    "Z_MIN_VIOLATION",
    "J6_LIMIT_VIOLATION",
    "CANCELED",
    "SAFETY_STOP",
})


def load_buffer_slots(zones_yaml_path: str | Path) -> list[list[float]] | None:
    """workspace_zones.yaml에서 입고 임시 버퍼 슬롯 좌표(zones.buffer_slots)를 로드한다."""
    path = Path(zones_yaml_path)
    if not path.exists():
        return None
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    slots = (data.get("zones") or {}).get("buffer_slots") or data.get("inbound_buffer_slots")
    if isinstance(slots, list) and slots:
        return [[float(v) for v in s] for s in slots]
    return None


def load_display_tiers(zones_yaml_path: str | Path) -> dict[str, list[list[float]]]:
    """workspace_zones.yaml에서 3단 계단형 진열대 슬롯 좌표(zones.display_tiers)를 로드한다."""
    path = Path(zones_yaml_path)
    if not path.exists():
        return {}
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    raw_tiers = (data.get("zones") or {}).get("display_tiers") or {}
    tiers: dict[str, list[list[float]]] = {}
    if isinstance(raw_tiers, dict):
        for tier_name, slots in raw_tiers.items():
            if isinstance(slots, list):
                tiers[str(tier_name)] = [[float(v) for v in s] for s in slots]
    return tiers


def load_zone_bounds(zones_yaml_path: str | Path) -> dict[str, dict[str, list[float]]]:
    """workspace_zones.yaml에서 구역 경계(display_area, inbound_box)의 x_range/y_range를 로드한다."""
    path = Path(zones_yaml_path)
    if not path.exists():
        return {}
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    zones = data.get("zones") or {}
    bounds: dict[str, dict[str, list[float]]] = {}
    for zone_name in ("display_area", "inbound_box"):
        raw = zones.get(zone_name)
        if isinstance(raw, dict) and "x_range" in raw and "y_range" in raw:
            bounds[zone_name] = {
                "x_range": [float(v) for v in raw["x_range"][:2]],
                "y_range": [float(v) for v in raw["y_range"][:2]],
            }
    return bounds


def load_flip_zone(zones_yaml_path: str | Path) -> list[float] | None:
    """workspace_zones.yaml에서 고정 Flip Zone 좌표(zones.flip_zone)를 로드한다."""
    path = Path(zones_yaml_path)
    if not path.exists():
        return None
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    raw = (data.get("zones") or {}).get("flip_zone")
    if isinstance(raw, list) and len(raw) >= 3:
        vals = [float(v) for v in raw[:4]]
        while len(vals) < 4:
            vals.append(0.0)
        return vals
    return None


@dataclass
class PipelineTrace:
    """단일 명령 처리의 전체 추적 결과 (테스트 · 트레이서 · UI 로그 공용)."""

    input_text: str
    source: str
    mode: str
    parsed: OrderConditions | None = None
    allowed: bool = True
    success: bool = False
    safety_stop: bool = False
    needs_clarification: bool = False
    clarification_question: str = ""
    clarification_reason: str = ""
    inferred_candidates: list[str] = field(default_factory=list)
    message: str = ""
    catalog_candidates: list[dict[str, Any]] = field(default_factory=list)
    executed_actions: list[RobotActionStep] = field(default_factory=list)
    logs: list[str] = field(default_factory=list)
    cycles: list[dict[str, Any]] = field(default_factory=list)


class StorePipelineEngine:
    """STOREPLAN(TaskFormalizer + ActionDomain + SymbolicPlanner) 기반 통합 실행 엔진."""

    def __init__(
        self,
        kg_manager: KGManager,
        mode_fsm: ModeFSM,
        llm_parser: LLMParser,
        task_formalizer: TaskFormalizer,
        action_domain: ActionDomain,
        symbolic_planner: SymbolicPlanner,
        scan_fn: ScanCallback,
        execute_action_fn: ExecuteCallback,
        display_tiers: dict[str, list[list[float]]] | None = None,
        max_rescan_loops: int = 16,
    ) -> None:
        self.kg = kg_manager
        self.fsm = mode_fsm
        self.parser = llm_parser
        self.formalizer = task_formalizer
        self.domain = action_domain
        self.planner = symbolic_planner
        self._scan_fn = scan_fn
        self._execute_fn = execute_action_fn
        self._display_tiers = dict(display_tiers or {})
        self.max_rescan_loops = max_rescan_loops
        self._last_products: list[Any] = []
        self._agent_logs: list[str] = []
        self._pending_clarification: dict[str, Any] | None = None

    def clear_clarification(self) -> None:
        """대기 중인 확인질문 대화 맥락을 초기화한다."""
        self._pending_clarification = None

    def _finish_with_clarification(
        self,
        trace: PipelineTrace,
        question: str,
        reason: str,
        candidates: list[str],
        raw_text: str,
    ) -> PipelineTrace:
        """로봇 행동 없이 온톨로지 기반 확인질문(역질문)을 반환하고 다음 턴 맥락을 저장한다."""
        clean_q = question.strip() or (
            "말씀하신 내용을 정확히 이해하지 못했습니다. 찾으시는 상품명이나 원하시는 작업을 다시 말씀해 주세요."
        )
        trace.needs_clarification = True
        trace.clarification_question = clean_q
        trace.clarification_reason = reason
        trace.inferred_candidates = list(dict.fromkeys(c for c in candidates if c))
        trace.success = False
        trace.message = clean_q
        self._pending_clarification = {
            "raw_text": raw_text,
            "question": clean_q,
            "candidates": trace.inferred_candidates,
            "reason": reason,
        }
        self._log(trace, f"[CLARIFY] {reason} -> {clean_q}")
        return trace

    def _log(self, trace: PipelineTrace, msg: str) -> None:
        trace.logs.append(msg)
        self._agent_logs.append(msg)
        if len(self._agent_logs) > 50:
            self._agent_logs = self._agent_logs[-50:]

    def _run_step(self, trace: PipelineTrace, step: RobotActionStep) -> tuple[bool, str]:
        self._log(trace, f"[ACTION] {step.action_type} (track={step.track_id or '-'}) : {step.reason}")
        ok, err = self._execute_fn(step)
        trace.executed_actions.append(step)
        if not ok:
            self._log(trace, f"[ACTION_FAIL] {step.action_type}: {err}")
            if err in SAFETY_ERROR_CODES:
                trace.safety_stop = True
        return ok, err

    def _can_retry_grasp_fail(
        self,
        trace: PipelineTrace,
        step: RobotActionStep,
        step_err: str,
        grasp_retry_counts: dict[str, int],
    ) -> bool:
        """GRASP_FAIL 시 ScanWorkspace 재촬영 후 동일 track_id당 최대 1회만 재시도를 허용한다."""
        if step_err != "GRASP_FAIL" or not step.track_id or step.action_type == "scan_pose":
            return False
        if grasp_retry_counts.get(step.track_id, 0) >= 1:
            return False
        grasp_retry_counts[step.track_id] = 1
        self._log(trace, f"[GRASP_FAIL_RETRY] '{step.track_id}' 좌표 재스캔 후 1회 재시도")
        return True

    def _scan_and_sync(
        self,
        trace: PipelineTrace,
        zone: str,
        run_vlm: bool,
    ) -> tuple[bool, list[Any], dict[str, dict[str, Any]]]:
        ok, err = self._run_step(
            trace,
            RobotActionStep(action_type="scan_pose", reason=f"스캔 위치 이동 (zone={zone})"),
        )
        if not ok:
            trace.message = f"scan_pose 이동 실패: {err}"
            return False, [], {}

        scan_ok, scan_err, products, containers = self._scan_fn(zone, run_vlm)
        if not scan_ok:
            trace.message = f"ScanWorkspace 실패: {scan_err}"
            self._log(trace, f"[SCAN_FAIL] {scan_err}")
            return False, [], {}

        self.kg.sync_from_scan(products, containers)
        self._last_products = list(products)
        self._log(trace, f"[SCAN] 상품 {len(products)}개, 컨테이너 {len(containers)}개 동기화 완료")
        return True, products, self.kg.get_containers()

    def handle_command(self, text: str, source: str = "text") -> PipelineTrace:
        """자연어 명령을 받아 권한 확인 → 1차 파싱 → STOREPLAN 폐루프 실행을 수행한다."""
        trace = PipelineTrace(input_text=text, source=source, mode=self.fsm.mode)
        self._log(trace, f"[INPUT:{source}] '{text}' (mode={self.fsm.mode})")

        # 1) 1차 LLM 의도 및 슬롯 추출 (대기 중인 확인질문이 있으면 대화 맥락 전달)
        all_catalog = self.kg.query_catalog_by_slots({"include_remote_auth": True})
        conv_ctx = ""
        if self._pending_clarification:
            prev = self._pending_clarification
            conv_ctx = (
                f"직전 사용자 발화='{prev['raw_text']}', "
                f"로봇 확인질문='{prev['question']}', 후보={prev['candidates']}"
            )
        try:
            if conv_ctx:
                parsed = self.parser.parse_intent(
                    text, catalog_candidates=all_catalog, conversation_context=conv_ctx
                )
            else:
                parsed = self.parser.parse_intent(text, catalog_candidates=all_catalog)
        except Exception as exc:
            trace.message = f"의도 파싱 실패: {exc}"
            self._log(trace, f"[PARSE_ERR] {exc}")
            return trace

        trace.parsed = parsed
        self._log(trace, f"[PARSE] intent={parsed.intent}, items={len(parsed.items)}")

        # 2) 모드 권한 체크
        allowed, reason = self.fsm.check_permission(parsed.intent)
        trace.allowed = allowed
        if not allowed:
            trace.message = reason
            self._log(trace, f"[BLOCKED] {reason}")
            return trace

        # 3) 명확성·역질문 분기 (1차 LLM 판정 + 온톨로지 관계 추론)
        if parsed.intent == "clarify":
            cands = list(parsed.candidate_names)
            clar_reason = parsed.clarification_reason or "모호하거나 매장 카탈로그와 즉시 일치하지 않는 발화"
            clar_question = parsed.clarification_question
            if not cands:
                slots = parsed.items[0].to_slot_dict() if parsed.items else {}
                rel = self.kg.infer_ontology_relations(slots, raw_text=text)
                if rel["match_type"] == "exact":
                    cands = [c["name"] for c in rel["all_candidates"] if c.get("name")]
                elif rel["match_type"] == "related":
                    cands = [c["name"] for c in rel["related_candidates"] if c.get("name")]
                    clar_reason = f"온톨로지 관계 추론: {', '.join(rel['relation_reasons'][:3])}"
                if cands:
                    clar_question = (
                        f"온톨로지 추론 결과 관련 상품으로 {', '.join(cands)}이(가) 있습니다. "
                        "이 중 원하시는 상품이 있나요?"
                    )
            return self._finish_with_clarification(
                trace,
                question=clar_question,
                reason=clar_reason,
                candidates=cands,
                raw_text=text,
            )

        # 4) 1차 정적 카탈로그 후보 조회
        if parsed.items:
            trace.catalog_candidates = self.kg.query_catalog_by_slots(parsed.items[0].to_slot_dict())
            self._log(
                trace,
                f"[KG_STATIC] 1차 후보 {len(trace.catalog_candidates)}건: "
                f"{[c['name'] for c in trace.catalog_candidates]}",
            )

        # 5) 작업 실행 시작
        self.fsm.set_task_active(True)
        try:
            intent = parsed.intent

            if intent == "query_stock":
                self._pending_clarification = None
                ok, _, _ = self._scan_and_sync(trace, zone="all", run_vlm=False)
                if not ok:
                    return trace
                slots = parsed.items[0].to_slot_dict() if parsed.items else {}
                sellable = self.kg.query_sellable_by_slots(slots)
                trace.success = True
                trace.message = f"판매 가능 재고 {len(sellable)}건: {[s['name'] for s in sellable]}"
                self._log(trace, f"[STOCK] {trace.message}")
                return trace

            if intent == "remote":
                self._pending_clarification = None
                trace.allowed = False
                trace.message = "원격 모드는 관리자 키 인증(/store/verify_remote_key)을 통해서만 전환할 수 있습니다."
                self._log(trace, f"[BLOCKED] {trace.message}")
                return trace

            if intent == "order":
                if not parsed.items:
                    return self._finish_with_clarification(
                        trace,
                        question="어떤 상품을 찾으시나요? 상품명이나 맛, 색상을 말씀해 주세요.",
                        reason="주문 상품 조건 누락",
                        candidates=[],
                        raw_text=text,
                    )
                for item in parsed.items:
                    slots = item.to_slot_dict()
                    rel = self.kg.infer_ontology_relations(slots, raw_text=text)
                    normal_cands = rel["normal_candidates"]
                    all_cands = rel["all_candidates"]

                    if not normal_cands and any(c.get("requires_remote_auth") for c in all_cands):
                        self._pending_clarification = None
                        names = ", ".join(
                            c["name"] for c in all_cands if c.get("requires_remote_auth") and c.get("name")
                        )
                        trace.allowed = False
                        trace.message = (
                            f"원격 관리자 인증이 필요한 상품입니다 ({names or '원격 전용'}). "
                            "원격 제어 전환을 대기합니다."
                        )
                        self._log(trace, f"[REMOTE_AUTH_REQUIRED] {trace.message}")
                        return trace

                    if rel["match_type"] == "related":
                        rel_names = [c["name"] for c in rel["related_candidates"] if c.get("name")]
                        if len(rel_names) == 1:
                            q_msg = f"말씀하신 상품이 '{rel_names[0]}'이(가) 맞나요?"
                        else:
                            q_msg = f"말씀하신 내용과 관련된 매장의 상품은 {', '.join(rel_names)}입니다. 어느 상품을 원하시나요?"
                        return self._finish_with_clarification(
                            trace,
                            question=q_msg,
                            reason=f"온톨로지 관계 추론: {', '.join(rel['relation_reasons'][:3])}",
                            candidates=rel_names,
                            raw_text=text,
                        )

                    if rel["match_type"] == "none":
                        sellable_names = ", ".join(
                            str(c["name"])
                            for c in self.kg.query_catalog_by_slots({})
                            if c.get("name")
                        )
                        return self._finish_with_clarification(
                            trace,
                            question=(
                                f"말씀하신 상품은 매장 카탈로그에서 찾지 못했습니다. "
                                f"취급 상품({sellable_names or '등록 상품'}) 중 찾으시는 상품을 다시 말씀해 주세요."
                            ),
                            reason="온톨로지 카탈로그 미매칭",
                            candidates=[],
                            raw_text=text,
                        )

                    # 단일 상품(quantity=1)에 상품명 미지정 & 제외조건/아무거나 명시가 없는데 2종 이상 매칭되면 확인질문
                    is_explicit_open = bool(item.exclude) or item.quantity > 1 or any(
                        kw in text for kw in ("아무", "상관없", "랜덤", "전부", "모두")
                    )
                    if len(normal_cands) > 1 and not item.name and not is_explicit_open:
                        cand_names = [c["name"] for c in normal_cands if c.get("name")]
                        cond_desc = ", ".join(
                            f"{k}={v}" for k, v in slots.items() if v and k != "exclude"
                        )
                        return self._finish_with_clarification(
                            trace,
                            question=f"조건({cond_desc})에 해당하는 상품이 {', '.join(cand_names)} 여러 종 있습니다. 어느 상품을 드릴까요?",
                            reason=f"온톨로지 다중 후보({cond_desc})",
                            candidates=cand_names,
                            raw_text=text,
                        )

                self._pending_clarification = None
                return self._run_storeplan_loop(trace, parsed, zone="all")

            if intent in ("inspect", "dispose"):
                self._pending_clarification = None
                return self._run_storeplan_loop(trace, parsed, zone="inspect")

            if intent == "inbound":
                self._pending_clarification = None
                return self._run_storeplan_loop(trace, parsed, zone="inbound")

            trace.message = f"지원하지 않는 intent입니다: {intent}"
            return trace
        finally:
            self.fsm.set_task_active(False)

    def _run_storeplan_loop(
        self,
        trace: PipelineTrace,
        parsed: OrderConditions,
        zone: str,
    ) -> PipelineTrace:
        """STOREPLAN 통합 폐루프: 스캔 → 2차 LLM TaskSpec → Subgoal별 BFS → 1스텝 실행 → 재스캔."""
        reorient_counts: dict[str, int] = {}
        grasp_retry_counts: dict[str, int] = {}
        buffer_stack: list[BufferEntry] = []

        total_requested = sum(item.quantity for item in parsed.items) if parsed.intent == "order" else 0
        checkout_fulfilled = 0
        last_unfulfillable: str | None = None
        moved = False
        exceeded_loops = True

        for cycle_idx in range(1, self.max_rescan_loops + 1):
            ok, _, _ = self._scan_and_sync(trace, zone=zone, run_vlm=True)
            moved = True
            if not ok:
                return trace

            snap = self.kg.get_world_state_snapshot(display_tiers=self._display_tiers)
            world_state = self.domain.build_world_state(
                snap,
                reorient_counts=reorient_counts,
                buffer_stack=buffer_stack,
            )
            cycle_info: dict[str, Any] = {
                "cycle": cycle_idx,
                "observed": [
                    {
                        "track_id": p.track_id,
                        "name": p.name,
                        "status": p.status,
                        "is_backside": p.is_backside,
                        "depth_layer": p.depth_layer,
                        "location": p.location,
                        "z_mm": round(p.pose[2], 1) if p.pose_valid else None,
                    }
                    for p in world_state.products
                ],
            }

            try:
                task_spec, sanitize_warnings = self.formalizer.formalize(
                    parsed, world_state, snap["catalog"], self.domain
                )
            except Exception as exc:
                trace.message = f"TaskFormalizer 오류: {exc}"
                self._log(trace, f"[FORMALIZE_ERR] {exc}")
                trace.cycles.append(cycle_info)
                return trace

            cycle_info["reasoning"] = task_spec.reasoning
            cycle_info["subgoals"] = [
                {
                    "goal_type": sg.goal_type,
                    "track_id": sg.track_id,
                    "target": sg.target,
                    "reason": sg.reason,
                }
                for sg in task_spec.subgoals
            ]
            cycle_info["unfulfillable_reason"] = task_spec.unfulfillable_reason
            cycle_info["sanitize_warnings"] = list(sanitize_warnings)

            self._log(
                trace,
                f"[STOREPLAN_SPEC] subgoals={len(task_spec.subgoals)} | {task_spec.reasoning or '-'}",
            )
            for warn in sanitize_warnings:
                self._log(trace, f"[SANITIZE_WARN] {warn}")

            last_unfulfillable = task_spec.unfulfillable_reason
            if not task_spec.subgoals:
                trace.cycles.append(cycle_info)
                exceeded_loops = False
                break

            plan = self.planner.plan(world_state, task_spec, self.domain)
            cycle_info["planned_actions"] = [
                {
                    "action_type": s.action_type,
                    "track_id": s.track_id,
                    "dest_id": s.dest_id,
                    "purpose": s.purpose,
                    "needs_rescan": s.needs_rescan,
                    "reason": s.reason,
                }
                for s in plan.symbolic_actions
            ]
            cycle_info["needs_rescan"] = plan.needs_rescan
            cycle_info["safety_stop_reason"] = plan.stop_reason if plan.safety_stop else ""
            trace.cycles.append(cycle_info)

            self._log(
                trace,
                f"[STOREPLAN_BFS] steps={len(plan.actions)}, rescan={plan.needs_rescan}, "
                f"solved={plan.solved_subgoals}/{plan.total_subgoals} ({plan.search_ms:.2f}ms)",
            )
            if plan.safety_stop:
                trace.safety_stop = True
                trace.message = plan.stop_reason
                self._log(trace, f"[SAFETY_STOP] {plan.stop_reason}")
                return trace

            if not plan.actions:
                exceeded_loops = False
                break

            cur_state = world_state
            rescan_for_retry = False
            for step, sym_act in zip(plan.actions, plan.symbolic_actions):
                step_ok, step_err = self._run_step(trace, step)
                if not step_ok:
                    if self._can_retry_grasp_fail(trace, step, step_err, grasp_retry_counts):
                        rescan_for_retry = True
                        break
                    if buffer_stack:
                        warn = f"버퍼 슬롯에 미복원 상품 {len(buffer_stack)}개가 남아 있습니다."
                        self._log(trace, f"[BUFFER_REMAINING] {warn}")
                        trace.message = f"{step_err} ({warn})"
                    else:
                        trace.message = step_err
                    return trace

                # 실제 실행 성공 후에만 상태·카운트·폐기 이력 갱신
                if sym_act.action_type == "pick_place" and sym_act.dest_id == "disposal":
                    prod = cur_state.get_product(sym_act.track_id)
                    reason = (
                        (prod.disposal_reason if prod and prod.disposal_reason else "")
                        or (prod.status if prod else "")
                        or "Unjudgeable"
                    )
                    self.kg.record_disposal(sym_act.track_id, reason)
                elif sym_act.action_type == "pick_place" and sym_act.dest_id == "checkout":
                    checkout_fulfilled += 1
                elif sym_act.action_type == "reorient":
                    reorient_counts[sym_act.track_id] = reorient_counts.get(sym_act.track_id, 0) + 1

                cur_state = self.domain.apply(cur_state, sym_act)
                buffer_stack = list(cur_state.buffer_stack)

            if rescan_for_retry or plan.needs_rescan:
                continue

            exceeded_loops = False
            break

        if exceeded_loops:
            trace.message = "최대 재스캔 횟수를 초과했습니다."
            return trace

        if moved:
            self._run_step(
                trace,
                RobotActionStep(action_type="home", reason=f"{parsed.intent} 처리 완료 홈 복귀"),
            )

        if parsed.intent == "order":
            if checkout_fulfilled >= total_requested and total_requested > 0:
                trace.success = True
                trace.message = "고객 주문 처리 완료"
            elif checkout_fulfilled > 0:
                trace.success = False
                trace.message = (
                    last_unfulfillable
                    or f"주문 수량 일부만 처리되었습니다 ({checkout_fulfilled}/{total_requested}개, 잔여 품절)."
                )
            else:
                trace.success = False
                trace.message = (
                    last_unfulfillable
                    or "요청한 상품을 매대에서 찾지 못했습니다 (품절 또는 판독 불가)."
                )
            return trace

        if last_unfulfillable and len(trace.executed_actions) <= 2:
            trace.success = False
            trace.message = last_unfulfillable
            return trace

        trace.success = True
        if parsed.intent == "inbound":
            trace.message = "입고 3단 진열대 정리 완료"
        else:
            trace.message = "검수 및 폐기 처리 완료"
        return trace

    def build_ui_snapshot(self) -> dict[str, Any]:
        """관제 UI(`/store/ui_snapshot`)에 발행할 5개 표준 키 JSON 딕셔너리를 생성한다."""
        kg_json = self.kg.to_networkx_json()
        inventory = [
            {
                "track_id": str(_get_field(p, "track_id", "")),
                "name": str(_get_field(p, "product_name", "")),
                "status": str(_get_field(p, "status", "")),
                "depth_layer": int(_get_field(p, "depth_layer", 1)),
            }
            for p in self._last_products
        ]
        return {
            "mode": self.fsm.mode,
            "inventory": inventory,
            "disposal_history": kg_json.get("disposal_history", []),
            "kg_graph": {
                "nodes": kg_json.get("nodes", []),
                "links": kg_json.get("links", []),
                "stats": kg_json.get("stats", {}),
            },
            "agent_log": list(self._agent_logs[-20:]),
        }
