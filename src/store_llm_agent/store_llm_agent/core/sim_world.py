"""STOREPLAN 상태 기반 3D 시뮬레이션 월드 및 오프라인 벤치마크 러너 (순수 Python)."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
import math
from pathlib import Path
import time
from types import SimpleNamespace
from typing import Any

import yaml

from store_llm_agent.core.action_domain import (
    ActionDomain,
    MAX_REORIENT_Z_MM,
    RobotActionStep,
)
from store_llm_agent.core.kg_manager import KGManager
from store_llm_agent.core.llm_parser import ItemCondition, LLMParser, OrderConditions
from store_llm_agent.core.mode_fsm import ModeFSM
from store_llm_agent.core.paths import find_workspace_root
from store_llm_agent.core.pipeline_engine import (
    PipelineTrace,
    StorePipelineEngine,
    load_buffer_slots,
    load_display_tiers,
    load_zone_bounds,
)
from store_llm_agent.core.symbolic_planner import SymbolicPlanner
from store_llm_agent.core.task_formalizer import Subgoal, TaskFormalizer, TaskSpec

PRODUCT_COLORS_RGB: dict[str, tuple[float, float, float]] = {
    "postick_front": (0.18, 0.48, 0.92),        # 파랑 (포스틱)
    "pepero_almond_front": (0.20, 0.72, 0.35),  # 초록 (아몬드 빼빼로)
    "drink_front": (0.35, 0.78, 0.90),          # 하늘 (병음료)
    "ciga": (0.90, 0.90, 0.92),                 # 하양 (모형 담배)
    "product_back": (0.52, 0.52, 0.56),         # 회색 (뒷면)
}

STATUS_CYCLE = ("Normal", "Expired", "MarkerContaminated", "Unreadable")


@dataclass
class SimProduct:
    """가상 매장에 배치된 단일 상품 객체."""

    track_id: str
    true_name: str
    true_yolo_class: str
    robot_category: str
    pose_mm: list[float]  # [x, y, z, rz]
    is_backside: bool = False
    depth_layer: int = 1
    expiration_raw: str = "2027.06.01"
    true_status: str = "Normal"
    disposal_reason: str = ""
    hidden_under: str | None = None
    size_mm: tuple[float, float, float] = (65.0, 45.0, 38.0)

    def to_detected_dict(self) -> dict[str, Any]:
        """비전(ScanWorkspace)이 현재 관측할 DetectedProduct 딕셔너리로 변환한다."""
        if self.is_backside:
            return {
                "track_id": self.track_id,
                "product_name": "",
                "yolo_class": "product_back",
                "is_backside": True,
                "confidence": 0.94,
                "bbox_px": [100, 100, 200, 200],
                "pose_3d_base": list(self.pose_mm),
                "grasp_width_mm": 0.0,
                "depth_layer": self.depth_layer,
                "expiration_raw": "",
                "status": "NeedsReorientation",
                "disposal_reason": "",
            }
        return {
            "track_id": self.track_id,
            "product_name": self.true_name,
            "yolo_class": self.true_yolo_class,
            "is_backside": False,
            "confidence": 0.96,
            "bbox_px": [100, 100, 200, 200],
            "pose_3d_base": list(self.pose_mm),
            "grasp_width_mm": 0.0,
            "depth_layer": self.depth_layer,
            "expiration_raw": self.expiration_raw,
            "status": self.true_status,
            "disposal_reason": self.disposal_reason,
        }


class SimWorld:
    """실시간 드래그·액션 실행·프리셋 로드를 지원하는 상태 기반 가상 매장 월드."""

    def __init__(self, config_dir: str | Path | None = None) -> None:
        ws_root = find_workspace_root(__file__)
        self.config_dir = Path(config_dir) if config_dir else (ws_root / "config")
        self.zones_path = self.config_dir / "workspace_zones.yaml"
        self.catalog_path = self.config_dir / "products_catalog.yaml"
        self.ttl_path = self.config_dir / "ontology" / "store_ontology.ttl"

        with open(self.zones_path, "r", encoding="utf-8") as f:
            self.zones_cfg = yaml.safe_load(f) or {}

        self.display_tiers = load_display_tiers(self.zones_path)
        self.buffer_slots = load_buffer_slots(self.zones_path) or []
        self.zone_bounds = load_zone_bounds(self.zones_path)

        # 선택적 로봇 파트 SafetyGuard / GraspPlanner 직결 검증
        self._safety_guard = None
        self._grasp_planner = None
        try:
            from store_robot_control.core.grasp_planner import GraspPlanner
            from store_robot_control.core.safety_guard import SafetyGuard

            self._safety_guard = SafetyGuard(self.zones_path)
            self._grasp_planner = GraspPlanner(self.catalog_path)
        except Exception:
            pass

        self.products: dict[str, SimProduct] = {}
        self.containers: dict[str, dict[str, Any]] = {}
        self.active_preset: str = "default_store"
        self.safety_violations: list[str] = []
        self.load_preset("default_store")

    def _default_containers(self) -> dict[str, dict[str, Any]]:
        return {
            "checkout": {
                "container_type": "checkout",
                "valid": True,
                "drop_pose_base": [260.0, -240.0, 70.0],
                "confidence": 0.99,
            },
            "disposal": {
                "container_type": "disposal",
                "valid": True,
                "drop_pose_base": [260.0, 290.0, 70.0],
                "confidence": 0.98,
            },
        }

    def load_preset(self, preset_name: str) -> None:
        """지정된 프리셋으로 가상 매장 상품과 컨테이너를 초기화한다."""
        self.active_preset = preset_name
        self.containers = self._default_containers()
        self.safety_violations.clear()
        self.products.clear()

        if preset_name == "default_store":
            # 확정 3종(포스틱·병음료·아몬드 빼빼로) × 각 2개씩 총 6개 매장/입고 배치
            items = [
                SimProduct("p1", "포스틱", "postick_front", "snack_bag", [300.0, -150.0, 40.0, 0.0],
                           expiration_raw="2027.05.01"),
                SimProduct("p2", "병음료", "drink_front", "bottle", [420.0, 0.0, 140.0, 0.0],
                           expiration_raw="2027.03.01"),
                SimProduct("p3", "아몬드 빼빼로", "pepero_almond_front", "paper_box", [520.0, 150.0, 240.0, 0.0],
                           expiration_raw="2026.12.15"),
                SimProduct("p4", "포스틱", "postick_front", "snack_bag", [330.0, 250.0, 45.0, 0.0],
                           expiration_raw="2027.08.01"),
                SimProduct("p5", "병음료", "drink_front", "bottle", [390.0, 260.0, 45.0, 0.0],
                           is_backside=True, expiration_raw="2027.02.01"),
                SimProduct("p6", "아몬드 빼빼로", "pepero_almond_front", "paper_box", [420.0, -150.0, 140.0, 0.0],
                           true_status="Expired", expiration_raw="2025.01.01", disposal_reason="유통기한 만료"),
            ]
        elif preset_name == "inbound_3tier":
            # 확정 3종 × 각 2개 = 6개 입고 상자 배치 (뒷면 + 2층 가림 포함, disposal[260,290] 회피)
            items = [
                SimProduct("in1", "포스틱", "postick_front", "snack_bag", [310.0, 225.0, 45.0, 0.0]),
                SimProduct("in2", "병음료", "drink_front", "bottle", [360.0, 225.0, 45.0, 0.0]),
                SimProduct("in3", "아몬드 빼빼로", "pepero_almond_front", "paper_box", [410.0, 225.0, 45.0, 0.0], is_backside=True),
                SimProduct("in4", "포스틱", "postick_front", "snack_bag", [410.0, 320.0, 45.0, 0.0]),
                SimProduct("in5", "병음료", "drink_front", "bottle", [355.0, 320.0, 90.0, 0.0], depth_layer=2),
                SimProduct("in6", "아몬드 빼빼로", "pepero_almond_front", "paper_box", [355.0, 320.0, 45.0, 0.0], hidden_under="in5"),
            ]
        elif preset_name == "order_fefo":
            # 동일 종(포스틱 3개) 중 만료 제외 + 유통기한 임박(FEFO: f_soon) 우선 선택 검증
            items = [
                SimProduct("f_late", "포스틱", "postick_front", "snack_bag", [300.0, -150.0, 40.0, 0.0],
                           expiration_raw="2027.12.01"),
                SimProduct("f_soon", "포스틱", "postick_front", "snack_bag", [300.0, 0.0, 40.0, 0.0],
                           expiration_raw="2026.12.01"),
                SimProduct("f_exp", "포스틱", "postick_front", "snack_bag", [300.0, 150.0, 40.0, 0.0],
                           true_status="Expired", expiration_raw="2025.01.01", disposal_reason="유통기한 만료"),
                SimProduct("f_alt", "아몬드 빼빼로", "pepero_almond_front", "paper_box", [420.0, 0.0, 140.0, 0.0],
                           expiration_raw="2027.04.01"),
            ]
        elif preset_name == "tier3_backside":
            items = [
                SimProduct("t_high", "아몬드 빼빼로", "pepero_almond_front", "paper_box", [520.0, 0.0, 240.0, 0.0],
                           is_backside=True, expiration_raw="2027.05.01"),
                SimProduct("t_exp", "포스틱", "postick_front", "snack_bag", [300.0, 0.0, 40.0, 0.0],
                           true_status="Expired", expiration_raw="2025.02.01", disposal_reason="유통기한 만료"),
            ]
        elif preset_name == "all_exceptions":
            items = [
                SimProduct("ex1", "아몬드 빼빼로", "pepero_almond_front", "paper_box", [520.0, 0.0, 240.0, 0.0],
                           is_backside=True, true_status="Expired", disposal_reason="유통기한 만료"),
                SimProduct("ex2", "병음료", "drink_front", "bottle", [420.0, 0.0, 140.0, 0.0],
                           true_status="MarkerContaminated", disposal_reason="마카 오염"),
                SimProduct("ex3", "포스틱", "postick_front", "snack_bag", [350.0, 260.0, 90.0, 0.0],
                           depth_layer=2),
                SimProduct("ex4", "아몬드 빼빼로", "pepero_almond_front", "paper_box", [350.0, 260.0, 45.0, 0.0],
                           hidden_under="ex3"),
                SimProduct("ex5", "모형 담배", "ciga", "ciga", [300.0, 150.0, 40.0, 0.0]),
            ]
        else:
            raise ValueError(f"알 수 없는 프리셋입니다: {preset_name}")

        for item in items:
            self.products[item.track_id] = item

    def list_presets(self) -> list[str]:
        return ["default_store", "inbound_3tier", "order_fefo", "tier3_backside", "all_exceptions"]

    def move_product_mm(
        self,
        track_id: str,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        rz_deg: float | None = None,
    ) -> bool:
        """RViz2 드래그 또는 GUI 입력으로 상품의 3D 좌표를 실시간 갱신한다."""
        prod = self.products.get(track_id)
        if prod is None:
            return False
        new_rz = prod.pose_mm[3] if rz_deg is None else float(rz_deg)
        prod.pose_mm = [float(x_mm), float(y_mm), max(15.0, float(z_mm)), new_rz]
        self._reveal_if_unblocked(track_id)
        return True

    def toggle_backside(self, track_id: str) -> bool:
        prod = self.products.get(track_id)
        if prod is None:
            return False
        prod.is_backside = not prod.is_backside
        return True

    def cycle_status(self, track_id: str) -> str:
        prod = self.products.get(track_id)
        if prod is None:
            return ""
        idx = STATUS_CYCLE.index(prod.true_status) if prod.true_status in STATUS_CYCLE else 0
        prod.true_status = STATUS_CYCLE[(idx + 1) % len(STATUS_CYCLE)]
        prod.disposal_reason = "" if prod.true_status == "Normal" else prod.true_status
        return prod.true_status

    def toggle_depth_layer(self, track_id: str) -> int:
        prod = self.products.get(track_id)
        if prod is None:
            return 1
        prod.depth_layer = 2 if prod.depth_layer == 1 else 1
        return prod.depth_layer

    def _reveal_if_unblocked(self, moved_track_id: str) -> None:
        """상단 물품이 이동하면 그 아래 숨겨져 있던 1층 물품을 노출시킨다."""
        for p in self.products.values():
            if p.hidden_under == moved_track_id:
                p.hidden_under = None

    def scan_visible(self) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
        """현재 카메라 시야에 보이는 상품(컨테이너 투입 완료·가려진 1층 제외)과 컨테이너를 반환한다."""
        visible: list[dict[str, Any]] = []
        checkout_p = self.containers["checkout"]["drop_pose_base"]
        disposal_p = self.containers["disposal"]["drop_pose_base"]

        for prod in self.products.values():
            if prod.hidden_under is not None:
                continue
            # 이미 결제통/폐기통에 들어간 물품은 매대 스캔에서 제외
            if math.hypot(prod.pose_mm[0] - checkout_p[0], prod.pose_mm[1] - checkout_p[1]) < 45.0:
                continue
            if math.hypot(prod.pose_mm[0] - disposal_p[0], prod.pose_mm[1] - disposal_p[1]) < 45.0:
                continue
            visible.append(prod.to_detected_dict())

        return visible, [deepcopy(c) for c in self.containers.values()]

    def find_product_near(self, pose_mm: list[float], tol_mm: float = 65.0) -> SimProduct | None:
        best: SimProduct | None = None
        best_d = tol_mm
        for prod in self.products.values():
            if prod.hidden_under is not None:
                continue
            d = math.hypot(prod.pose_mm[0] - pose_mm[0], prod.pose_mm[1] - pose_mm[1])
            if d <= best_d:
                best_d = d
                best = prod
        return best

    def validate_robot_safety(self, step: RobotActionStep) -> tuple[bool, str]:
        """로봇 파트의 실제 SafetyGuard + GraspPlanner 규격으로 스텝의 물리 안전을 검증한다."""
        if step.action_type in ("home", "scan_pose"):
            return True, "OK"

        if self._safety_guard is not None:
            for pose_label, pose in (("target", step.target_pose), ("dest", step.dest_pose)):
                if pose_label == "dest" and step.action_type == "reorient":
                    continue
                check = self._safety_guard.validate_pose(pose)
                if not check.ok:
                    err_code = str(check.error_code or "INVALID_POSE")
                    self.safety_violations.append(f"{step.action_type}({pose_label}): {err_code}")
                    return False, err_code

        if self._grasp_planner is not None and step.product_category:
            plan = self._grasp_planner.plan_grasp(
                step.product_category,
                step.grasp_width_mm,
                step.grasp_force_n,
            )
            if step.action_type == "reorient" and plan.reorient_mode == "none":
                return False, "INVALID_ACTION"

        if step.action_type == "reorient" and step.target_pose[2] > MAX_REORIENT_Z_MM:
            self.safety_violations.append(f"reorient Z={step.target_pose[2]:.1f} > {MAX_REORIENT_Z_MM}")
            return False, "OUT_OF_REACH"

        return True, "OK"

    def apply_robot_action(self, step: RobotActionStep) -> tuple[bool, str, SimProduct | None]:
        """로봇 액션을 물리 안전 검증 후 가상 월드에 실제 적용한다."""
        ok, err = self.validate_robot_safety(step)
        if not ok:
            return False, err, None

        if step.action_type in ("home", "scan_pose"):
            return True, "OK", None

        prod = self.products.get(step.track_id) if step.track_id else None
        if prod is None:
            prod = self.find_product_near(step.target_pose)
        if prod is None:
            return False, "GRASP_FAIL", None

        if step.action_type in ("pick_place", "move_to_buffer", "restore_from_buffer"):
            prod.pose_mm = [float(v) for v in step.dest_pose[:4]]
            prod.depth_layer = 1
            self._reveal_if_unblocked(prod.track_id)
            return True, "OK", prod

        if step.action_type == "reorient":
            prod.is_backside = False
            return True, "OK", prod

        return False, "INVALID_ACTION", None


@dataclass
class BenchmarkRow:
    """단일 벤치마크 시나리오 채점 결과."""

    name: str
    passed: bool
    steps: int
    rescans: int
    safety_violations: int
    elapsed_ms: float
    detail: str
    actions: list[RobotActionStep] = field(default_factory=list)
    trace: PipelineTrace | None = None


_INTENT_KO = {
    "order": "상품 주문(판매)",
    "query_stock": "재고 확인",
    "inspect": "매대 검수",
    "dispose": "불량 상품 폐기",
    "inbound": "3단 진열대 입고 정리",
    "remote": "원격 제어 전환",
    "clarify": "확인/재질문 필요",
}

_LOC_KO = {
    "inbound_box": "입고 상자",
    "display_area": "진열대 구역",
    "checkout": "결제통",
    "disposal": "폐기통",
    "unknown": "테이블 위",
}


def _loc_label_ko(loc: str) -> str:
    if loc in _LOC_KO:
        return _LOC_KO[loc]
    if loc.startswith("tier_"):
        parts = loc.split("_")
        if len(parts) == 3:
            return f"{parts[1]}단 {int(parts[2]) + 1}번째 자리"
    if loc.startswith("buffer_"):
        return f"임시 보관 자리 {int(loc.split('_')[1]) + 1}"
    return loc


def _describe_subgoal_ko(sg: dict[str, Any]) -> str:
    gtype = sg.get("goal_type", "")
    tid = sg.get("track_id") or ""
    target = sg.get("target") or ""
    if gtype == "in_container":
        dest_ko = "결제통" if target == "checkout" else "폐기통"
        return f"상품({tid})을 {dest_ko}으로 옮기기"
    if gtype == "on_tier_slot":
        return f"상품({tid})을 {_loc_label_ko(target)}에 진열하기"
    if gtype == "front_facing":
        return f"뒷면이 보이는 상품({tid})을 뒤집어 정체 확인하기"
    if gtype == "unblock_layer":
        return f"2층에 얹혀 가리는 상품({tid})을 임시 자리로 치우기"
    if gtype == "buffer_empty":
        return "임시 자리에 빼둔 상품을 원래 자리에 되돌려놓기"
    return f"{gtype} ({tid})"


def _describe_action_ko(act: dict[str, Any]) -> str:
    atype = act.get("action_type", "")
    tid = act.get("track_id", "")
    dest = act.get("dest_id", "")
    purpose = act.get("purpose", "")
    if atype == "pick_place":
        return f"집어서 옮기기: {tid} → {_loc_label_ko(dest)}"
    if atype == "move_to_buffer":
        why = "높이가 높아(200mm 초과) 안전하게 내린 뒤 뒤집기 위해" if purpose == "lower_for_reorient" else "아래 물건을 보기 위해"
        return f"임시 자리로 치우기: {tid} → {_loc_label_ko(dest)} ({why})"
    if atype == "reorient":
        return f"180도 뒤집기: {tid}"
    if atype == "restore_from_buffer":
        return f"원래 자리로 되돌려놓기: {tid} → {_loc_label_ko(dest)}"
    return f"{atype}: {tid}"


def narrate_trace_korean(trace: PipelineTrace) -> str:
    """STOREPLAN 실행 추적 결과를 비개발자도 읽기 쉬운 일상적인 자연어 흐름으로 풀어쓴다."""
    lines: list[str] = []
    src_label = "음성 인식 원문 (Whisper STT)" if trace.source == "voice" else "입력 원문 (텍스트 입력)"
    lines.append(f"[1. {src_label}]")
    lines.append(f'  " {trace.input_text} "')

    if trace.parsed is None:
        lines.append(f"\n[중단] 말을 이해하지 못했습니다: {trace.message}")
        return "\n".join(lines)

    p = trace.parsed
    intent_ko = _INTENT_KO.get(p.intent, p.intent)
    lines.append("\n[2. 온톨로지 + LLM 해석]")
    lines.append(f"  → 파악된 의도: '{intent_ko}'")
    for item in p.items:
        conds = []
        if item.name:
            conds.append(f"이름='{item.name}'")
        if item.brand:
            conds.append(f"브랜드='{item.brand}'")
        if item.color:
            conds.append(f"색상='{item.color}'")
        if item.taste:
            conds.append(f"맛='{item.taste}'")
        if item.category:
            conds.append(f"종류='{item.category}'")
        if item.exclude:
            conds.append(f"제외={item.exclude}")
        cond_str = ", ".join(conds) if conds else "전체"
        lines.append(f"  → 찾으시는 조건: {cond_str} (수량 {item.quantity}개)")
    if trace.catalog_candidates:
        cand_names = [str(c.get("name", "")) for c in trace.catalog_candidates if c.get("name")]
        if cand_names:
            lines.append(f"  → 온톨로지 매칭 상품: {', '.join(cand_names)}")
    if p.sort_by:
        order_ko = "낮은/작은 순" if p.sort_order != "desc" else "높은/큰 순"
        lines.append(f"  → 정렬 기준: '{p.sort_by}' ({order_ko})")

    if trace.needs_clarification:
        if trace.clarification_reason:
            lines.append(f"  → 판단 근거: {trace.clarification_reason}")
        if trace.inferred_candidates:
            lines.append(f"  → 온톨로지 추론 후보: {', '.join(trace.inferred_candidates)}")
        else:
            lines.append("  → 온톨로지 추론 후보: 없음 (매장 카탈로그와 무관)")
        lines.append("\n[확인 질문 (로봇 동작 없이 대기)]")
        lines.append(f'  🤖 "{trace.clarification_question}"')
        return "\n".join(lines)

    mode_ko = {"CUSTOMER": "손님", "EMPLOYEE": "직원", "REMOTE": "원격 관리자"}.get(trace.mode, trace.mode)
    lines.append(f"\n[3. 권한 확인 (현재 모드: {mode_ko})]")
    if not trace.allowed:
        lines.append(f"  → 실행 불가: {trace.message}")
        return "\n".join(lines)
    lines.append("  → 권한 확인 통과, 매장 작업을 시작합니다.")

    for c in trace.cycles:
        cnum = c.get("cycle", 1)
        lines.append(f"\n[4-{cnum}. 매대 살펴보기 & 계획 세우기 ({cnum}번째 확인)]")
        observed = c.get("observed", [])
        if not observed:
            lines.append("  • 현재 매대에 보이는 상품이 없습니다.")
        else:
            lines.append(f"  • 눈에 보이는 상품 ({len(observed)}개):")
            for obj in observed:
                name = "정체 모를 뒷면" if obj["is_backside"] else (obj["name"] or obj["track_id"])
                st = "뒷면(뒤집기 필요)" if obj["is_backside"] else (
                    "정상" if obj["status"] == "Normal" else f"불량({obj['status']})"
                )
                layer_txt = ", 2층 위" if obj["depth_layer"] == 2 else ""
                z_txt = f", 높이 {int(obj['z_mm'])}mm" if obj.get("z_mm") is not None else ""
                lines.append(f"    - {obj['track_id']}: {name} [{st}, {_loc_label_ko(obj['location'])}{layer_txt}{z_txt}]")

        reasoning = c.get("reasoning")
        if reasoning:
            lines.append(f"  • 판단 근거: {reasoning}")

        subgoals = c.get("subgoals", [])
        if subgoals:
            lines.append("  • 세운 목표 순서:")
            for idx, sg in enumerate(subgoals, 1):
                lines.append(f"    {idx}) {_describe_subgoal_ko(sg)}")
        elif c.get("unfulfillable_reason"):
            lines.append(f"  • 목표를 세울 수 없는 이유: {c['unfulfillable_reason']}")

        if c.get("safety_stop_reason"):
            lines.append(f"  • 안전 정지: {c['safety_stop_reason']}")
            continue

        planned = c.get("planned_actions", [])
        if planned:
            lines.append("  • 이번 차례 실행 계획:")
            for idx, act in enumerate(planned, 1):
                lines.append(f"    {idx}) {_describe_action_ko(act)}")
            if c.get("needs_rescan"):
                lines.append("  • 뒤집기/치우기 이후에는 눈으로 다시 확인해야 하므로 여기까지 실행 후 재확인합니다.")

    lines.append("\n[5. 최종 도출된 로봇 행동 리스트]")
    if not trace.executed_actions:
        lines.append("  (생성된 로봇 행동 없음)")
    else:
        for idx, step in enumerate(trace.executed_actions, 1):
            lines.append(
                f"  {idx:2d}. {step.action_type:20s} | 대상={step.track_id or '-':6s} | "
                f"종류={step.product_category or '-':10s} | {step.reason}"
            )

    status_ko = "성공" if trace.success else ("안전 정지" if trace.safety_stop else "미완료/차단")
    lines.append(f"\n[결론] {status_ko} — {trace.message}")
    return "\n".join(lines)


def build_live_engine(
    world: SimWorld,
    employee_mode: bool = False,
) -> StorePipelineEngine:
    """실제 GPT-4o(1차 LLMParser + 2차 TaskFormalizer)와 SimWorld를 연결한 엔진을 생성한다."""
    kg = KGManager()
    kg.load_initial_graph(world.ttl_path, world.catalog_path, zones_yaml=world.zones_path)
    fsm = ModeFSM(required_vest_frames=1)
    if employee_mode:
        fsm.update_vest_detection(True, 0.99, now=1.0)

    domain = ActionDomain(
        display_tiers=world.display_tiers,
        buffer_slots=world.buffer_slots,
        zone_bounds=world.zone_bounds,
    )
    return StorePipelineEngine(
        kg_manager=kg,
        mode_fsm=fsm,
        llm_parser=LLMParser(),
        task_formalizer=TaskFormalizer(),
        action_domain=domain,
        symbolic_planner=SymbolicPlanner(),
        scan_fn=lambda _z, _v: (True, "OK", *world.scan_visible()),
        execute_action_fn=lambda step: world.apply_robot_action(step)[:2],
        display_tiers=world.display_tiers,
    )


def run_benchmark(
    mode: str = "offline",
    config_dir: str | Path | None = None,
) -> list[BenchmarkRow]:
    """STOREPLAN + SimWorld + 로봇 SafetyGuard 기반 CLI 벤치마크 세트를 일괄 실행한다."""
    rows: list[BenchmarkRow] = []
    world = SimWorld(config_dir=config_dir)

    def _run_case(
        name: str,
        preset: str,
        mode_employee: bool,
        text: str,
        parsed: OrderConditions,
        specs: list[TaskSpec],
        post_check,
    ) -> None:
        world.load_preset(preset)
        kg = KGManager()
        kg.load_initial_graph(world.ttl_path, world.catalog_path, zones_yaml=world.zones_path)
        fsm = ModeFSM(required_vest_frames=1)
        if mode_employee:
            fsm.update_vest_detection(True, 0.99, now=1.0)

        if mode == "live":
            llm_parser = LLMParser()
            task_formalizer = TaskFormalizer()
        else:
            p_client = SimpleNamespace(
                beta=SimpleNamespace(
                    chat=SimpleNamespace(
                        completions=SimpleNamespace(
                            parse=lambda **_: SimpleNamespace(
                                choices=[SimpleNamespace(message=SimpleNamespace(parsed=parsed))]
                            )
                        )
                    )
                )
            )
            spec_idx = {"i": 0}

            def _f_parse(**_):
                cur = min(spec_idx["i"], len(specs) - 1)
                spec_idx["i"] += 1
                return SimpleNamespace(choices=[SimpleNamespace(message=SimpleNamespace(parsed=specs[cur]))])

            f_client = SimpleNamespace(
                beta=SimpleNamespace(chat=SimpleNamespace(completions=SimpleNamespace(parse=_f_parse)))
            )
            llm_parser = LLMParser(client=p_client)
            task_formalizer = TaskFormalizer(client=f_client)

        domain = ActionDomain(
            display_tiers=world.display_tiers,
            buffer_slots=world.buffer_slots,
            zone_bounds=world.zone_bounds,
        )
        engine = StorePipelineEngine(
            kg_manager=kg,
            mode_fsm=fsm,
            llm_parser=llm_parser,
            task_formalizer=task_formalizer,
            action_domain=domain,
            symbolic_planner=SymbolicPlanner(),
            scan_fn=lambda _z, _v: (True, "OK", *world.scan_visible()),
            execute_action_fn=lambda step: world.apply_robot_action(step)[:2],
            display_tiers=world.display_tiers,
        )
        t0 = time.perf_counter()
        trace = engine.handle_command(text, source=f"benchmark:{mode}")
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        rescans = sum(1 for a in trace.executed_actions if a.action_type == "scan_pose")
        ok, detail = post_check(trace, world)
        rows.append(
            BenchmarkRow(
                name=name,
                passed=ok and len(world.safety_violations) == 0,
                steps=len(trace.executed_actions),
                rescans=rescans,
                safety_violations=len(world.safety_violations),
                elapsed_ms=elapsed_ms,
                detail=detail,
                actions=list(trace.executed_actions),
                trace=trace,
            )
        )

    # 1. 주문 FEFO 우선 선택 (동일 종 포스틱 3개 중 만료 제외 + 유통기한 임박 우선)
    _run_case(
        "1. 주문 FEFO (유통기한 임박 우선)",
        "order_fefo",
        False,
        "포스틱 하나 줘",
        OrderConditions(intent="order", items=[ItemCondition(name="포스틱", quantity=1)]),
        [TaskSpec(intent="order", subgoals=[Subgoal(goal_type="in_container", track_id="f_soon", target="checkout")])],
        lambda tr, w: (
            tr.success and w.products["f_soon"].pose_mm[:2] == [260.0, -240.0],
            "f_soon 결제통 이동 완료" if tr.success else tr.message,
        ),
    )

    # 2. 3단(Z=240mm) 뒷면 자동 버퍼 하강 → reorient → 원위치 복원
    _run_case(
        "2. 3단(Z=240mm) 뒷면 버퍼 경유 뒤집기",
        "tier3_backside",
        True,
        "매대 검수해줘",
        OrderConditions(intent="inspect"),
        [
            TaskSpec(
                intent="inspect",
                subgoals=[
                    Subgoal(goal_type="in_container", track_id="t_exp", target="disposal"),
                    Subgoal(goal_type="front_facing", track_id="t_high"),
                ],
            ),
            TaskSpec(intent="inspect", subgoals=[Subgoal(goal_type="buffer_empty")]),
        ],
        lambda tr, w: (
            tr.success
            and (not w.products["t_high"].is_backside)
            and w.products["t_high"].pose_mm[:3] == [520.0, 0.0, 240.0]
            and w.products["t_exp"].pose_mm[:2] == [260.0, 290.0],
            "폐기+하강+뒤집기+3단복원 완료" if tr.success else tr.message,
        ),
    )

    # 3. 3종(포스틱·병음료·아몬드 빼빼로) 입고 3단 진열 (앞면 + 뒷면 + 2층 가림 해제)
    _run_case(
        "3. 3단 입고 정렬 + 뒷면 + 2층 노출",
        "inbound_3tier",
        True,
        "단맛 낮은 순으로 3단 진열대 정리해줘",
        OrderConditions(intent="inbound", sort_by="sweetness", sort_order="asc"),
        [
            TaskSpec(
                intent="inbound",
                subgoals=[
                    Subgoal(goal_type="on_tier_slot", track_id="in1", target="tier_1_0"),
                    Subgoal(goal_type="on_tier_slot", track_id="in2", target="tier_2_0"),
                    Subgoal(goal_type="on_tier_slot", track_id="in4", target="tier_1_1"),
                    Subgoal(goal_type="on_tier_slot", track_id="in5", target="tier_2_1"),
                ],
            ),
            TaskSpec(
                intent="inbound",
                subgoals=[
                    Subgoal(goal_type="on_tier_slot", track_id="in6", target="tier_3_0"),
                    Subgoal(goal_type="front_facing", track_id="in3"),
                ],
            ),
            TaskSpec(
                intent="inbound",
                subgoals=[Subgoal(goal_type="on_tier_slot", track_id="in3", target="tier_3_1")],
            ),
        ],
        lambda tr, w: (
            tr.success
            and w.products["in1"].pose_mm[2] == 40.0
            and w.products["in4"].pose_mm[2] == 40.0
            and w.products["in2"].pose_mm[2] == 140.0
            and w.products["in5"].pose_mm[2] == 140.0
            and w.products["in3"].pose_mm[2] == 240.0
            and w.products["in6"].pose_mm[2] == 240.0
            and (not w.products["in3"].is_backside)
            and len({tuple(p.pose_mm[:2]) for p in w.products.values()}) == 6,
            "6개 상품 3단 종별 배치+뒷면+2층 완료" if tr.success else tr.message,
        ),
    )

    # 4. 담배 원격 인증 자동 차단
    _run_case(
        "4. 모형 담배 원격 인증 차단",
        "all_exceptions",
        False,
        "모형 담배 하나 줘",
        OrderConditions(intent="order", items=[ItemCondition(name="모형 담배", quantity=1)]),
        [TaskSpec(intent="order")],
        lambda tr, _w: (not tr.allowed and len(tr.executed_actions) == 0, tr.message),
    )

    # 5. 모호한 발화 → 온톨로지 다중 후보 확인질문 ("과자" -> Snack 하위 포스틱·아몬드 빼빼로 2종, 병음료 제외)
    _run_case(
        "5. 모호 발화 → 온톨로지 확인질문",
        "default_store",
        False,
        "과자 하나 줘",
        OrderConditions(
            intent="clarify",
            items=[ItemCondition(category="과자", quantity=1)],
            candidate_names=["아몬드 빼빼로", "포스틱"],
            clarification_reason="과자류에 해당하는 상품이 여러 종입니다.",
            clarification_question="과자 상품에는 포스틱, 아몬드 빼빼로가 있습니다. 어떤 상품을 드릴까요?",
        ),
        [TaskSpec(intent="clarify")],
        lambda tr, _w: (
            tr.needs_clarification
            and len(tr.executed_actions) == 0
            and set(tr.inferred_candidates) >= {"아몬드 빼빼로", "포스틱"},
            tr.clarification_question or tr.message,
        ),
    )

    # 6. 이해 불가 발화 → 억지 정정 없이 단순 역질문
    _run_case(
        "6. 이해 불가 발화 → 단순 역질문",
        "default_store",
        False,
        "기가 막힐 toys",
        OrderConditions(
            intent="clarify",
            candidate_names=[],
            clarification_reason="매장 카탈로그와 무관하거나 이해할 수 없는 발화입니다.",
            clarification_question="말씀하신 내용을 이해하지 못했습니다. 상품명이나 원하시는 작업을 다시 말씀해 주세요.",
        ),
        [TaskSpec(intent="clarify")],
        lambda tr, _w: (
            tr.needs_clarification
            and len(tr.executed_actions) == 0
            and len(tr.inferred_candidates) == 0,
            tr.clarification_question or tr.message,
        ),
    )

    return rows


def run_offline_benchmark(config_dir: str | Path | None = None) -> list[BenchmarkRow]:
    """기존 호환용 오프라인 벤치마크 진입점."""
    return run_benchmark(mode="offline", config_dir=config_dir)
