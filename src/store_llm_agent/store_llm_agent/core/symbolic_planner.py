"""STOREPLAN Symbolic Planner — Subgoal별 BFS 최단 경로 탐색 및 부분 관측성(4-A) 재스캔 분할."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import time

from store_llm_agent.core.action_domain import (
    ActionDomain,
    RobotActionStep,
    SymbolicAction,
    WorldState,
)
from store_llm_agent.core.task_formalizer import Subgoal, TaskSpec


@dataclass
class PlanResult:
    """SymbolicPlanner 1사이클 탐색 결과."""

    actions: list[RobotActionStep] = field(default_factory=list)
    symbolic_actions: list[SymbolicAction] = field(default_factory=list)
    final_state: WorldState | None = None
    needs_rescan: bool = False
    safety_stop: bool = False
    stop_reason: str = ""
    disposals: list[dict[str, str]] = field(default_factory=list)
    solved_subgoals: int = 0
    total_subgoals: int = 0
    search_ms: float = 0.0


def is_subgoal_satisfied(state: WorldState, subgoal: Subgoal) -> bool:
    """현재 WorldState가 단일 Subgoal을 이미 만족하는지 판정한다."""
    if subgoal.goal_type == "buffer_empty":
        return len(state.buffer_stack) == 0

    if not subgoal.track_id:
        return False
    prod = state.get_product(subgoal.track_id)
    if prod is None:
        return False

    if subgoal.goal_type in ("in_container", "on_tier_slot"):
        return prod.location == subgoal.target
    if subgoal.goal_type == "front_facing":
        return (not prod.is_backside) and (prod.status != "NeedsReorientation")
    if subgoal.goal_type == "unblock_layer":
        return prod.location.startswith("buffer_")
    return False


class SymbolicPlanner:
    """ActionDomain(Pre/Eff) 위에서 TaskSpec의 Subgoal별 BFS 탐색을 수행한다."""

    def __init__(self, max_depth: int = 6, max_nodes: int = 500) -> None:
        self.max_depth = max_depth
        self.max_nodes = max_nodes

    def _candidate_actions(
        self,
        state: WorldState,
        subgoal: Subgoal,
        domain: ActionDomain,
    ) -> list[SymbolicAction]:
        """서브골 달성과 직접 관련된 grounded SymbolicAction 후보만 생성해 분기 폭발을 방지한다."""
        if subgoal.goal_type == "buffer_empty":
            if not state.buffer_stack:
                return []
            top = state.buffer_stack[-1]
            return [
                SymbolicAction(
                    action_type="restore_from_buffer",
                    track_id=top.track_id,
                    dest_id=top.original_location,
                    needs_rescan=False,
                    reason=subgoal.reason or f"임시 보관 상품 LIFO 복원 ({top.track_id} -> {top.original_location})",
                )
            ]

        if not subgoal.track_id:
            return []
        prod = state.get_product(subgoal.track_id)
        if prod is None:
            return []

        if subgoal.goal_type in ("in_container", "on_tier_slot"):
            dest = subgoal.target or ""
            # 입고 상자 2층 물품을 빼내면 아래 1층이 새로 드러날 수 있으므로 재스캔 표시
            rescan = (prod.location == "inbound_box" and prod.depth_layer == 2)
            return [
                SymbolicAction(
                    action_type="pick_place",
                    track_id=prod.track_id,
                    dest_id=dest,
                    needs_rescan=rescan,
                    reason=subgoal.reason or f"{prod.name or prod.track_id} -> {dest}",
                )
            ]

        if subgoal.goal_type == "front_facing":
            candidates: list[SymbolicAction] = [
                SymbolicAction(
                    action_type="reorient",
                    track_id=prod.track_id,
                    needs_rescan=True,
                    reason=subgoal.reason or f"뒷면 감지({prod.track_id}) → 180도 뒤집기",
                )
            ]
            # 고정 Flip Zone이 설정된 경우 pick_place(-> flip_zone, rz=0°) 선행 후보 추가
            if domain.flip_zone_pose is not None and prod.location != "flip_zone":
                candidates.append(
                    SymbolicAction(
                        action_type="pick_place",
                        track_id=prod.track_id,
                        dest_id="flip_zone",
                        needs_rescan=False,
                        reason=f"고정 Flip Zone 이동(rz=0°) 후 뒤집기 ({prod.track_id})",
                    )
                )
            # Flip Zone 미사용 시 Z_surface > 200mm인 경우 낮은 버퍼 슬롯으로 먼저 하강 후 reorient하는 경로 후보 추가
            elif prod.pose_valid and prod.pose[2] > domain.max_reorient_z_mm:
                low_buf = domain.first_free_buffer_slot(state, max_z_mm=domain.max_reorient_z_mm)
                if low_buf is not None:
                    candidates.append(
                        SymbolicAction(
                            action_type="move_to_buffer",
                            track_id=prod.track_id,
                            dest_id=low_buf,
                            purpose="lower_for_reorient",
                            needs_rescan=False,
                            reason=f"높이 초과(Z={prod.pose[2]:.0f}mm>{domain.max_reorient_z_mm:.0f}mm) → {low_buf} 하강 후 뒤집기",
                        )
                    )
            return candidates

        if subgoal.goal_type == "unblock_layer":
            free_buf = domain.first_free_buffer_slot(state)
            if free_buf is None:
                return []
            return [
                SymbolicAction(
                    action_type="move_to_buffer",
                    track_id=prod.track_id,
                    dest_id=free_buf,
                    purpose="unblock",
                    needs_rescan=True,
                    reason=subgoal.reason or f"2층 방해물 임시 이동 ({prod.track_id} -> {free_buf})",
                )
            ]

        return []

    def _diagnose_failure(
        self,
        state: WorldState,
        subgoal: Subgoal,
        domain: ActionDomain,
    ) -> str:
        """BFS가 경로를 찾지 못했을 때 가장 정확한 SAFETY_STOP 사유를 반환한다."""
        if subgoal.goal_type == "unblock_layer" and domain.first_free_buffer_slot(state) is None:
            return "SAFETY_STOP: 임시 보관 버퍼 슬롯이 모두 찼습니다."
        if subgoal.goal_type == "front_facing" and subgoal.track_id:
            prod = state.get_product(subgoal.track_id)
            if prod and prod.pose_valid and prod.pose[2] > domain.max_reorient_z_mm:
                if domain.first_free_buffer_slot(state, max_z_mm=domain.max_reorient_z_mm) is None:
                    return (
                        f"SAFETY_STOP: '{prod.track_id}' 높이(Z={prod.pose[2]:.1f}mm)가 reorient 상한을 초과했으나 "
                        "하강 가능한 빈 버퍼 슬롯이 없습니다."
                    )

        for cand in self._candidate_actions(state, subgoal, domain):
            ok, reason = domain.check_precondition(state, cand)
            if not ok:
                return reason if reason.startswith("SAFETY_STOP") else f"SAFETY_STOP: {reason}"

        return f"SAFETY_STOP: 서브골({subgoal.goal_type}, track={subgoal.track_id}, target={subgoal.target}) 경로를 찾지 못했습니다."

    def _bfs_subgoal(
        self,
        start_state: WorldState,
        subgoal: Subgoal,
        domain: ActionDomain,
    ) -> list[SymbolicAction] | None:
        """단일 Subgoal에 대해 최단 SymbolicAction 시퀀스를 BFS로 탐색한다."""
        if is_subgoal_satisfied(start_state, subgoal):
            return []

        queue: deque[tuple[WorldState, list[SymbolicAction]]] = deque([(start_state, [])])
        visited: set[WorldState] = {start_state}
        expanded = 0

        while queue and expanded < self.max_nodes:
            cur_state, path = queue.popleft()
            if len(path) >= self.max_depth:
                continue
            expanded += 1

            for cand in self._candidate_actions(cur_state, subgoal, domain):
                ok, _ = domain.check_precondition(cur_state, cand)
                if not ok:
                    continue
                next_state = domain.apply(cur_state, cand)
                next_path = [*path, cand]
                if is_subgoal_satisfied(next_state, subgoal):
                    return next_path
                if next_state not in visited:
                    visited.add(next_state)
                    queue.append((next_state, next_path))

        return None

    def plan(
        self,
        initial_state: WorldState,
        task_spec: TaskSpec,
        domain: ActionDomain,
    ) -> PlanResult:
        """TaskSpec의 모든 Subgoal을 순차 BFS로 풀고, needs_rescan(4-A) 지점에서 분할 반환한다."""
        t0 = time.perf_counter()
        cur_state = initial_state
        robot_steps: list[RobotActionStep] = []
        sym_steps: list[SymbolicAction] = []
        disposals: list[dict[str, str]] = []
        solved = 0
        total = len(task_spec.subgoals)

        for subgoal in task_spec.subgoals:
            if is_subgoal_satisfied(cur_state, subgoal):
                solved += 1
                continue

            sub_path = self._bfs_subgoal(cur_state, subgoal, domain)
            if sub_path is None:
                stop_reason = self._diagnose_failure(cur_state, subgoal, domain)
                return PlanResult(
                    actions=robot_steps,
                    symbolic_actions=sym_steps,
                    final_state=cur_state,
                    safety_stop=True,
                    stop_reason=stop_reason,
                    disposals=disposals,
                    solved_subgoals=solved,
                    total_subgoals=total,
                    search_ms=(time.perf_counter() - t0) * 1000.0,
                )

            for sym_act in sub_path:
                step, err = domain.ground_to_robot_step(cur_state, sym_act)
                if step is None:
                    return PlanResult(
                        actions=robot_steps,
                        symbolic_actions=sym_steps,
                        final_state=cur_state,
                        safety_stop=True,
                        stop_reason=err,
                        disposals=disposals,
                        solved_subgoals=solved,
                        total_subgoals=total,
                        search_ms=(time.perf_counter() - t0) * 1000.0,
                    )

                if sym_act.action_type == "pick_place" and sym_act.dest_id == "disposal":
                    prod = cur_state.get_product(sym_act.track_id)
                    reason = (
                        (prod.disposal_reason if prod and prod.disposal_reason else "")
                        or (prod.status if prod else "")
                        or "Unjudgeable"
                    )
                    disposals.append({"track_id": sym_act.track_id, "reason": reason})

                robot_steps.append(step)
                sym_steps.append(sym_act)
                cur_state = domain.apply(cur_state, sym_act)

                # 4-A 즉시 분할: reorient / 2층 제거 등 재스캔이 필요한 액션 직후 플랜 분할
                if sym_act.needs_rescan:
                    return PlanResult(
                        actions=robot_steps,
                        symbolic_actions=sym_steps,
                        final_state=cur_state,
                        needs_rescan=True,
                        safety_stop=False,
                        disposals=disposals,
                        solved_subgoals=solved + (1 if is_subgoal_satisfied(cur_state, subgoal) else 0),
                        total_subgoals=total,
                        search_ms=(time.perf_counter() - t0) * 1000.0,
                    )

            solved += 1

        return PlanResult(
            actions=robot_steps,
            symbolic_actions=sym_steps,
            final_state=cur_state,
            needs_rescan=False,
            safety_stop=False,
            disposals=disposals,
            solved_subgoals=solved,
            total_subgoals=total,
            search_ms=(time.perf_counter() - t0) * 1000.0,
        )
