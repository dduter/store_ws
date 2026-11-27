"""STOREPLAN 2차 LLM Task Formalizer — OrderConditions + WorldState를 검증 가능한 TaskSpec(Subgoal 리스트)로 변환."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Literal

from dotenv import load_dotenv
from openai import OpenAI
from pydantic import BaseModel, Field

from store_llm_agent.core.action_domain import ActionDomain, WorldState
from store_llm_agent.core.llm_parser import IntentType, OrderConditions
from store_llm_agent.core.paths import find_workspace_root

SubgoalType = Literal[
    "in_container",
    "on_tier_slot",
    "front_facing",
    "unblock_layer",
    "buffer_empty",
]

_FORMALIZER_SYSTEM_PROMPT = """\
너는 24시 무인편의점 협동로봇 판단 엔진 STOREPLAN의 Task Formalizer(2차 LLM)다.
1차 파싱 결과(OrderConditions)와 현재 관측된 매장 상태(WorldState)를 바탕으로,
Symbolic Planner가 실행할 목표 상태(Subgoal) 리스트(TaskSpec)를 정확하게 생성한다.

사용 가능한 Subgoal 타입:
1. in_container: track_id 상품을 target("checkout" 또는 "disposal") 컨테이너로 이동
2. on_tier_slot: track_id 상품을 3단 진열대 슬롯(target: "tier_1_0" ~ "tier_3_2")에 배치
3. front_facing: 뒷면(is_backside=true 또는 NeedsReorientation)인 track_id 상품을 앞면으로 뒤집기 (target=null)
4. unblock_layer: 2층 방해물(depth_layer=2)인 track_id 상품을 버퍼로 임시 이동해 하단 물품 노출 (target=null)
5. buffer_empty: 버퍼 스택에 남아 있는 임시 이동 상품을 LIFO 원위치 복원 (track_id=null, target=null)

절대 규칙 (위반 금지):
- 반드시 observed_products에 실제로 존재하는 track_id와, free_tier_slots 또는 "checkout"/"disposal"만 사용한다. 없는 ID를 만들어내지 마라.
- [폐기 절대 금지 규칙] status == "Normal" 이고 is_backside == false 인 정상 상품은 reorient_count가 1 이상이더라도 절대 disposal로 보내지 마라!
  * 뒤집기(reorient)를 거쳐 앞면이 확인된 정상 상품은 reorient_count==1이지만 정상(Normal)이다.
  * 뒷면 상품 폐기는 오직 is_backside == true 이면서 동시에 reorient_count >= 1 일 때만 허용된다.
- intent == "order":
  * 요청 조건(name/brand/color/taste/category/exclude)에 맞는 status=="Normal" & is_backside==false 상품 중 expiration_raw가 가장 빠른 것(FEFO)부터 quantity개만큼 in_container(track_id, "checkout")를 생성한다.
  * 요청 조건에 맞는 상품이 Expired/MarkerContaminated/Unreadable/Unjudgeable이면 in_container(track_id, "disposal")를 생성하고, 부족한 수량은 동일 조건(같은 맛/카테고리 우선)의 Normal 대체품으로 in_container(alt_id, "checkout")를 채운다.
  * 수량이 모자라고 매대에 reorient_count < 1인 뒷면 상품이 있으면 front_facing(track_id)를 추가한다.
  * 어떤 상품/대체품도 찾을 수 없으면 subgoals=[]로 두고 unfulfillable_reason에 사유를 적는다.
- intent in ("inspect", "dispose"):
  * 불량 상품(status가 Expired/MarkerContaminated/Unreadable/Unjudgeable인 앞면 상품, 또는 is_backside==true 이면서 reorient_count>=1인 뒷면 상품)에 대해서만 in_container(track_id, "disposal")를 생성한다.
  * is_backside==true 이고 reorient_count < 1인 뒷면 상품은 front_facing(track_id)로 뒤쪽에 배치한다.
  * buffer_stack에 상품이 있고 그 상품이 Normal 앞면으로 확인되었다면 폐기하지 말고 buffer_empty만 추가해 원래 자리로 복원한다.
- intent == "inbound":
  * 매장 취급 상품은 3종(포스틱·아몬드 빼빼로·병음료) × 각 최대 3개 = 총 9개이며, 3단 진열대의 각 단(tier_1, tier_2, tier_3, 단당 3슬롯)에 동일 종끼리 모아서 진열한다.
  * payload에 제공된 recommended_tier_mapping(예: {"tier_1": "포스틱", "tier_2": "병음료", "tier_3": "아몬드 빼빼로"})을 반드시 그대로 따른다. 임의로 단 순서를 바꾸지 마라.
  * 이미 자신의 목표 단(예: tier_1_* / tier_2_* / tier_3_*)에 올바르게 진열된 상품은 절대 다시 옮기지 마라! 아직 inbound_box나 buffer_*에 남아 있는 상품만 해당 단의 free_tier_slots에 on_tier_slot으로 배치한다.
  * inbound_box에 있는 앞면 Normal 상품은 depth_layer==2이더라도 unblock_layer를 쓰지 말고 바로 목표 단에 on_tier_slot으로 진열한다(2층 상품을 진열대로 옮기면 아래 1층 상품이 자동으로 노출된다). 불량품은 in_container(track_id, "disposal"), 뒷면 상품의 front_facing은 뒤쪽에 배치한다.
  * unblock_layer와 buffer_empty는 전체 입고 정리가 아니라 특정 단일 상품만 꺼내야 할 때(목표가 안 보이고 다른 상품이 2층을 가릴 때)만 사용한다. 전체 입고 정리 중에는 buffer_*에 있는 Normal 상품도 buffer_empty로 되돌리지 말고 목표 단에 on_tier_slot으로 진열한다.
- reasoning과 모든 subgoal의 reason은 반드시 간결한 한국어로만 작성한다 (영어 문장 금지).
"""


class Subgoal(BaseModel):
    """Symbolic Planner가 달성해야 할 단일 목표 상태 술어."""

    goal_type: SubgoalType
    track_id: str | None = Field(default=None, description="대상 상품 track_id (buffer_empty는 null)")
    target: str | None = Field(
        default=None,
        description="목표 위치 (in_container: checkout|disposal, on_tier_slot: tier_1_0~tier_3_2)",
    )
    reason: str = Field(default="", description="서브골 생성 사유")


class TaskSpec(BaseModel):
    """2차 LLM이 형식화한 전체 작업 목표 명세."""

    intent: IntentType
    subgoals: list[Subgoal] = Field(default_factory=list)
    reasoning: str = Field(default="", description="목표 분해 및 우선순위 판단 근거 요약")
    unfulfillable_reason: str | None = Field(default=None, description="목표 수립 불가 사유 (품절 등)")


def compute_recommended_tier_mapping(
    catalog: list[dict[str, Any]],
    sort_by: str | None,
    sort_order: str | None,
    tier_names: list[str] | None = None,
) -> dict[str, str]:
    """입고 정렬 기준(sweetness/category/size/name)에 따라 3종 상품을 tier_1~tier_3에 1종씩 결정론적으로 매핑한다."""
    sellable = [c for c in catalog if not c.get("requires_remote_auth", False) and c.get("name")]
    if not sellable:
        return {}

    key_name = (sort_by or "sweetness").lower()
    reverse = (sort_order or "asc").lower() == "desc"

    if key_name in ("sweetness", "sweetness_rank", "단맛"):
        ordered = sorted(
            sellable,
            key=lambda c: (int(c.get("sweetness_rank", 0)), str(c.get("name", ""))),
            reverse=reverse,
        )
    elif key_name in ("category", "카테고리", "종류"):
        cat_rank = {"paper_box": 0, "snack_bag": 1, "bottle": 2, "candy_chocolate": 3}
        ordered = sorted(
            sellable,
            key=lambda c: (
                cat_rank.get(str(c.get("robot_category") or c.get("category") or ""), 99),
                str(c.get("name", "")),
            ),
            reverse=reverse,
        )
    elif key_name in ("size", "size_mm", "크기"):
        ordered = sorted(
            sellable,
            key=lambda c: (float(c.get("size_mm", 0.0)), str(c.get("name", ""))),
            reverse=reverse,
        )
    else:
        ordered = sorted(sellable, key=lambda c: str(c.get("name", "")), reverse=reverse)

    tiers = tier_names or ["tier_1", "tier_2", "tier_3"]
    return {tier: str(item["name"]) for tier, item in zip(tiers, ordered)}


def sanitize_task_spec(
    spec: TaskSpec,
    world_state: WorldState,
    domain: ActionDomain,
) -> tuple[TaskSpec, list[str]]:
    """2차 LLM 출력(TaskSpec)의 환각 ID·잘못된 슬롯·중복 슬롯 배정을 결정론적으로 검증·정화한다."""
    warnings: list[str] = []
    valid_track_ids = {p.track_id for p in world_state.products}
    valid_tier_slots = set(domain.all_tier_slot_ids)
    used_tier_slots = set(s for s in world_state.occupied_slots if s.startswith("tier_"))
    sanitized_subgoals: list[Subgoal] = []

    for sg in spec.subgoals:
        if sg.goal_type == "buffer_empty":
            sanitized_subgoals.append(
                Subgoal(goal_type="buffer_empty", track_id=None, target=None, reason=sg.reason)
            )
            continue

        if not sg.track_id or sg.track_id not in valid_track_ids:
            warnings.append(f"존재하지 않는 track_id '{sg.track_id}' 서브골 제거")
            continue

        if sg.goal_type == "in_container":
            if sg.target not in ("checkout", "disposal"):
                warnings.append(f"유효하지 않은 컨테이너 '{sg.target}' 서브골 제거")
                continue
            sanitized_subgoals.append(sg)
            continue

        if sg.goal_type == "on_tier_slot":
            target_slot = sg.target or ""
            if target_slot not in valid_tier_slots:
                warnings.append(f"유효하지 않은 진열 슬롯 '{target_slot}' 서브골 제거")
                continue
            if target_slot in used_tier_slots:
                # 같은 단(tier_k) 내 빈 슬롯으로 자동 보정
                tier_prefix = target_slot.rsplit("_", 1)[0] + "_"
                replacement = next(
                    (
                        s for s in domain.all_tier_slot_ids
                        if s.startswith(tier_prefix) and s not in used_tier_slots
                    ),
                    None,
                )
                if replacement is None:
                    warnings.append(f"진열 단 '{tier_prefix[:-1]}'에 빈 슬롯이 없어 '{sg.track_id}' 서브골 제거")
                    continue
                warnings.append(f"중복 슬롯 '{target_slot}' → '{replacement}' 자동 보정")
                target_slot = replacement
            used_tier_slots.add(target_slot)
            sanitized_subgoals.append(
                Subgoal(
                    goal_type="on_tier_slot",
                    track_id=sg.track_id,
                    target=target_slot,
                    reason=sg.reason,
                )
            )
            continue

        if sg.goal_type in ("front_facing", "unblock_layer"):
            sanitized_subgoals.append(
                Subgoal(goal_type=sg.goal_type, track_id=sg.track_id, target=None, reason=sg.reason)
            )

    cleaned = TaskSpec(
        intent=spec.intent,
        subgoals=sanitized_subgoals,
        reasoning=spec.reasoning,
        unfulfillable_reason=spec.unfulfillable_reason,
    )
    return cleaned, warnings


class TaskFormalizer:
    """GPT-4o Structured Output을 사용해 OrderConditions + WorldState를 TaskSpec으로 형식화한다."""

    def __init__(
        self,
        client: Any | None = None,
        model: str = "gpt-4o",
        env_path: str | Path | None = None,
    ) -> None:
        if env_path is not None:
            load_dotenv(dotenv_path=str(env_path))
        else:
            default_env = find_workspace_root(__file__) / ".env"
            if default_env.exists():
                load_dotenv(dotenv_path=str(default_env))
            else:
                load_dotenv()

        self.model = model
        self._client = client

    @property
    def client(self) -> Any:
        if self._client is None:
            self._client = OpenAI()
        return self._client

    def formalize(
        self,
        parsed: OrderConditions,
        world_state: WorldState,
        catalog: list[dict[str, Any]],
        domain: ActionDomain,
    ) -> tuple[TaskSpec, list[str]]:
        """2차 LLM을 호출해 TaskSpec을 생성하고 Schema Sanitizer로 검증한다."""
        payload: dict[str, Any] = {
            "order_conditions": parsed.model_dump(),
            "catalog": catalog,
            "world_state": world_state.to_prompt_dict(all_tier_slots=domain.all_tier_slot_ids),
        }
        if parsed.intent == "inbound":
            tier_names = sorted({s.rsplit("_", 1)[0] for s in domain.all_tier_slot_ids})
            payload["recommended_tier_mapping"] = compute_recommended_tier_mapping(
                catalog=catalog,
                sort_by=parsed.sort_by,
                sort_order=parsed.sort_order,
                tier_names=tier_names,
            )
        completion = self.client.beta.chat.completions.parse(
            model=self.model,
            messages=[
                {"role": "system", "content": _FORMALIZER_SYSTEM_PROMPT},
                {"role": "user", "content": json.dumps(payload, ensure_ascii=False)},
            ],
            response_format=TaskSpec,
            temperature=0.0,
        )
        raw_spec = completion.choices[0].message.parsed
        if raw_spec is None:
            raise RuntimeError("TaskFormalizer LLM Structured Output 결과가 비어 있습니다.")
        return sanitize_task_spec(raw_spec, world_state, domain)
