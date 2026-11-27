"""OntoPlan Scene Explorer 에이전트 — OpenAI Function Calling으로 KGManager의 Ask(SPARQL) 도구를 선택 호출."""

from __future__ import annotations

from dataclasses import dataclass, field
import json
from pathlib import Path
import time
from typing import Any

from dotenv import load_dotenv
from openai import OpenAI

from store_llm_agent.core.kg_manager import KGManager
from store_llm_agent.core.llm_parser import OrderConditions
from store_llm_agent.core.paths import find_workspace_root

_EXPLORER_SYSTEM_PROMPT = """\
너는 24시 무인편의점 협동로봇 판단 엔진(OntoPlan/STOREPLAN)의 Scene Explorer 에이전트다.
사용자의 명령(OrderConditions)을 처리하기 위해, 지식 그래프(KGManager)의 Ask(SPARQL) 도구를 선택적으로 호출하여
작업에 필요한 실물 근거(판매 가능 재고, 폐기/뒤집기 OWL 추론 대상, 구역/단 위치, 대체 상품, 컨테이너 상태)만 정확히 수집한다.

규칙:
- 필요한 Ask 도구를 가급적 한 번에 병렬로 호출해 불필요한 왕복을 줄여라.
- order: ask_sellable_items와 ask_containers(필요 시 ask_ontology_relations("disposal_candidates") 또는 ask_alternatives)를 조회하라.
- inspect / dispose: ask_ontology_relations("disposal_candidates"), ask_ontology_relations("needs_reorient"), ask_containers를 조회하라.
- inbound: ask_ontology_relations("in_zone", zone="inbound_box"), ask_ontology_relations("in_zone", zone="display_area"), ask_ontology_relations("needs_reorient"), ask_ontology_relations("blocking_pairs")를 조회하라.
- 모든 근거를 수집했으면 한국어로 2~3문장 이내의 간결한 현장 요약만 반환하라.
"""

_EXPLORER_TOOLS: list[dict[str, Any]] = [
    {
        "type": "function",
        "function": {
            "name": "ask_sellable_items",
            "description": "현재 Normal 앞면 상태인 판매 가능 상품 인스턴스를 FEFO(유통기한 임박) 순으로 SPARQL 조회한다.",
            "parameters": {
                "type": "object",
                "properties": {
                    "name": {"type": "string", "description": "상품명 필터"},
                    "brand": {"type": "string", "description": "브랜드 필터"},
                    "color": {"type": "string", "description": "색상 필터"},
                    "taste": {"type": "string", "description": "맛 필터"},
                    "category": {"type": "string", "description": "카테고리 필터"},
                    "exclude": {
                        "type": "array",
                        "items": {"type": "string"},
                        "description": "제외 키워드 목록",
                    },
                },
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "ask_ontology_relations",
            "description": "OWL 2 RL 전방 추론 결과 및 공간/속성 관계를 SPARQL로 조회한다.",
            "parameters": {
                "type": "object",
                "properties": {
                    "relation_type": {
                        "type": "string",
                        "enum": [
                            "disposal_candidates",
                            "needs_reorient",
                            "remote_restricted",
                            "in_zone",
                            "on_tier",
                            "same_brand",
                            "same_taste",
                            "same_color",
                            "blocking_pairs",
                            "by_class",
                        ],
                    },
                    "name": {"type": "string", "description": "same_brand/same_taste/same_color 기준 상품명"},
                    "zone": {"type": "string", "description": "in_zone 대상 (display_area, inbound_box, buffer)"},
                    "tier": {"type": "string", "description": "on_tier 대상 (tier_1, tier_2, tier_3)"},
                    "class_name": {"type": "string", "description": "by_class 대상 OWL 클래스명"},
                },
                "required": ["relation_type"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "ask_alternatives",
            "description": "요청 상품이 불량/품절일 때 동일 맛·브랜드·카테고리의 Normal 대체 상품을 조회한다.",
            "parameters": {
                "type": "object",
                "properties": {
                    "name": {"type": "string"},
                    "category": {"type": "string"},
                    "taste": {"type": "string"},
                    "exclude": {"type": "array", "items": {"type": "string"}},
                },
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "ask_containers",
            "description": "결제통(checkout)과 폐기통(disposal)의 유효성 및 좌표를 조회한다.",
            "parameters": {"type": "object", "properties": {}},
        },
    },
]


@dataclass
class SceneExplorerResult:
    """Scene Explorer가 KGManager Ask 도구를 호출해 수집한 근거 및 요약."""

    summary: str = ""
    tool_calls: list[dict[str, Any]] = field(default_factory=list)
    evidence: dict[str, Any] = field(default_factory=dict)
    elapsed_ms: float = 0.0

    def to_prompt_dict(self) -> dict[str, Any]:
        return {
            "summary": self.summary,
            "queried_tools": [f"{tc['tool']}({tc['args']})" for tc in self.tool_calls],
            "evidence": self.evidence,
        }


def _dispatch_ask_tool(kg: KGManager, tool_name: str, args: dict[str, Any]) -> Any:
    if tool_name == "ask_sellable_items":
        slots = {
            k: args[k]
            for k in ("name", "brand", "color", "taste", "category", "exclude")
            if args.get(k)
        }
        return kg.query_sellable_by_slots(slots)
    if tool_name == "ask_ontology_relations":
        rel_type = str(args.get("relation_type", ""))
        params = {k: v for k, v in args.items() if k != "relation_type" and v}
        return kg.query_by_relation(rel_type, **params)
    if tool_name == "ask_alternatives":
        return kg.query_alternatives(args)
    if tool_name == "ask_containers":
        return kg.get_containers()
    return {"error": f"unknown tool: {tool_name}"}


class SceneExplorer:
    """GPT-4o Tool Calling으로 지식 그래프(Ask)를 탐색해 TaskFormalizer에 근거를 전달한다."""

    def __init__(
        self,
        client: Any | None = None,
        model: str = "gpt-4o",
        max_tool_turns: int = 3,
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
        self.max_tool_turns = max_tool_turns
        self._client = client

    @property
    def client(self) -> Any:
        if self._client is None:
            self._client = OpenAI()
        return self._client

    def explore(self, parsed: OrderConditions, kg: KGManager) -> SceneExplorerResult:
        """OrderConditions에 맞춰 KGManager의 Ask 도구를 호출하고 수집된 근거를 요약한다."""
        t0 = time.perf_counter()
        messages: list[dict[str, Any]] = [
            {"role": "system", "content": _EXPLORER_SYSTEM_PROMPT},
            {
                "role": "user",
                "content": json.dumps(
                    {"order_conditions": parsed.model_dump()},
                    ensure_ascii=False,
                ),
            },
        ]
        recorded_calls: list[dict[str, Any]] = []
        evidence: dict[str, Any] = {}
        summary = ""

        for _ in range(self.max_tool_turns):
            resp = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                tools=_EXPLORER_TOOLS,
                tool_choice="auto",
                temperature=0.0,
            )
            msg = resp.choices[0].message
            tool_calls = getattr(msg, "tool_calls", None) or []
            if not tool_calls:
                summary = str(getattr(msg, "content", "") or "").strip()
                break

            assistant_msg: dict[str, Any] = {
                "role": "assistant",
                "content": getattr(msg, "content", None) or "",
                "tool_calls": [
                    {
                        "id": tc.id,
                        "type": "function",
                        "function": {
                            "name": tc.function.name,
                            "arguments": tc.function.arguments or "{}",
                        },
                    }
                    for tc in tool_calls
                ],
            }
            messages.append(assistant_msg)

            for tc in tool_calls:
                tname = tc.function.name
                try:
                    targs = json.loads(tc.function.arguments or "{}")
                except Exception:
                    targs = {}
                result = _dispatch_ask_tool(kg, tname, targs)
                recorded_calls.append({"tool": tname, "args": targs, "count": len(result) if isinstance(result, (list, dict)) else 1})
                ev_key = tname if tname != "ask_ontology_relations" else f"rel:{targs.get('relation_type', '')}"
                evidence[ev_key] = result
                messages.append({
                    "role": "tool",
                    "tool_call_id": tc.id,
                    "content": json.dumps(result, ensure_ascii=False),
                })

        if not summary:
            summary = f"Ask 도구 {len(recorded_calls)}회 호출로 온톨로지 근거 수집 완료"

        return SceneExplorerResult(
            summary=summary,
            tool_calls=recorded_calls,
            evidence=evidence,
            elapsed_ms=(time.perf_counter() - t0) * 1000.0,
        )
