"""OpenAI GPT-4o Structured Output 기반 자연어 의도·슬롯 파서."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Literal

from dotenv import load_dotenv
from openai import OpenAI
from pydantic import BaseModel, Field

from store_llm_agent.core.paths import find_workspace_root

IntentType = Literal[
    "order",
    "inspect",
    "dispose",
    "inbound",
    "query_stock",
    "remote",
    "clarify",
]

_SYSTEM_PROMPT = """\
너는 24시 무인편의점 협동로봇의 자연어 명령 파서다.
사용자의 발화를 분석하여 intent와 items 슬롯을 정확하게 JSON 구조로 추출한다.

intent 분류 기준:
- order: 고객이 상품 구매/전달을 요청할 때 (예: "포스틱 하나 줘", "빼빼로 줘", "병음료 두 개 줘", "단 거 말고 과자 하나 줘")
- query_stock: 재고/상품 유무 문의 (예: "포스틱 남았어?", "음료 있어?")
- inspect: 직원용 매대/상품 검수 지시 (예: "매대 유통기한 검사해줘")
- dispose: 직원용 불량/만료 상품 폐기 지시 (예: "상한 거 폐기통으로 치워줘")
- inbound: 직원용 입고 상품 3단 진열대 정리/배치 요청 (예: "단맛 순서대로 정리해줘", "카테고리별로 1단부터 배치해줘")
- remote: 원격 제어 전환 요청
- clarify: 발화가 모호하거나 이해할 수 없어 바로 로봇을 움직이지 말고 사용자에게 확인/재질문해야 할 때

명확성 판단 & 역질문(clarify) 원칙 (매우 중요):
- 절대 아무 단어나 소음/무관한 문장을 억지로 상품명으로 끼워 맞추지 마라.
  * 예: "기가 막힐 toys", "오늘 날씨 어때", 의미 불명 텍스트 → 반드시 intent="clarify", candidate_names=[], clarification_question="말씀하신 내용을 정확히 이해하지 못했습니다. 찾으시는 상품명(포스틱, 아몬드 빼빼로, 병음료)이나 원하시는 작업을 다시 말씀해 주세요."
- 발음/단어가 카탈로그 공식 상품명과 완전히 일치하지 않지만 온톨로지 카탈로그에서 유사/연관 상품이 추론되는 경우(예: "코스틱 줘", "빼빼루 줘"):
  * 임의로 order를 확정하지 말고 intent="clarify"로 둔 뒤, candidate_names에 추론된 후보 상품명을 넣고 clarification_question으로 사용자에게 확인질문을 한다 (예: "말씀하신 상품이 '포스틱'이(가) 맞나요?").
- 단일 상품 주문(수량 1개, 특정 상품명 미지정, exclude 없음, '아무거나' 미명시)인데 온톨로지 속성(예: "과자 하나 줘")에 해당하는 상품이 2종 이상('포스틱', '아몬드 빼빼로')인 경우:
  * intent="clarify", candidate_names=["포스틱", "아몬드 빼빼로"], clarification_question에 후보들을 안내하며 어느 상품을 원하는지 확인질문한다.
- 반대로 다음 경우는 되묻지 말고 즉시 order로 실행한다:
  1) 공식 상품명 또는 명확한 통칭일 때: "포스틱", "아몬드 빼빼로"(또는 "빼빼로" -> name="아몬드 빼빼로"), "병음료"(또는 "음료"/"음료수" -> name="병음료"), "모형 담배"
  2) 속성으로 온톨로지상 유일한 1종이 특정될 때 (예: "파란색 과자"·"농심 거"·"봉지과자"·"짠 과자" -> 포스틱 / "초록색 과자"·"롯데 거"·"박스 과자"·"단 과자"·"초콜릿 과자"·"아몬드 과자" -> 아몬드 빼빼로 / "병음료"·"음료" -> 병음료)
  3) "빼고/제외"(exclude) 조건, 2개 이상 복수 수량, 또는 "아무거나"가 명시된 조건부 주문일 때 (예: "초콜릿 빼고 과자 두 개 줘")
- 직전 대화 맥락(previous_clarification)이 주어지고 사용자가 "응", "맞아", "포스틱", "빼빼로", "첫 번째 거" 등으로 응답했다면, 이전 맥락과 결합해 확정 intent(예: order)로 파싱한다.

기타 규칙:
- "빼고", "말고", "제외" 표현은 해당 item의 exclude 리스트에 넣는다.
- 수량 언급이 없으면 quantity는 1로 둔다.
- 명시되지 않은 속성(brand, color, taste 등)은 null로 둔다.
- 입고/정리 시 정렬 기준("단맛 순"->sweetness, "카테고리별"->category, "크기 순"->size, "유통기한 순"->expiration)이 있으면 sort_by에, 방향("오름차순/낮은순/아래부터" -> asc, "내림차순/높은순/큰것부터" -> desc)이 있으면 sort_order에 추출한다.
"""


class ItemCondition(BaseModel):
    """단일 요청 상품의 검색 조건 슬롯."""

    name: str | None = Field(default=None, description="상품명 (포스틱, 아몬드 빼빼로, 병음료, 모형 담배)")
    brand: str | None = Field(default=None, description="브랜드명 (예: 농심, 롯데)")
    color: str | None = Field(default=None, description="포장 색상 (예: 파랑, 초록)")
    taste: str | None = Field(default=None, description="맛 (예: 짠맛, 감자, 단맛, 초콜릿, 아몬드)")
    category: str | None = Field(default=None, description="카테고리 (예: 봉지과자, 종이상자, 병음료, 과자, 담배)")
    exclude: list[str] = Field(default_factory=list, description="제외할 키워드 목록")
    quantity: int = Field(default=1, ge=1, description="요청 수량")

    def to_slot_dict(self) -> dict[str, Any]:
        """KGManager SPARQL 조회 메서드에 전달할 슬롯 딕셔너리로 변환한다."""
        return {
            "name": self.name,
            "brand": self.brand,
            "color": self.color,
            "taste": self.taste,
            "category": self.category,
            "exclude": list(self.exclude),
        }


class OrderConditions(BaseModel):
    """자연어 발화에서 추출한 전체 의도, 상품 조건 및 정렬 방향성 키워드."""

    intent: IntentType
    items: list[ItemCondition] = Field(default_factory=list)
    sort_by: str | None = Field(default=None, description="정렬 기준 (예: sweetness, size, expiration, category)")
    sort_order: Literal["asc", "desc"] | None = Field(default=None, description="정렬 방향 (asc 또는 desc)")
    clarification_question: str = Field(
        default="",
        description="intent=='clarify'일 때 사용자에게 되물을 확인 질문 또는 재요청 질문",
    )
    clarification_reason: str = Field(
        default="",
        description="모호함 또는 온톨로지 추론 근거 요약",
    )
    candidate_names: list[str] = Field(
        default_factory=list,
        description="온톨로지 관계 추론으로 연결된 후보 상품명 목록",
    )
    raw_text: str = ""


class LLMParser:
    """GPT-4o Structured Output을 사용해 자연어를 OrderConditions로 파싱한다."""

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

    def parse_intent(
        self,
        text: str,
        catalog_candidates: list[dict[str, Any]] | None = None,
        conversation_context: str = "",
    ) -> OrderConditions:
        """자연어 발화를 OrderConditions 모델로 파싱한다."""
        cleaned = (text or "").strip()
        if not cleaned:
            raise ValueError("파싱할 입력 텍스트가 비어 있습니다.")

        system_msg = _SYSTEM_PROMPT
        if catalog_candidates:
            entries: list[str] = []
            for c in catalog_candidates:
                name = c.get("name", "")
                if not name:
                    continue
                meta = [str(c[k]) for k in ("category", "taste", "color", "brand") if c.get(k)]
                entries.append(f"{name}({'/'.join(meta)})" if meta else name)
            if entries:
                system_msg += f"\n현재 매장 취급 상품 참고 목록(온톨로지): {', '.join(entries)}"
        if conversation_context:
            system_msg += f"\n직전 확인질문 맥락(previous_clarification): {conversation_context}"

        completion = self.client.beta.chat.completions.parse(
            model=self.model,
            messages=[
                {"role": "system", "content": system_msg},
                {"role": "user", "content": cleaned},
            ],
            response_format=OrderConditions,
            temperature=0.0,
        )
        parsed = completion.choices[0].message.parsed
        if parsed is None:
            raise RuntimeError("LLM Structured Output 파싱 결과가 비어 있습니다.")
        parsed.raw_text = cleaned
        return parsed
