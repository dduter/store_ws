"""LLMParser 단위 테스트 (Mock OpenAI 클라이언트)."""

from types import SimpleNamespace

import pytest

from store_llm_agent.core.llm_parser import ItemCondition, LLMParser, OrderConditions


class _MockCompletions:
    def __init__(self, parsed_obj: OrderConditions) -> None:
        self._parsed = parsed_obj
        self.last_kwargs: dict = {}

    def parse(self, **kwargs):
        self.last_kwargs = kwargs
        msg = SimpleNamespace(parsed=self._parsed)
        return SimpleNamespace(choices=[SimpleNamespace(message=msg)])


class _MockOpenAIClient:
    def __init__(self, parsed_obj: OrderConditions) -> None:
        self.completions = _MockCompletions(parsed_obj)
        self.beta = SimpleNamespace(chat=SimpleNamespace(completions=self.completions))


def test_parse_intent_order_with_exclude() -> None:
    expected = OrderConditions(
        intent="order",
        items=[ItemCondition(category="과자", exclude=["초코"], quantity=2)],
    )
    mock_client = _MockOpenAIClient(expected)
    parser = LLMParser(client=mock_client)

    res = parser.parse_intent(
        "초코 빼고 과자 두 개 줘",
        catalog_candidates=[{"name": "포스틱"}, {"name": "아몬드 빼빼로"}],
    )

    assert res.intent == "order"
    assert res.raw_text == "초코 빼고 과자 두 개 줘"
    assert len(res.items) == 1
    assert res.items[0].quantity == 2
    assert res.items[0].to_slot_dict() == {
        "name": None,
        "brand": None,
        "color": None,
        "taste": None,
        "category": "과자",
        "exclude": ["초코"],
    }
    sys_prompt = mock_client.completions.last_kwargs["messages"][0]["content"]
    assert "포스틱" in sys_prompt


def test_parse_intent_empty_raises() -> None:
    parser = LLMParser(client=_MockOpenAIClient(OrderConditions(intent="order")))
    with pytest.raises(ValueError):
        parser.parse_intent("   ")


def test_parse_intent_includes_catalog_metadata() -> None:
    mock_client = _MockOpenAIClient(OrderConditions(intent="order"))
    parser = LLMParser(client=mock_client)
    parser.parse_intent(
        "파란색 과자 줘",
        catalog_candidates=[
            {"name": "포스틱", "category": "봉지과자", "taste": "짠맛(감자)", "color": "파랑", "brand": "농심"},
        ],
    )
    sys_prompt = mock_client.completions.last_kwargs["messages"][0]["content"]
    assert "포스틱(봉지과자/짠맛(감자)/파랑/농심)" in sys_prompt
