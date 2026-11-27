from store_llm_agent.core.inbound_planner import plan_lifo_restore
from store_llm_agent.core.kg_manager import build_sparql_query
from store_llm_agent.core.mode_fsm import ModeFSM, StoreMode
from store_llm_agent.core.rule_validator import decide_product_action


def test_mode_fsm_customer_blocks_employee_intents():
    fsm = ModeFSM(StoreMode.CUSTOMER)
    assert fsm.is_intent_allowed('order') is True
    assert fsm.is_intent_allowed('dispose') is False
    fsm.update_from_vest(True, consecutive_frames=5)
    assert fsm.mode == StoreMode.EMPLOYEE
    assert fsm.is_intent_allowed('dispose') is True


def test_rule_validator_deterministic():
    assert decide_product_action('Expired', requested=True) == 'disposal_bin'
    assert decide_product_action('MarkerContaminated', requested=False) == 'disposal_bin'
    assert decide_product_action('Normal', requested=True) == 'checkout_bin'
    assert decide_product_action('Normal', requested=False) == 'keep'


def test_sparql_template():
    q = build_sparql_query(category='snack', exclude_flavor='chocolate')
    assert 'FILTER(LCASE(?category) = "snack")' in q
    assert 'FILTER(LCASE(?flavor) != "chocolate")' in q


def test_lifo_restore():
    stack = [{'id': 'p1'}, {'id': 'p2'}]
    assert [x['id'] for x in plan_lifo_restore(stack)] == ['p2', 'p1']
